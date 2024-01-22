/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 Eiren Rain & SlimeVR contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN EspNowCONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "esp_now.h"

#include <WifiEspNow.h>

#include "GlobalVars.h"
#include "featureflags.h"
#include "globals.h"
#include "logging/Logger.h"

// TODO: Cleanup with proper classes
SlimeVR::Logging::Logger espNowLogger("ESP-NOW");

// The recipient MAC address. It must be modified for each device.
static uint8_t PEER[]{0x2E, 0x3A, 0xE8, 0x05, 0xFC, 0x4C};

void onPacket(
	const uint8_t mac[WIFIESPNOW_ALEN],
	const uint8_t* buf,
	size_t count,
	void* arg
) {
	networkConnection.onPacket(buf, count);
}

void EspNow::setUp() {
	espNowLogger.info("Setting up ESP-NOW");

	WiFi.persistent(false);
	WiFi.mode(WIFI_AP);
	WiFi.disconnect();
	WiFi.softAP("ESPNOW", nullptr, 3);
	WiFi.softAPdisconnect(false);

	espNowLogger.info("MAC address of this node is %s", WiFi.softAPmacAddress());

	uint8_t mac[6];
	WiFi.softAPmacAddress(mac);
	espNowLogger.debug(
		"\nYou can paste the following into the program for the other device:\nstatic "
		"uint8_t PEER[]{0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};\n",
		mac[0],
		mac[1],
		mac[2],
		mac[3],
		mac[4],
		mac[5]
	);

	bool ok = WifiEspNow.begin();
	if (!ok) {
		espNowLogger.fatal("WifiEspNow.begin() failed");
		return;
	}

	WifiEspNow.onReceive(onPacket, nullptr);

	ok = WifiEspNow.addPeer(PEER);
	if (!ok) {
		espNowLogger.fatal("WifiEspNow.addPeer() failed");
		return;
	}

	networkConnection.reset();
}

#define TIMEOUT 3000UL

template <typename T>
unsigned char* convert_to_chars(T src, unsigned char* target) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	un.v = src;
	for (size_t i = 0; i < sizeof(T); i++) {
		target[i] = un.c[sizeof(T) - i - 1];
	}
	return target;
}

// why???
template <typename T>
T convert_chars(unsigned char* const src) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	for (size_t i = 0; i < sizeof(T); i++) {
		un.c[i] = src[sizeof(T) - i - 1];
	}
	return un.v;
}

namespace SlimeVR {
namespace Network {

#define MUST_TRANSFER_BOOL(b) \
	if (!b)                   \
		return false;

#define MUST(b) \
	if (!b)     \
		return;

bool EspNowConnection::beginPacket() {
	if (m_IsBundle) {
		m_BundlePacketPosition = 0;
		return true;
	}

	m_OutPacketPosition = 0;

	return true;
}

bool EspNowConnection::endPacket() {
	if (m_IsBundle) {
		uint32_t innerPacketSize = m_BundlePacketPosition;

		MUST_TRANSFER_BOOL((innerPacketSize > 0));

		m_IsBundle = false;

		if (m_BundlePacketInnerCount == 0) {
			sendPacketType(PACKET_BUNDLE);
			sendPacketNumber();
		}
		sendShort(innerPacketSize);
		sendBytes(m_Packet, innerPacketSize);

		m_BundlePacketInnerCount++;
		m_IsBundle = true;
		return true;
	}

	int r = WifiEspNow.send(
		PEER,
		reinterpret_cast<const uint8_t*>(m_OutPacket),
		m_OutPacketPosition
	);
	// m_Logger.info("Sent packet len %i", m_OutPacketPosition);
	if (r == 0) {
		m_Logger.warn("UDP endPacket() failed");
	}

	return r > 0;
}

bool EspNowConnection::beginBundle() {
	MUST_TRANSFER_BOOL(m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT));
	MUST_TRANSFER_BOOL(m_Connected);
	MUST_TRANSFER_BOOL(!m_IsBundle);
	MUST_TRANSFER_BOOL(beginPacket());

	m_IsBundle = true;
	m_BundlePacketInnerCount = 0;
	return true;
}

bool EspNowConnection::endBundle() {
	MUST_TRANSFER_BOOL(m_IsBundle);

	m_IsBundle = false;

	MUST_TRANSFER_BOOL((m_BundlePacketInnerCount > 0));

	return endPacket();
}

size_t EspNowConnection::write(const uint8_t* buffer, size_t size) {
	if (m_IsBundle) {
		if (m_BundlePacketPosition + size > sizeof(m_Packet)) {
			return 0;
		}
		memcpy(m_Packet + m_BundlePacketPosition, buffer, size);
		m_BundlePacketPosition += size;
		return size;
	}

	if (m_OutPacketPosition + size > sizeof(m_OutPacket)) {
		return 0;
	}
	memcpy(m_OutPacket + m_OutPacketPosition, buffer, size);
	m_OutPacketPosition += size;
	return size;
}

size_t EspNowConnection::write(uint8_t byte) { return write(&byte, 1); }

bool EspNowConnection::sendFloat(float f) {
	convert_to_chars(f, m_Buf);

	return write(m_Buf, sizeof(f)) != 0;
}

bool EspNowConnection::sendByte(uint8_t c) { return write(&c, 1) != 0; }

bool EspNowConnection::sendShort(uint16_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool EspNowConnection::sendInt(uint32_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool EspNowConnection::sendLong(uint64_t l) {
	convert_to_chars(l, m_Buf);

	return write(m_Buf, sizeof(l)) != 0;
}

bool EspNowConnection::sendBytes(const uint8_t* c, size_t length) {
	return write(c, length) != 0;
}

bool EspNowConnection::sendPacketNumber() {
	if (m_IsBundle) {
		return true;
	}

	uint64_t pn = m_PacketNumber++;

	return sendLong(pn);
}

bool EspNowConnection::sendShortString(const char* str) {
	uint8_t size = strlen(str);

	MUST_TRANSFER_BOOL(sendByte(size));
	MUST_TRANSFER_BOOL(sendBytes((const uint8_t*)str, size));

	return true;
}

bool EspNowConnection::sendPacketType(uint8_t type) {
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));

	return sendByte(type);
}

bool EspNowConnection::sendLongString(const char* str) {
	int size = strlen(str);

	MUST_TRANSFER_BOOL(sendInt(size));

	return sendBytes((const uint8_t*)str, size);
}

int EspNowConnection::getWriteError() { return 0; }

// PACKET_HEARTBEAT 0
void EspNowConnection::sendHeartbeat() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HEARTBEAT));
	MUST(sendPacketNumber());

	MUST(endPacket());
}

// PACKET_ACCEL 4
void EspNowConnection::sendSensorAcceleration(uint8_t sensorId, Vector3 vector) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ACCEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(vector.x));
	MUST(sendFloat(vector.y));
	MUST(sendFloat(vector.z));
	MUST(sendByte(sensorId));

	MUST(endPacket());
}

// PACKET_BATTERY_LEVEL 12
void EspNowConnection::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_BATTERY_LEVEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(batteryVoltage));
	MUST(sendFloat(batteryPercentage));

	MUST(endPacket());
}

// PACKET_TAP 13
void EspNowConnection::sendSensorTap(uint8_t sensorId, uint8_t value) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TAP));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(value));

	MUST(endPacket());
}

// PACKET_ERROR 14
void EspNowConnection::sendSensorError(uint8_t sensorId, uint8_t error) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ERROR));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(error));

	MUST(endPacket());
}

// PACKET_SENSOR_INFO 15
void EspNowConnection::sendSensorInfo(Sensor* sensor) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SENSOR_INFO));
	MUST(sendPacketNumber());
	MUST(sendByte(sensor->getSensorId()));
	MUST(sendByte((uint8_t)sensor->getSensorState()));
	MUST(sendByte(sensor->getSensorType()));

	MUST(endPacket());
}

// PACKET_ROTATION_DATA 17
void EspNowConnection::sendRotationData(
	uint8_t sensorId,
	Quat* const quaternion,
	uint8_t dataType,
	uint8_t accuracyInfo
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ROTATION_DATA));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(dataType));
	MUST(sendFloat(quaternion->x));
	MUST(sendFloat(quaternion->y));
	MUST(sendFloat(quaternion->z));
	MUST(sendFloat(quaternion->w));
	MUST(sendByte(accuracyInfo));

	MUST(endPacket());
}

// PACKET_MAGNETOMETER_ACCURACY 18
void EspNowConnection::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_MAGNETOMETER_ACCURACY));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(accuracyInfo));

	MUST(endPacket());
}

// PACKET_SIGNAL_STRENGTH 19
void EspNowConnection::sendSignalStrength(uint8_t signalStrength) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SIGNAL_STRENGTH));
	MUST(sendPacketNumber());
	MUST(sendByte(255));
	MUST(sendByte(signalStrength));

	MUST(endPacket());
}

// PACKET_TEMPERATURE 20
void EspNowConnection::sendTemperature(uint8_t sensorId, float temperature) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TEMPERATURE));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(temperature));

	MUST(endPacket());
}

// PACKET_FEATURE_FLAGS 22
void EspNowConnection::sendFeatureFlags() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_FEATURE_FLAGS));
	MUST(sendPacketNumber());
	MUST(write(FirmwareFeatures::flags.data(), FirmwareFeatures::flags.size()));

	MUST(endPacket());
}

void EspNowConnection::sendTrackerDiscovery() {
	MUST(!m_Connected);

	uint8_t mac[6];
	WiFi.macAddress(mac);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HANDSHAKE));
	// Packet number is always 0 for handshake
	MUST(sendLong(0));
	MUST(sendInt(BOARD));
	// This is kept for backwards compatibility,
	// but the latest SlimeVR server will not initialize trackers
	// with firmware build > 8 until it recieves a sensor info packet
	MUST(sendInt(IMU));
	MUST(sendInt(HARDWARE_MCU));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(FIRMWARE_BUILD_NUMBER));
	MUST(sendShortString(FIRMWARE_VERSION));
	// MAC address string
	MUST(sendBytes(mac, 6));

	MUST(endPacket());
}

#if ENABLE_INSPECTION
void EspNowConnection::sendInspectionRawIMUData(
	uint8_t sensorId,
	int16_t rX,
	int16_t rY,
	int16_t rZ,
	uint8_t rA,
	int16_t aX,
	int16_t aY,
	int16_t aZ,
	uint8_t aA,
	int16_t mX,
	int16_t mY,
	int16_t mZ,
	uint8_t mA
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_INT));

	MUST(sendInt(rX));
	MUST(sendInt(rY));
	MUST(sendInt(rZ));
	MUST(sendByte(rA));

	MUST(sendInt(aX));
	MUST(sendInt(aY));
	MUST(sendInt(aZ));
	MUST(sendByte(aA));

	MUST(sendInt(mX));
	MUST(sendInt(mY));
	MUST(sendInt(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}

void EspNowConnection::sendInspectionRawIMUData(
	uint8_t sensorId,
	float rX,
	float rY,
	float rZ,
	uint8_t rA,
	float aX,
	float aY,
	float aZ,
	uint8_t aA,
	float mX,
	float mY,
	float mZ,
	uint8_t mA
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_FLOAT));

	MUST(sendFloat(rX));
	MUST(sendFloat(rY));
	MUST(sendFloat(rZ));
	MUST(sendByte(rA));

	MUST(sendFloat(aX));
	MUST(sendFloat(aY));
	MUST(sendFloat(aZ));
	MUST(sendByte(aA));

	MUST(sendFloat(mX));
	MUST(sendFloat(mY));
	MUST(sendFloat(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}
#endif

void EspNowConnection::returnLastPacket(int len) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendBytes(m_Packet, len));

	MUST(endPacket());
}

void EspNowConnection::updateSensorState(std::vector<Sensor*>& sensors) {
	if (millis() - m_LastSensorInfoPacketTimestamp <= 1000) {
		return;
	}

	m_LastSensorInfoPacketTimestamp = millis();

	for (int i = 0; i < (int)sensors.size(); i++) {
		if (m_AckedSensorState[i] != sensors[i]->getSensorState()) {
			sendSensorInfo(sensors[i]);
		}
	}
}

void EspNowConnection::maybeRequestFeatureFlags() {
	if (m_ServerFeatures.isAvailable() || m_FeatureFlagsRequestAttempts >= 15) {
		return;
	}

	if (millis() - m_FeatureFlagsRequestTimestamp < 500) {
		return;
	}

	sendFeatureFlags();
	m_FeatureFlagsRequestTimestamp = millis();
	m_FeatureFlagsRequestAttempts++;
}

void EspNowConnection::searchForServer() {
	auto now = millis();

	// This makes the LED blink for 20ms every second
	if (m_LastConnectionAttemptTimestamp + 1000 < now) {
		m_LastConnectionAttemptTimestamp = now;
		m_Logger.info("Searching for the server on the local network...");
		EspNowConnection::sendTrackerDiscovery();
		ledManager.on();
	} else if (m_LastConnectionAttemptTimestamp + 20 < now) {
		ledManager.off();
	}
}

void EspNowConnection::reset() {
	m_Connected = false;
	std::fill(
		m_AckedSensorState,
		m_AckedSensorState + MAX_IMU_COUNT,
		SensorStatus::SENSOR_OFFLINE
	);

	// m_UDP.begin(m_ServerPort);

	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

void EspNowConnection::update() {
	std::vector<Sensor*>& sensors = sensorManager.getSensors();

	updateSensorState(sensors);
	maybeRequestFeatureFlags();

	if (!m_Connected) {
		searchForServer();
		return;
	}

	if (m_LastPacketTimestamp + TIMEOUT < millis()) {
		statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

		m_Connected = false;
		std::fill(
			m_AckedSensorState,
			m_AckedSensorState + MAX_IMU_COUNT,
			SensorStatus::SENSOR_OFFLINE
		);
		m_Logger.warn("Connection to server timed out");

		return;
	}
}

void EspNowConnection::onPacket(const uint8_t* buf, size_t len) {
	m_LastPacketTimestamp = millis();

	std::copy(buf, buf + len, m_Packet);

	// #ifdef DEBUG_NETWORK
	// m_Logger.info("Received %d bytes", len);
	// m_Logger.infoArray("UDP packet contents: ", m_Packet, len);
	// m_Logger.info("Packet header %d", convert_chars<int>(m_Packet));
	// m_Logger.info("Packet header %d", m_Packet[0]);
	// m_Logger.info(
	// 	"Connecting %d",
	// 	statusManager.hasStatus(SlimeVR::Status::SERVER_CONNECTING)
	// );
	// #endif

	if (statusManager.hasStatus(SlimeVR::Status::SERVER_CONNECTING)) {
		if (m_Packet[0] == PACKET_RECEIVE_HANDSHAKE) {
			if (strncmp((char*)m_Packet + 1, "Hey OVR =D 5", 12) != 0) {
				m_Logger.error("Received invalid handshake packet");
				return;
			}

			m_LastPacketTimestamp = millis();
			m_Connected = true;

			m_FeatureFlagsRequestAttempts = 0;
			m_ServerFeatures = ServerFeatures{};

			statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, false);
			ledManager.off();

			m_Logger.debug("Handshake successful");
		}
		return;
	}

	switch (convert_chars<int>(m_Packet)) {
		case PACKET_RECEIVE_HEARTBEAT: {
			sendHeartbeat();
			break;
		}

		case PACKET_RECEIVE_VIBRATE:
			break;

		case PACKET_RECEIVE_HANDSHAKE: {
			break;
		}

		case PACKET_RECEIVE_COMMAND:
			break;

		case PACKET_CONFIG:
			break;

		case PACKET_PING_PONG: {
			returnLastPacket(len);
			break;
		}

		case PACKET_SENSOR_INFO: {
			if (len < 6) {
				m_Logger.warn("Wrong sensor info packet");
				break;
			}

			std::vector<Sensor*>& sensors = sensorManager.getSensors();

			for (int i = 0; i < (int)sensors.size(); i++) {
				if (m_Packet[4] == sensors[i]->getSensorId()) {
					m_AckedSensorState[i] = (SensorStatus)m_Packet[5];
					break;
				}
			}

			break;
		}

		case PACKET_FEATURE_FLAGS: {
			// Packet type (4) + Packet number (8) + flags (len - 12)
			if (len < 13) {
				m_Logger.warn("Invalid feature flags packet: too short");
				break;
			}

			bool hadFlags = m_ServerFeatures.isAvailable();

			uint32_t flagsLength = len - 12;
			m_ServerFeatures = ServerFeatures::from(&m_Packet[12], flagsLength);

			if (!hadFlags) {
#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
				if (m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
					m_Logger.debug("Server supports packet bundling");
				}
#endif
			}

			break;
		}
	}
}

}  // namespace Network
}  // namespace SlimeVR
