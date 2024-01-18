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
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include "featureflags.h"
#include "globals.h"
#include "quat.h"
#include "sensors/sensor.h"

namespace EspNow {
// bool isConnected();
void setUp();
// void upkeep();
// void setWiFiCredentials(const char* SSID, const char* pass);
// IPAddress getAddress();
// uint8_t getWiFiState();
}  // namespace EspNow

/** Wifi Reconnection Statuses **/
// typedef enum {
// 	SLIME_WIFI_NOT_SETUP = 0,
// 	SLIME_WIFI_SAVED_ATTEMPT,
// 	SLIME_WIFI_SAVED_G_ATTEMPT,
// 	SLIME_WIFI_HARDCODE_ATTEMPT,
// 	SLIME_WIFI_HARDCODE_G_ATTEMPT,
// 	SLIME_WIFI_SERVER_CRED_ATTEMPT,
// 	SLIME_WIFI_SERVER_CRED_G_ATTEMPT
// } wifi_reconnection_statuses;

namespace SlimeVR {
namespace Network {

class EspNowConnection {
public:
	void searchForServer();
	void update();
	void reset();
	bool isConnected() const { return m_Connected; }

	// PACKET_ACCEL 4
	void sendSensorAcceleration(uint8_t sensorId, Vector3 vector);

	// PACKET_BATTERY_LEVEL 12
	void sendBatteryLevel(float batteryVoltage, float batteryPercentage);

	// PACKET_TAP 13
	void sendSensorTap(uint8_t sensorId, uint8_t value);

	// PACKET_ERROR 14
	void sendSensorError(uint8_t sensorId, uint8_t error);

	// PACKET_ROTATION_DATA 17
	void sendRotationData(
		uint8_t sensorId,
		Quat* const quaternion,
		uint8_t dataType,
		uint8_t accuracyInfo
	);

	// PACKET_MAGNETOMETER_ACCURACY 18
	void sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo);

	// PACKET_SIGNAL_STRENGTH 19
	void sendSignalStrength(uint8_t signalStrength);

	// PACKET_TEMPERATURE 20
	void sendTemperature(uint8_t sensorId, float temperature);

	// PACKET_FEATURE_FLAGS 22
	void sendFeatureFlags();

	const ServerFeatures& getServerFeatureFlags() { return m_ServerFeatures; }

	bool beginBundle();
	bool endBundle();

	void onPacket(const uint8_t* buf, size_t count);

private:
	void updateSensorState(std::vector<Sensor*>& sensors);
	void maybeRequestFeatureFlags();

	bool beginPacket();
	bool endPacket();

	size_t write(const uint8_t* buffer, size_t size);
	size_t write(uint8_t byte);

	bool sendPacketType(uint8_t type);
	bool sendPacketNumber();
	bool sendFloat(float f);
	bool sendByte(uint8_t c);
	bool sendShort(uint16_t i);
	bool sendInt(uint32_t i);
	bool sendLong(uint64_t l);
	bool sendBytes(const uint8_t* c, size_t length);
	bool sendShortString(const char* str);
	bool sendLongString(const char* str);

	int getWriteError();

	void returnLastPacket(int len);

	// PACKET_HEARTBEAT 0
	void sendHeartbeat();

	// PACKET_HANDSHAKE 3
	void sendTrackerDiscovery();

	// PACKET_SENSOR_INFO 15
	void sendSensorInfo(Sensor* sensor);

	bool m_Connected = false;
	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("UDPConnection");

	unsigned char m_Packet[128];  // buffer for incoming packets
	uint64_t m_PacketNumber = 0;

	unsigned long m_LastConnectionAttemptTimestamp;
	unsigned long m_LastPacketTimestamp;

	SensorStatus m_AckedSensorState[MAX_IMU_COUNT] = {SensorStatus::SENSOR_OFFLINE};
	unsigned long m_LastSensorInfoPacketTimestamp = 0;

	uint8_t m_FeatureFlagsRequestAttempts = 0;
	unsigned long m_FeatureFlagsRequestTimestamp = millis();
	ServerFeatures m_ServerFeatures{};

	bool m_IsBundle = false;
	uint16_t m_BundlePacketPosition = 0;
	uint16_t m_BundlePacketInnerCount = 0;

	unsigned char m_OutPacket[128];  // buffer for outgoing packets
	int m_OutPacketPosition = 0;

	unsigned char m_Buf[8];
};

}  // namespace Network
}  // namespace SlimeVR
