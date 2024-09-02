#pragma once

#include "stdafx.h"
#include <mutex>
#include <shared_mutex>

enum class ConnectionStatus
{
	Idle,// Default state, no connection attempt has been made yet
	Connecting, //Connect attempt is currently running
	Connected, // Connection was successful
	Disconnected, // Device was connected but is now disconnected
	Timeout,  // Connection attempt timed out
	Failed, // Connection attempt failed due to known reasons
	Unknown_Error // Connection attempt failed due to unknown/unexpected reasons
};


// Typedef for the callback function to notify Unity
//typedef void (*ConnectionStatusCallback)(ConnectionStatus status);

struct DeviceUpdate {
	wchar_t id[100];
	bool isConnectable = false;
	bool isConnectableUpdated = false;
	bool isConnected = false;
	wchar_t name[50];
	bool nameUpdated = false;
};



struct Service {
	wchar_t uuid[100];
};

struct Characteristic {
	wchar_t uuid[100];
	wchar_t userDescription[100];
};

struct BLEData {
	uint8_t buf[512];
	uint16_t size;
	wchar_t deviceId[256];
	wchar_t serviceUuid[256];
	wchar_t characteristicUuid[256];
};
enum class LogLevel { Info, Warning, Error, Exception };
struct LogMessage {
	LogLevel logLevel;
	wchar_t message[1024];
};

enum class ScanStatus { Processing, Available, Finished, Failed };





extern "C" {

	__declspec(dllexport) void StartDeviceScan();

	__declspec(dllexport) ScanStatus PollDevice(DeviceUpdate* device, bool block);

	__declspec(dllexport) void StopDeviceScan();

	__declspec(dllexport) void ScanServices(wchar_t* deviceId);

	__declspec(dllexport) ScanStatus PollService(Service* service, bool block);

	__declspec(dllexport) void ScanCharacteristics(wchar_t* deviceId, wchar_t* serviceId);

	__declspec(dllexport) ScanStatus PollCharacteristic(Characteristic* characteristic, bool block);

	__declspec(dllexport) bool ConnectDevice(wchar_t* deviceId);
	__declspec(dllexport) ConnectionStatus PollConnectionStatusForDevice(wchar_t* deviceId);
	__declspec(dllexport) bool DisconnectDevice(wchar_t* deviceId);

	__declspec(dllexport) bool SubscribeCharacteristic(wchar_t* deviceId, wchar_t* serviceId, wchar_t* characteristicId, bool block);

	__declspec(dllexport) bool ReadCharacteristicData(wchar_t* deviceId, wchar_t* serviceId, wchar_t* characteristicId, bool block);

	__declspec(dllexport) bool PollData(BLEData* data, bool block);

	__declspec(dllexport) bool SendData(BLEData* data, bool block);


	__declspec(dllexport) void SetOutputLogLevel(LogLevel level);

	__declspec(dllexport) void Quit();

	__declspec(dllexport) bool DequeueLogMessage(LogMessage* outMessage);
}
