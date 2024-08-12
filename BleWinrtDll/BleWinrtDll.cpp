// BleWinrtDll.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "BleWinrtDll.h"
#include <shared_mutex>
#include <mutex>
#include <iostream>

#pragma comment(lib, "windowsapp")

#define __WFILE__ L"BleWinrtDll.cpp"

using namespace std;
using namespace winrt;
using namespace Windows::Foundation;
using namespace Windows::Foundation::Collections;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace Windows::Devices::Enumeration;
using namespace Windows::Storage::Streams;

// Pointer to the callback function in Unity
void (*DebugLogCallback)(const char*) = nullptr;

void SetDebugLogCallback(void (*func)(const char*))
{
    DebugLogCallback = func;
}

void DebugLog(const char* message)
{
    if (DebugLogCallback)
    {
        DebugLogCallback(message);
    }
}

// Helper function to convert winrt::hstring to std::string
std::string ConvertHStringToString(const winrt::hstring& hstr)
{
    std::wstring wstr = hstr.c_str();
    std::string str(wstr.begin(), wstr.end());
    return str;
}



union to_guid {
    uint8_t buf[16];
    guid guid;
};

const uint8_t BYTE_ORDER[] = { 3, 2, 1, 0, 5, 4, 7, 6, 8, 9, 10, 11, 12, 13, 14, 15 };

guid make_guid(const wchar_t* value) {
    to_guid to_guid;
    memset(&to_guid, 0, sizeof(to_guid));
    int offset = 0;
    for (int i = 0; i < wcslen(value); i++) {
        uint8_t digit = 0;
        if (value[i] >= '0' && value[i] <= '9')
            digit = value[i] - '0';
        else if (value[i] >= 'A' && value[i] <= 'F')
            digit = 10 + value[i] - 'A';
        else if (value[i] >= 'a' && value[i] <= 'f')
            digit = 10 + value[i] - 'a';
        else
            continue;

        to_guid.buf[BYTE_ORDER[offset / 2]] += offset % 2 == 0 ? digit << 4 : digit;
        offset++;
    }

    return to_guid.guid;
}

mutex errorLock;
wchar_t last_error[2048] = L"Ok";

struct Subscription {
    GattCharacteristic characteristic = nullptr;
    GattCharacteristic::ValueChanged_revoker revoker;
};

struct CharacteristicCacheEntry {
    GattCharacteristic characteristic = nullptr;
};

struct ServiceCacheEntry {
    GattDeviceService service = nullptr;
    map<long, CharacteristicCacheEntry> characteristics = {};
};

struct DeviceCacheEntry {
    BluetoothLEDevice device = nullptr;
    map<long, ServiceCacheEntry> services = {};
};

map<long, DeviceCacheEntry> cache;
std::shared_mutex cacheMutex;

long hsh(const wchar_t* wstr) {
    long hash = 5381;
    int c;
    while (c = *wstr++)
        hash = ((hash << 5) + hash) + c;
    return hash;
}

void clearError() {
    lock_guard<mutex> error_lock(errorLock);
    wcscpy_s(last_error, L"Ok");
}

void saveError(const wchar_t* message, ...) {
    lock_guard<mutex> error_lock(errorLock);
    va_list args;
    va_start(args, message);
    vswprintf_s(last_error, message, args);
    va_end(args);
    wcout << last_error << endl;
}

IAsyncOperation<BluetoothLEDevice> retrieveDevice(const wchar_t* deviceId) {
    long deviceHash = hsh(deviceId);
    {
        shared_lock<shared_mutex> lock(cacheMutex);
        if (cache.count(deviceHash))
            co_return cache[deviceHash].device;
    }

    BluetoothLEDevice result = co_await BluetoothLEDevice::FromIdAsync(deviceId);
    if (result == nullptr) {
        saveError(L"%s:%d Failed to connect to device.", __WFILE__, __LINE__);
        co_return nullptr;
    }
    else {
        clearError();
        unique_lock<shared_mutex> lock(cacheMutex);
        cache[deviceHash] = { result };
        co_return cache[deviceHash].device;
    }
}

IAsyncOperation<GattDeviceService> retrieveService(const wchar_t* deviceId, const wchar_t* serviceId) {
    auto device = co_await retrieveDevice(deviceId);
    if (device == nullptr)
        co_return nullptr;

    long deviceHash = hsh(deviceId);
    long serviceHash = hsh(serviceId);

    {
        shared_lock<shared_mutex> lock(cacheMutex);
        if (cache[deviceHash].services.count(serviceHash))
            co_return cache[deviceHash].services[serviceHash].service;
    }

    GattDeviceServicesResult result = co_await device.GetGattServicesForUuidAsync(make_guid(serviceId), BluetoothCacheMode::Uncached);
    if (result.Status() != GattCommunicationStatus::Success) {
        saveError(L"%s:%d Failed retrieving services.", __WFILE__, __LINE__);
        co_return nullptr;
    }
    else if (result.Services().Size() == 0) {
        saveError(L"%s:%d No service found with uuid.", __WFILE__, __LINE__);
        co_return nullptr;
    }
    else {
        clearError();
        unique_lock<shared_mutex> lock(cacheMutex);
        cache[deviceHash].services[serviceHash] = { result.Services().GetAt(0) };
        co_return cache[deviceHash].services[serviceHash].service;
    }
}

IAsyncOperation<GattCharacteristic> retrieveCharacteristic(const wchar_t* deviceId, const wchar_t* serviceId, const wchar_t* characteristicId) {
    auto service = co_await retrieveService(deviceId, serviceId);
    if (service == nullptr)
        co_return nullptr;

    long deviceHash = hsh(deviceId);
    long serviceHash = hsh(serviceId);
    long characteristicHash = hsh(characteristicId);

    {
        shared_lock<shared_mutex> lock(cacheMutex);
        if (cache[deviceHash].services[serviceHash].characteristics.count(characteristicHash))
            co_return cache[deviceHash].services[serviceHash].characteristics[characteristicHash].characteristic;
    }

    GattCharacteristicsResult result = co_await service.GetCharacteristicsForUuidAsync(make_guid(characteristicId), BluetoothCacheMode::Uncached);
    if (result.Status() != GattCommunicationStatus::Success) {
        saveError(L"%s:%d Error scanning characteristics from service %s with status %d", __WFILE__, __LINE__, serviceId, result.Status());
        co_return nullptr;
    }
    else if (result.Characteristics().Size() == 0) {
        saveError(L"%s:%d No characteristic found with uuid %s", __WFILE__, __LINE__, characteristicId);
        co_return nullptr;
    }
    else {
        clearError();
        unique_lock<shared_mutex> lock(cacheMutex);
        cache[deviceHash].services[serviceHash].characteristics[characteristicHash] = { result.Characteristics().GetAt(0) };
        co_return cache[deviceHash].services[serviceHash].characteristics[characteristicHash].characteristic;
    }
}

queue<DeviceUpdate> deviceQueue;
mutex deviceQueueLock;
condition_variable deviceQueueSignal;
bool deviceScanFinished = false;
bool deviceScanError = false;

queue<Service> serviceQueue;
mutex serviceQueueLock;
condition_variable serviceQueueSignal;
bool serviceScanFinished = false;
bool serviceScanError = false;

queue<Characteristic> characteristicQueue;
mutex characteristicQueueLock;
condition_variable characteristicQueueSignal;
bool characteristicScanFinished = false;
bool characteristicScanError = false;

list<Subscription*> subscriptions;
mutex subscribeQueueLock;
condition_variable subscribeQueueSignal;

queue<BLEData> dataQueue;
mutex dataQueueLock;
condition_variable dataQueueSignal;

bool quitFlag = false;
mutex quitLock;

winrt::Windows::Devices::Enumeration::DeviceWatcher deviceWatcher{ nullptr };

bool QuittableWait(condition_variable& signal, unique_lock<mutex>& waitLock) {
    signal.wait(waitLock, [] { return quitFlag; });
    return quitFlag;
}

void DeviceWatcher_Added(DeviceWatcher sender, DeviceInformation deviceInfo) {
    DeviceUpdate deviceUpdate;
    wcscpy_s(deviceUpdate.id, deviceInfo.Id().c_str());
    wcscpy_s(deviceUpdate.name, deviceInfo.Name().c_str());
    deviceUpdate.nameUpdated = true;

    if (deviceInfo.Properties().HasKey(L"System.Devices.Aep.Bluetooth.Le.IsConnectable")) {
        deviceUpdate.isConnectable = unbox_value<bool>(deviceInfo.Properties().Lookup(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"));
        deviceUpdate.isConnectableUpdated = true;
    }
    if (deviceInfo.Properties().HasKey(L"System.Devices.Aep.IsConnected")) {
        deviceUpdate.isConnected = unbox_value<bool>(deviceInfo.Properties().Lookup(L"System.Devices.Aep.IsConnected"));
    }

    lock_guard<mutex> lock(deviceQueueLock);
    deviceQueue.push(deviceUpdate);
    deviceQueueSignal.notify_one();
}

void DeviceWatcher_Updated(DeviceWatcher sender, DeviceInformationUpdate deviceInfoUpdate) {
    DeviceUpdate deviceUpdate;
    wcscpy_s(deviceUpdate.id, deviceInfoUpdate.Id().c_str());

    if (deviceInfoUpdate.Properties().HasKey(L"System.Devices.Aep.Bluetooth.Le.IsConnectable")) {
        deviceUpdate.isConnectable = unbox_value<bool>(deviceInfoUpdate.Properties().Lookup(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"));
        deviceUpdate.isConnectableUpdated = true;
    }
    if (deviceInfoUpdate.Properties().HasKey(L"System.Devices.Aep.IsConnected")) {
        deviceUpdate.isConnected = unbox_value<bool>(deviceInfoUpdate.Properties().Lookup(L"System.Devices.Aep.IsConnected"));
    }

    lock_guard<mutex> lock(deviceQueueLock);
    deviceQueue.push(deviceUpdate);
    deviceQueueSignal.notify_one();
}

void DeviceWatcher_EnumerationCompleted(DeviceWatcher sender, IInspectable const&) {
    StopDeviceScan();
}

void StartDeviceScan()
{
    try
    {
        DebugLog("Starting device scan...");
        deviceScanFinished = false;
        deviceScanError = false;
        deviceQueue = {};  // Clear the queue

        IVector<hstring> requestedProperties = single_threaded_vector<hstring>({
            L"System.Devices.Aep.DeviceAddress",
            L"System.Devices.Aep.IsConnected",
            L"System.Devices.Aep.Bluetooth.Le.IsConnectable"
            });

        hstring aqsAllBluetoothLEDevices = BluetoothLEDevice::GetDeviceSelector();
        deviceWatcher = DeviceInformation::CreateWatcher(aqsAllBluetoothLEDevices, requestedProperties, DeviceInformationKind::AssociationEndpoint);

        if (!deviceWatcher)
        {
            DebugLog("Failed to create DeviceWatcher.");
            deviceScanError = true;
            return;
        }

        deviceWatcher.Added([&](DeviceWatcher sender, DeviceInformation deviceInfo)
            {
                lock_guard<mutex> lock(deviceQueueLock);
                DebugLog("Device found...");

                DeviceUpdate deviceUpdate;
                wcscpy_s(deviceUpdate.id, deviceInfo.Id().c_str());
                wcscpy_s(deviceUpdate.name, deviceInfo.Name().c_str());
                deviceUpdate.nameUpdated = true;

                if (deviceInfo.Properties().HasKey(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"))
                {
                    deviceUpdate.isConnectable = unbox_value<bool>(deviceInfo.Properties().Lookup(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"));
                    deviceUpdate.isConnectableUpdated = true;
                }
                if (deviceInfo.Properties().HasKey(L"System.Devices.Aep.IsConnected"))
                {
                    deviceUpdate.isConnected = unbox_value<bool>(deviceInfo.Properties().Lookup(L"System.Devices.Aep.IsConnected"));
                }

                deviceQueue.push(deviceUpdate);
            });

        deviceWatcher.EnumerationCompleted([&](DeviceWatcher sender, IInspectable const&)
            {
                lock_guard<mutex> lock(deviceQueueLock);
                DebugLog("Device enumeration completed.");
                deviceScanFinished = true;
            });

        DebugLog("Starting device watcher...");
        deviceWatcher.Start();
    }
    catch (const std::exception& e)
    {
        DebugLog("Error during StartDeviceScan:");
        DebugLog(e.what());
        deviceScanError = true; // Mark that an error occurred
    }
}

ScanStatus PollDevice(DeviceUpdate* device, bool block)
{
    try
    {
        DebugLog("Polling device...");
        lock_guard<mutex> lock(deviceQueueLock);

        if (deviceScanError)
        {
            DebugLog("Polling failed: Device scan error detected.");
            return ScanStatus::FAILED;
        }

        if (!deviceQueue.empty())
        {
            *device = deviceQueue.front();
            deviceQueue.pop();

            std::string message = "Device available: ID = " + ConvertHStringToString(device->id) + ", Name = " + ConvertHStringToString(device->name);
            DebugLog(message.c_str());

            return ScanStatus::AVAILABLE;
        }

        if (deviceScanFinished)
        {
            DebugLog("Polling completed: No more devices, scan finished.");
            return ScanStatus::FINISHED;
        }

        DebugLog("Polling in progress: No devices available yet.");
        return ScanStatus::PROCESSING;
    }
    catch (const std::exception& e)
    {
        std::string error_message = "Error during PollDevice: " + std::string(e.what());
        DebugLog(error_message.c_str());
        return ScanStatus::FAILED; // Return FAILED status if an exception occurs
    }
}


void StopDeviceScan()
{
    lock_guard<mutex> lock(deviceQueueLock);
    deviceScanFinished = true;
}

fire_and_forget ScanServicesAsync(const wchar_t* deviceId)
{
    try
    {
        DebugLog("Starting service scan...");
        serviceScanFinished = false;
        serviceScanError = false;
        serviceQueue = {}; // Clear the queue

        auto bluetoothLeDevice = co_await BluetoothLEDevice::FromIdAsync(deviceId);

        if (bluetoothLeDevice == nullptr)
        {
            DebugLog("Failed to connect to device.");
            serviceScanError = true;
            co_return;
        }

        auto result = co_await bluetoothLeDevice.GetGattServicesAsync(BluetoothCacheMode::Uncached);

        if (result.Status() == GattCommunicationStatus::Success)
        {
            for (auto&& service : result.Services())
            {
                Service serviceStruct;
                std::wstring uuidWString = winrt::to_hstring(service.Uuid()).c_str();
                wcscpy_s(serviceStruct.uuid, sizeof(serviceStruct.uuid) / sizeof(wchar_t), uuidWString.c_str());

                std::string message = "Service found: UUID = " + ConvertHStringToString(serviceStruct.uuid);
                DebugLog(message.c_str());

                lock_guard<mutex> lock(serviceQueueLock);
                serviceQueue.push(serviceStruct);
            }
        }
        else
        {
            DebugLog("Failed to retrieve GATT services.");
            serviceScanError = true;
        }
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during ScanServicesAsync: " + std::string(ex.what());
        DebugLog(error_message.c_str());
        serviceScanError = true;
    }

    lock_guard<mutex> lock(serviceQueueLock);
    serviceScanFinished = true;
}

void ScanServices(wchar_t* deviceId) {
    ScanServicesAsync(deviceId);
}

// Function to poll for services
ScanStatus PollService(Service* service, bool block)
{
    try
    {
        DebugLog("Polling for service...");

        lock_guard<mutex> lock(serviceQueueLock);

        if (serviceScanError)
        {
            DebugLog("Service scan error detected.");
            return ScanStatus::FAILED;
        }

        if (!serviceQueue.empty())
        {
            *service = serviceQueue.front();
            serviceQueue.pop();

            std::string message = "Service available: UUID = " + ConvertHStringToString(service->uuid);
            DebugLog(message.c_str());

            return ScanStatus::AVAILABLE;
        }

        if (serviceScanFinished)
        {
            DebugLog("Service scan finished.");
            return ScanStatus::FINISHED;
        }

        DebugLog("Service scan still processing.");
        return ScanStatus::PROCESSING;
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during PollService: " + std::string(ex.what());
        DebugLog(error_message.c_str());
        return ScanStatus::FAILED;
    }
}

// Function to scan characteristics asynchronously
fire_and_forget ScanCharacteristicsAsync(wchar_t* deviceId, wchar_t* serviceId)
{
    try
    {
        DebugLog("Starting characteristic scan...");
        characteristicScanFinished = false;
        characteristicScanError = false;
        characteristicQueue = {}; // Clear the queue

        auto service = co_await retrieveService(deviceId, serviceId);

        if (service == nullptr)
        {
            DebugLog("Failed to retrieve service.");
            characteristicScanError = true;
            co_return;
        }

        auto result = co_await service.GetCharacteristicsAsync(BluetoothCacheMode::Uncached);

        if (result.Status() == GattCommunicationStatus::Success)
        {
            DebugLog("Characteristic scan successful, processing characteristics...");
            for (auto&& characteristic : result.Characteristics())
            {
                Characteristic charStruct;
                wcscpy_s(charStruct.uuid, sizeof(charStruct.uuid) / sizeof(wchar_t), winrt::to_hstring(characteristic.Uuid()).c_str());

                auto descriptionResult = co_await characteristic.GetDescriptorsAsync();
                if (descriptionResult.Status() == GattCommunicationStatus::Success && descriptionResult.Descriptors().Size() > 0)
                {
                    auto descriptor = descriptionResult.Descriptors().GetAt(0);
                    auto readValueResult = co_await descriptor.ReadValueAsync();
                    if (readValueResult.Status() == GattCommunicationStatus::Success)
                    {
                        auto reader = DataReader::FromBuffer(readValueResult.Value());
                        wcscpy_s(charStruct.userDescription, sizeof(charStruct.userDescription) / sizeof(wchar_t), reader.ReadString(reader.UnconsumedBufferLength()).c_str());
                    }
                }

                std::string message = "Characteristic found: UUID = " + ConvertHStringToString(winrt::to_hstring(characteristic.Uuid())) + ", Description = " + ConvertHStringToString(winrt::to_hstring(charStruct.userDescription));
                DebugLog(message.c_str());

                {
                    lock_guard<mutex> lock(characteristicQueueLock);
                    characteristicQueue.push(charStruct);
                }

                // Add a slight delay if needed to avoid tight loops
                co_await resume_background(); // This can yield to other operations if necessary
            }
        }
        else
        {
            DebugLog("Failed to retrieve characteristics.");
            characteristicScanError = true;
        }
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during ScanCharacteristicsAsync: " + std::string(ex.what());
        DebugLog(error_message.c_str());
        characteristicScanError = true;
    }

    lock_guard<mutex> lock(characteristicQueueLock);
    characteristicScanFinished = true;
    DebugLog("Characteristic scan completed.");
}

void ScanCharacteristics(wchar_t* deviceId, wchar_t* serviceId) {
    ScanCharacteristicsAsync(deviceId, serviceId);
}

ScanStatus PollCharacteristic(Characteristic* characteristic, bool block)
{
    try
    {
        DebugLog("Polling for characteristic...");

        lock_guard<mutex> lock(characteristicQueueLock);

        if (characteristicScanError)
        {
            DebugLog("Characteristic scan error detected.");
            return ScanStatus::FAILED;
        }

        if (!characteristicQueue.empty())
        {
            *characteristic = characteristicQueue.front();
            characteristicQueue.pop();

            std::string message = "Characteristic available: UUID = " + ConvertHStringToString(winrt::to_hstring(characteristic->uuid)) + ", Description = " + ConvertHStringToString(winrt::to_hstring(characteristic->userDescription));
            DebugLog(message.c_str());

            return ScanStatus::AVAILABLE;
        }

        if (characteristicScanFinished)
        {
            DebugLog("Characteristic scan finished.");
            return ScanStatus::FINISHED;
        }

        DebugLog("Characteristic scan still processing.");
        return ScanStatus::PROCESSING;
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during PollCharacteristic: " + std::string(ex.what());
        DebugLog(error_message.c_str());
        return ScanStatus::FAILED;
    }
}

fire_and_forget SubscribeCharacteristicAsync(const wchar_t* deviceId, const wchar_t* serviceId, const wchar_t* characteristicId, bool* result)
{
    try
    {
        DebugLog("Starting characteristic subscription...");

        auto characteristic = co_await retrieveCharacteristic(deviceId, serviceId, characteristicId);

        if (characteristic == nullptr)
        {
            DebugLog("Failed to retrieve characteristic for subscription.");
            if (result != nullptr)
            {
                *result = false;
            }
            co_return;
        }

        DebugLog("Retrieved characteristic, attempting to subscribe...");

        // Check if characteristic supports Notify or Indicate
        GattCharacteristicProperties properties = characteristic.CharacteristicProperties();
        if ((properties & GattCharacteristicProperties::Notify) == GattCharacteristicProperties::None &&
            (properties & GattCharacteristicProperties::Indicate) == GattCharacteristicProperties::None)
        {
            DebugLog("Characteristic does not support Notify or Indicate. Subscription aborted.");
            if (result != nullptr)
            {
                *result = false;
            }
            co_return;
        }

        auto status = co_await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue::Notify);

        if (status != GattCommunicationStatus::Success)
        {
            std::string message = "Error subscribing to characteristic with UUID: " + ConvertHStringToString(winrt::to_hstring(characteristic.Uuid())) + ", Status: " + std::to_string(static_cast<int>(status));
            DebugLog(message.c_str());
            if (result != nullptr)
            {
                *result = false;
            }
        }
        else
        {
            std::string message = "Successfully subscribed to characteristic with UUID: " + ConvertHStringToString(winrt::to_hstring(characteristic.Uuid()));
            DebugLog(message.c_str());

            auto subscription = new Subscription{ characteristic };

            subscription->revoker = characteristic.ValueChanged(auto_revoke, [deviceId, serviceId, characteristicId](GattCharacteristic const&, GattValueChangedEventArgs const& args)
                {
                    try {
                        DebugLog("ValueChanged event triggered.");

                        BLEData bleData;
                        bleData.size = args.CharacteristicValue().Length();
                        memcpy(bleData.buf, args.CharacteristicValue().data(), bleData.size);

                        DebugLog("Data received from ValueChanged event, populating BLEData.");

                        // Populate the BLEData with device, service, and characteristic IDs
                        wcscpy_s(bleData.deviceId, sizeof(bleData.deviceId) / sizeof(wchar_t), deviceId);
                        wcscpy_s(bleData.serviceUuid, sizeof(bleData.serviceUuid) / sizeof(wchar_t), serviceId);
                        wcscpy_s(bleData.characteristicUuid, sizeof(bleData.characteristicUuid) / sizeof(wchar_t), characteristicId);

                        {
                            lock_guard<mutex> lock(dataQueueLock);
                            dataQueue.push(bleData);
                            DebugLog("Data enqueued into dataQueue.");
                            dataQueueSignal.notify_one();
                        }

                        std::string dataMessage = "Data received: Size = " + std::to_string(bleData.size);
                        DebugLog(dataMessage.c_str());
                    }
                    catch (const std::exception& ex)
                    {
                        std::string error_message = "Error during ValueChanged event handling: " + std::string(ex.what());
                        DebugLog(error_message.c_str());
                    }
                });

            {
                lock_guard<mutex> lock(subscribeQueueLock);
                subscriptions.push_back(subscription);
            }

            if (result != nullptr)
            {
                *result = true;
            }
        }
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during SubscribeCharacteristicAsync: " + std::string(ex.what());
        DebugLog(error_message.c_str());
        if (result != nullptr)
        {
            *result = false;
        }
    }

    DebugLog("Characteristic subscription process completed.");
    if (result != nullptr)
    {
        subscribeQueueSignal.notify_one();
    }
}

bool SubscribeCharacteristic(wchar_t* deviceId, wchar_t* serviceId, wchar_t* characteristicId, bool block) {
    
    bool result = false;
    {
        unique_lock<mutex> lock(subscribeQueueLock);
        SubscribeCharacteristicAsync(deviceId, serviceId, characteristicId, block ? &result : nullptr);

        if (block)
        {
            subscribeQueueSignal.wait(lock);
        }
    }

    std::string message = "Subscription to characteristic " + ConvertHStringToString(winrt::to_hstring(characteristicId)) + " was " + (result ? "successful" : "unsuccessful");
    DebugLog(message.c_str());

    return result;
}

bool PollData(BLEData* data, bool block)
{
    try
    {
        DebugLog("Polling for BLE data...");
        DebugLog("Attempting to acquire dataQueueLock...");
        unique_lock<mutex> lock(dataQueueLock);
        DebugLog("Successfully acquired dataQueueLock...");
        if (block && dataQueue.empty())
        {
            DebugLog("Data queue is empty, waiting for data...");
            dataQueueSignal.wait(lock);
        }

        if (!dataQueue.empty())
        {
            DebugLog("Data found in queue, retrieving...");
            *data = dataQueue.front();
            dataQueue.pop();

            std::string message = "Data available: Device ID = " + ConvertHStringToString(data->deviceId) +
                ", Service UUID = " + ConvertHStringToString(data->serviceUuid) +
                ", Characteristic UUID = " + ConvertHStringToString(data->characteristicUuid) +
                ", Data Size = " + std::to_string(data->size);
            DebugLog(message.c_str());

            return true;
        }

        DebugLog("No data available.");
        return false;
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during PollData: " + std::string(ex.what());
        DebugLog(error_message.c_str());
        return false;
    }
    catch (...)
    {
        DebugLog("Unknown error occurred during PollData.");
        return false;
    }
}

fire_and_forget SendDataAsync(BLEData data, condition_variable* signal, bool* result) {
    try {
        auto characteristic = co_await retrieveCharacteristic(data.deviceId, data.serviceUuid, data.characteristicUuid);
        if (characteristic != nullptr) {
            DataWriter writer;
            writer.WriteBytes(array_view<uint8_t const>(data.buf, data.buf + data.size));
            IBuffer buffer = writer.DetachBuffer();
            auto status = co_await characteristic.WriteValueAsync(buffer, GattWriteOption::WriteWithoutResponse);
            if (status != GattCommunicationStatus::Success) {
                saveError(L"%s:%d Error writing value to characteristic with uuid %s", __WFILE__, __LINE__, data.characteristicUuid);
            }
            else if (result) {
                *result = true;
            }
        }
    }
    catch (hresult_error& ex) {
        saveError(L"%s:%d SendDataAsync catch: %s", __WFILE__, __LINE__, ex.message().c_str());
    }

    if (signal)
        signal->notify_one();
}

bool SendData(BLEData* data, bool block) {
    condition_variable signal;
    bool result = false;

    SendDataAsync(*data, block ? &signal : nullptr, block ? &result : nullptr);

    if (block) {
        mutex _mutex;
        unique_lock<mutex> lock(_mutex);
        signal.wait(lock);
    }

    return result;
}

void Quit() {
    {
        lock_guard<mutex> lock(quitLock);
        quitFlag = true;
    }

    StopDeviceScan();
    deviceQueueSignal.notify_one();
    {
        lock_guard<mutex> lock(deviceQueueLock);
        deviceQueue = {};
    }

    // Signal that scanning should stop
    serviceScanFinished = true;
    serviceScanError = true;
    characteristicScanFinished = true;
    characteristicScanError = true;

    // Clear any remaining items in the queues
    {
        lock_guard<mutex> lock(serviceQueueLock);
        while (!serviceQueue.empty())
        {
            serviceQueue.pop();
        }
    }

    {
        lock_guard<mutex> lock(characteristicQueueLock);
        while (!characteristicQueue.empty())
        {
            characteristicQueue.pop();
        }
    }

    DebugLog("Cleaning up and shutting down...");

    // Stop and clean up the DeviceWatcher if it's running
    if (deviceWatcher != nullptr)
    {
        DebugLog("Stopping DeviceWatcher...");
        deviceWatcher.Stop();
        deviceWatcher = nullptr;
    }


    // Optionally, perform any additional resource cleanup here

   

    serviceQueueSignal.notify_one();
    {
        lock_guard<mutex> lock(serviceQueueLock);
        serviceQueue = {};
    }

    characteristicQueueSignal.notify_one();
    {
        lock_guard<mutex> lock(characteristicQueueLock);
        characteristicQueue = {};
    }

    subscribeQueueSignal.notify_one();
    {
        lock_guard<mutex> lock(subscribeQueueLock);
        for (auto subscription : subscriptions)
            subscription->revoker.revoke();
        subscriptions.clear();
    }

    dataQueueSignal.notify_one();
    {
        lock_guard<mutex> lock(dataQueueLock);
        dataQueue = {};
    }

    {
        unique_lock<shared_mutex> lock(cacheMutex);
        for (auto& device : cache) {
            device.second.device.Close();
            for (auto& service : device.second.services) {
                service.second.service.Close();
            }
        }
        cache.clear();
    }
    DebugLog("Cleanup completed.");

}

void GetError(ErrorMessage* buf) {
    lock_guard<mutex> lock(errorLock);
    wcscpy_s(buf->msg, last_error);
}
