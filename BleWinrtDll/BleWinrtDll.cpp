// BleWinrtDll.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "BleWinrtDll.h"
#include <shared_mutex>
#include <mutex>
#include <iostream>
#include <future>
#include <chrono>
#include <Windows.h>
#include <vector>
#include <sstream>

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




#pragma region LOGGING

LogLevel outputLogLevel;
std::queue<LogMessage> logQueue;
std::mutex logMutex;

template<typename... Args>
std::string MakeLogMessage(const char* function, const char* prefix, Args&&... args) {
    std::ostringstream oss;
    oss << "BleWinRTDll: " << function << ": " << prefix;
    ((oss << std::forward<Args>(args)), ...);
    return oss.str();
}

#define LOG(message) { if(outputLogLevel == LogLevel::Info) { EnqueueLogMessage(LogLevel::Info, MakeLogMessage(__FUNCTION__, message).c_str()); }}
#define LOG_WARNING(message) if(outputLogLevel == LogLevel::Info || outputLogLevel == LogLevel::Warning) { EnqueueLogMessage(LogLevel::Warning, MakeLogMessage(__FUNCTION__, "Warning: ", message).c_str()); }
#define LOG_ERROR(message) if(outputLogLevel == LogLevel::Info || outputLogLevel == LogLevel::Warning || outputLogLevel == LogLevel::Error){ EnqueueLogMessage(LogLevel::Error, MakeLogMessage(__FUNCTION__"Error: ", message).c_str());}
#define LOG_EXCEPTION(message){ EnqueueLogMessage(LogLevel::Exception, MakeLogMessage(__FUNCTION__"Exception: ", message).c_str());}

void EnqueueLogMessage(LogLevel logLevel, const char* message) {
    try {
        std::lock_guard<std::mutex> lock(logMutex);
        LogMessage logMsg;
        logMsg.logLevel = logLevel;

        // Convert char* to wchar_t*
        int wideSize = MultiByteToWideChar(CP_UTF8, 0, message, -1, NULL, 0);
        if (wideSize > 0) {
            std::vector<wchar_t> wideMessage(wideSize);
            MultiByteToWideChar(CP_UTF8, 0, message, -1, wideMessage.data(), wideSize);
            wcsncpy_s(logMsg.message, wideMessage.data(), std::size(logMsg.message) - 1);
            logMsg.message[std::size(logMsg.message) - 1] = L'\0';
        }
        logQueue.push(logMsg);
    }
    catch (...) {
        // Handle exception
    }
   
}


void SetOutputLogLevel(LogLevel level)
{
    outputLogLevel = level;
}

bool DequeueLogMessage(LogMessage* outMessage) {
    std::lock_guard<std::mutex> lock(logMutex);
    if (logQueue.empty()) {
        return false;
    }
    *outMessage = logQueue.front();
    logQueue.pop();
    return true;
}


#pragma endregion


#pragma region CONVERSION
// Helper function to convert winrt::hstring to std::string
std::string ConvertHStringToString(const winrt::hstring& hstr)
{
    std::wstring wstr = hstr.c_str();
    std::string str(wstr.begin(), wstr.end());
    return str;
}

std::string guid_to_string(const winrt::guid& guid) {
    char guid_cstr[37];
    snprintf(guid_cstr, sizeof(guid_cstr),
        "%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        guid.Data1, guid.Data2, guid.Data3,
        guid.Data4[0], guid.Data4[1], guid.Data4[2], guid.Data4[3],
        guid.Data4[4], guid.Data4[5], guid.Data4[6], guid.Data4[7]);
    return std::string(guid_cstr);
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
#pragma endregion


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



std::chrono::milliseconds timeout = std::chrono::milliseconds(30000);

std::mutex connectionStatusLock;
std::mutex deviceLock;
std::map<std::wstring, ConnectionStatus> deviceConnectionStatusMap;
std::map<std::wstring, DeviceCacheEntry> deviceMap;
std::condition_variable connectCV;
std::mutex connectMutex;
bool isConnecting = false;

map<long, DeviceCacheEntry> cache;
std::shared_mutex cacheMutex;

long hsh(const wchar_t* wstr) {
    long hash = 5381;
    int c;
    while (c = *wstr++)
        hash = ((hash << 5) + hash) + c;
    return hash;
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
        LOG_ERROR("%s:%d Failed to connect to device.", __FILE__, __LINE__);
        co_return nullptr;
    }
    else {
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
        LOG_ERROR("%s:%d Failed retrieving services.", __FILE__, __LINE__);
        co_return nullptr;
    }
    else if (result.Services().Size() == 0) {
        LOG_ERROR("%s:%d No service found with uuid.", __FILE__, __LINE__);
        co_return nullptr;
    }
    else {
        
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
        LOG_ERROR("%s:%d Error scanning characteristics from service %s with status %d", __WFILE__, __LINE__, serviceId, result.Status());
        co_return nullptr;
    }
    else if (result.Characteristics().Size() == 0) {
        LOG_ERROR("%s:%d No characteristic found with uuid %s", __WFILE__, __LINE__, characteristicId);
        co_return nullptr;
    }
    else {
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

std::list<std::unique_ptr<Subscription>> subscriptions;
mutex subscribeQueueLock;
condition_variable subscribeQueueSignal;

queue<BLEData> dataQueue;
mutex dataQueueLock;
condition_variable dataQueueSignal;

bool quitFlag = false;
mutex quitLock;

mutex connectLock;



winrt::Windows::Devices::Enumeration::DeviceWatcher deviceWatcher{ nullptr };

bool QuittableWait(condition_variable& signal, unique_lock<mutex>& waitLock) {
    signal.wait(waitLock, [] { return quitFlag; });
    return quitFlag;
}

bool IsQuitFlagSet()
{
    std::lock_guard<std::mutex> lock(quitLock);
    return quitFlag;
}

void ResetPluginState() {
    LOG("Starting to reset plugin state...");

    try {
        // Device queue reset
        LOG("About to acquire deviceQueueLock...");
        {
            std::lock_guard<std::mutex> lock(deviceQueueLock);
            LOG("Resetting device queue. Current size: ", std::to_string(deviceQueue.size()));
            deviceScanFinished = false;
            deviceScanError = false;
            std::queue<DeviceUpdate>().swap(deviceQueue);
            LOG("Device queue reset. New size: ", std::to_string(deviceQueue.size()));
        }
        LOG("Released deviceQueueLock.");

        // Service queue reset
        LOG("About to acquire serviceQueueLock...");
        {
            std::lock_guard<std::mutex> lock(serviceQueueLock);
            LOG("Resetting service queue. Current size: ", std::to_string(serviceQueue.size()));
            serviceScanFinished = false;
            serviceScanError = false;
            std::queue<Service>().swap(serviceQueue);
            LOG("Service queue reset. New size: ", std::to_string(serviceQueue.size()));
        }
        LOG("Released serviceQueueLock.");

        // Characteristic queue reset
        LOG("About to acquire characteristicQueueLock...");
        {
            std::lock_guard<std::mutex> lock(characteristicQueueLock);
            LOG("Resetting characteristic queue. Current size: ", std::to_string(characteristicQueue.size()));
            characteristicScanFinished = false;
            characteristicScanError = false;
            std::queue<Characteristic>().swap(characteristicQueue);
            LOG("Characteristic queue reset. New size: ", std::to_string(characteristicQueue.size()));
        }
        LOG("Released characteristicQueueLock.");

        // Data queue reset
        LOG("About to acquire dataQueueLock...");
        {
            std::lock_guard<std::mutex> lock(dataQueueLock);
            LOG("Resetting data queue. Current size: ", std::to_string(dataQueue.size()));
            std::queue<BLEData>().swap(dataQueue);
            LOG("Data queue reset. New size: ", std::to_string(dataQueue.size()));
        }
        LOG("Released dataQueueLock.");

        // Quit flag reset
        LOG("About to acquire quitLock...");
        {
            std::lock_guard<std::mutex> lock(quitLock);
            LOG("Resetting quit flag. Current value: ", std::to_string(quitFlag));
            quitFlag = false;
            LOG("Quit flag reset. New value: ", std::to_string(quitFlag));
        }
        LOG("Released quitLock.");

        // Reset connection state
        LOG("About to acquire connectLock...");
        {
            std::lock_guard<std::mutex> lock(connectLock);
            for(auto const& [id, cacheEntry]  : deviceMap)
            {
                auto bluetoothLeDevice = cacheEntry.device;
                if (bluetoothLeDevice != nullptr) {
                    LOG("Closing active BluetoothLEDevice connection...");
                    bluetoothLeDevice.Close();
                }
                LOG("BluetoothLEDevice connection reset.");
            }
            
        }
        LOG("Released connectLock.");

        // Reset DeviceWatcher
        LOG("Checking DeviceWatcher state...");
        if (deviceWatcher != nullptr) {
            LOG("Stopping DeviceWatcher...");
            deviceWatcher.Stop();
            deviceWatcher = nullptr;
            LOG("DeviceWatcher stopped and reset.");
        }
        else {
            LOG("DeviceWatcher was already null.");
        }

        LOG("Plugin state has been successfully reset.");
    }
    catch (const winrt::hresult_error& e) {
        LOG_ERROR("WinRT exception occurred during plugin state reset: ", winrt::to_string(e.message()));
    }
    catch (const std::exception& e) {
        LOG_ERROR("Standard exception occurred during plugin state reset: ", std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception occurred during plugin state reset.");
    }
}

void OnDeviceAdded(DeviceInformation deviceInfo)
{
    
    LOG("OnDeviceAdded: ", winrt::to_string(deviceInfo.Id()));

    DeviceUpdate deviceUpdate;
    wcscpy_s(deviceUpdate.id, deviceInfo.Id().c_str());
    wcscpy_s(deviceUpdate.name, deviceInfo.Name().c_str());
    deviceUpdate.nameUpdated = true;

    if (deviceInfo.Properties().HasKey(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"))
    {
        deviceUpdate.isConnectable = winrt::unbox_value<bool>(deviceInfo.Properties().Lookup(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"));
        deviceUpdate.isConnectableUpdated = true;
    }
    if (deviceInfo.Properties().HasKey(L"System.Devices.Aep.IsConnected"))
    {
        deviceUpdate.isConnected = winrt::unbox_value<bool>(deviceInfo.Properties().Lookup(L"System.Devices.Aep.IsConnected"));
    }
    std::lock_guard<std::mutex> lock(deviceQueueLock);
    deviceQueue.push(deviceUpdate);
    LOG("Device added to queue. Queue size: ", std::to_string(deviceQueue.size()));
    deviceQueueSignal.notify_one();
}

void OnDeviceUpdated(DeviceInformationUpdate deviceInfoUpdate)
{
    std::lock_guard<std::mutex> lock(deviceQueueLock);
    LOG("Device updated: ", winrt::to_string(deviceInfoUpdate.Id()));

    DeviceUpdate deviceUpdate;
    wcscpy_s(deviceUpdate.id, deviceInfoUpdate.Id().c_str());

    if (deviceInfoUpdate.Properties().HasKey(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"))
    {
        deviceUpdate.isConnectable = winrt::unbox_value<bool>(deviceInfoUpdate.Properties().Lookup(L"System.Devices.Aep.Bluetooth.Le.IsConnectable"));
        deviceUpdate.isConnectableUpdated = true;
    }
    if (deviceInfoUpdate.Properties().HasKey(L"System.Devices.Aep.IsConnected"))
    {
        deviceUpdate.isConnected = winrt::unbox_value<bool>(deviceInfoUpdate.Properties().Lookup(L"System.Devices.Aep.IsConnected"));
    }

    deviceQueue.push(deviceUpdate);
    deviceQueueSignal.notify_one();
}

void OnDeviceRemoved(DeviceInformationUpdate deviceInfoUpdate)
{
    std::lock_guard<std::mutex> lock(deviceQueueLock);
    LOG("Device removed: ", winrt::to_string(deviceInfoUpdate.Id()));

    // Implement device removal logic if needed
}

void OnEnumerationCompleted()
{
    std::lock_guard<std::mutex> lock(deviceQueueLock);
    LOG("Device enumeration completed.");
    deviceScanFinished = true;
    deviceQueueSignal.notify_all();
}

void OnScanStopped()
{
    std::lock_guard<std::mutex> lock(deviceQueueLock);
    LOG("Device scan stopped.");
    deviceScanFinished = true;
    deviceQueueSignal.notify_all();
}

void StartDeviceScan()
{
    try
    {
        LOG("Starting device scan...");
        ResetPluginState();

        LOG("Initializing WinRT components...");
        winrt::init_apartment();
        LOG("WinRT components initialized successfully.");

        LOG("Checking Bluetooth adapter state...");
        auto adapter = BluetoothAdapter::GetDefaultAsync().get();
        if (adapter == nullptr)
        {
            LOG_ERROR("No Bluetooth adapter found.");
            deviceScanError = true;
            return;
        }
        if (!adapter.IsLowEnergySupported())
        {
            LOG_ERROR("Bluetooth adapter does not support Low Energy.");
            deviceScanError = true;
            return;
        }
        LOG("Bluetooth adapter is ready for LE operations.");

        LOG("Creating DeviceWatcher...");
        IVector<hstring> requestedProperties = single_threaded_vector<hstring>({
            L"System.Devices.Aep.DeviceAddress",
            L"System.Devices.Aep.IsConnected",
            L"System.Devices.Aep.Bluetooth.Le.IsConnectable"
            });

        LOG("Getting device selector...");
        hstring aqsAllBluetoothLEDevices = BluetoothLEDevice::GetDeviceSelector();
        LOG("Device selector obtained: ", winrt::to_string(aqsAllBluetoothLEDevices));
        
        LOG("Creating watcher...");
        deviceWatcher = winrt::Windows::Devices::Enumeration::DeviceInformation::CreateWatcher(
            aqsAllBluetoothLEDevices,
            requestedProperties,
            winrt::Windows::Devices::Enumeration::DeviceInformationKind::AssociationEndpoint);

        if (deviceWatcher == nullptr)
        {
            LOG_ERROR("Failed to create DeviceWatcher.");
            deviceScanError = true;
            return;
        }
        LOG("DeviceWatcher created successfully.");

        LOG("Setting up DeviceWatcher callbacks...");
        deviceWatcher.Added([](DeviceWatcher sender, DeviceInformation deviceInfo)
            {
                LOG("DeviceWatcher_Added: ", winrt::to_string(deviceInfo.Id()));
                OnDeviceAdded(deviceInfo);
            });

        deviceWatcher.Updated([](DeviceWatcher sender, DeviceInformationUpdate deviceInfoUpdate)
            {
                LOG("DeviceWatcher_Updated: ", winrt::to_string(deviceInfoUpdate.Id()));
                OnDeviceUpdated(deviceInfoUpdate);
            });

        deviceWatcher.Removed([](DeviceWatcher sender, DeviceInformationUpdate deviceInfoUpdate)
            {
                LOG("DeviceWatcher_Removed: ", winrt::to_string(deviceInfoUpdate.Id()));
                OnDeviceRemoved(deviceInfoUpdate);
            });

        deviceWatcher.EnumerationCompleted([](DeviceWatcher sender, IInspectable const&)
            {
                LOG("DeviceWatcher_EnumerationCompleted");
                OnEnumerationCompleted();
            });

        deviceWatcher.Stopped([](DeviceWatcher sender, IInspectable const&)
            {
                LOG("DeviceWatcher_Stopped");
                OnScanStopped();
            });

        LOG("Starting DeviceWatcher...");
        deviceWatcher.Start();

        LOG("Device scan started successfully.");
    }
    catch (const winrt::hresult_error& e)
    {
        LOG_ERROR("WinRT error in StartDeviceScan: ", winrt::to_string(e.message()));
        deviceScanError = true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Standard exception in StartDeviceScan: ", std::string(e.what()));
        deviceScanError = true;
    }
    catch (...)
    {
        LOG_ERROR("Unknown exception in StartDeviceScan");
        deviceScanError = true;
    }
}



ScanStatus PollDevice(DeviceUpdate* device, bool block)
{
    try
    {
        LOG("Polling device...");
        lock_guard<mutex> lock(deviceQueueLock);

        if (deviceScanError)
        {
            LOG_ERROR("Polling failed: Device scan error detected.");
            return ScanStatus::Failed;
        }

        if (!deviceQueue.empty())
        {
            *device = deviceQueue.front();
            deviceQueue.pop();

            std::string message = "Device available: ID = " + ConvertHStringToString(device->id) + ", Name = " + ConvertHStringToString(device->name);
            LOG(message.c_str());

            return ScanStatus::Available;
        }

        if (deviceScanFinished)
        {
            LOG("Polling completed: No more devices, scan finished.");
            return ScanStatus::Finished;
        }

        LOG("No devices available yet. Queue size: ", std::to_string(deviceQueue.size()));
        return ScanStatus::Processing;
    }
    catch (const std::exception& e)
    {
        std::string error_message = "Error during PollDevice: " + std::string(e.what());
        LOG_ERROR(error_message.c_str());
        return ScanStatus::Failed; // Return FAILED status if an exception occurs
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
        LOG("Starting service scan...");
        {
            std::lock_guard<std::mutex> lock(serviceQueueLock);
            serviceScanFinished = false;
            serviceScanError = false;
            std::queue<Service>().swap(serviceQueue); // Clear the queue
        }

        auto bluetoothLeDevice = co_await BluetoothLEDevice::FromIdAsync(deviceId);

        if (bluetoothLeDevice == nullptr)
        {
            LOG_ERROR("Failed to connect to device.");
            std::lock_guard<std::mutex> lock(serviceQueueLock);
            serviceScanError = true;
            serviceQueueSignal.notify_all();
            co_return;
        }

        auto result = co_await bluetoothLeDevice.GetGattServicesAsync(BluetoothCacheMode::Uncached);

        if (result.Status() == GattCommunicationStatus::Success)
        {
            std::lock_guard<std::mutex> lock(serviceQueueLock);
            for (auto&& service : result.Services())
            {
                Service serviceStruct;
                std::wstring uuidWString = winrt::to_hstring(service.Uuid()).c_str();
                wcscpy_s(serviceStruct.uuid, sizeof(serviceStruct.uuid) / sizeof(wchar_t), uuidWString.c_str());

                std::string message = "Service found: UUID = " + ConvertHStringToString(serviceStruct.uuid);
                LOG(message.c_str());

                serviceQueue.push(serviceStruct);
            }
        }
        else
        {
            LOG_ERROR("Failed to retrieve GATT services.");
            std::lock_guard<std::mutex> lock(serviceQueueLock);
            serviceScanError = true;
        }
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during ScanServicesAsync: " + std::string(ex.what());
        LOG_ERROR(error_message.c_str());
        std::lock_guard<std::mutex> lock(serviceQueueLock);
        serviceScanError = true;
    }

    {
        std::lock_guard<std::mutex> lock(serviceQueueLock);
        serviceScanFinished = true;
    }
    serviceQueueSignal.notify_all();
}

void ScanServices(wchar_t* deviceId) {
    ScanServicesAsync(deviceId);
}

// Function to poll for services
ScanStatus PollService(Service* service, bool block)
{
    try
    {
        LOG("Polling for service...");

        lock_guard<mutex> lock(serviceQueueLock);

        if (serviceScanError)
        {
            LOG_ERROR("Service scan error detected.");
            return ScanStatus::Failed;
        }

        if (!serviceQueue.empty())
        {
            *service = serviceQueue.front();
            serviceQueue.pop();

            std::string message = "Service available: UUID = " + ConvertHStringToString(service->uuid);
            LOG(message.c_str());

            return ScanStatus::Available;
        }

        if (serviceScanFinished)
        {
            LOG("Service scan finished.");
            return ScanStatus::Finished;
        }

        LOG("Service scan still processing.");
        return ScanStatus::Processing;
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during PollService: " + std::string(ex.what());
        LOG_ERROR(error_message.c_str());
        return ScanStatus::Failed;
    }
}

// Function to scan characteristics asynchronously
fire_and_forget ScanCharacteristicsAsync(wchar_t* deviceId, wchar_t* serviceId)
{
    try
    {
        LOG("Starting characteristic scan...");
        characteristicScanFinished = false;
        characteristicScanError = false;
        characteristicQueue = {}; // Clear the queue

        auto service = co_await retrieveService(deviceId, serviceId);

        if (service == nullptr)
        {
            LOG_ERROR("Failed to retrieve service.");
            characteristicScanError = true;
            co_return;
        }

        auto result = co_await service.GetCharacteristicsAsync(BluetoothCacheMode::Uncached);

        if (result.Status() == GattCommunicationStatus::Success)
        {
            LOG("Characteristic scan successful, processing characteristics...");
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
                LOG(message.c_str());

                {
                    lock_guard<mutex> lock(characteristicQueueLock);
                    characteristicQueue.push(charStruct);

                    std::string message = "Characteristic added to queue: UUID = " + ConvertHStringToString(winrt::to_hstring(characteristic.Uuid()));
                    LOG(message.c_str());
                }

               

                // Add a slight delay if needed to avoid tight loops
                co_await resume_background(); // This can yield to other operations if necessary
            }
        }
        else
        {
            LOG_ERROR("Failed to retrieve characteristics.");
            characteristicScanError = true;
        }
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during ScanCharacteristicsAsync: " + std::string(ex.what());
        LOG_ERROR(error_message.c_str());
        characteristicScanError = true;
    }

    lock_guard<mutex> lock(characteristicQueueLock);
    characteristicScanFinished = true;
    LOG("Characteristic scan completed.");
}

void ScanCharacteristics(wchar_t* deviceId, wchar_t* serviceId) {
    ScanCharacteristicsAsync(deviceId, serviceId);
}

ScanStatus PollCharacteristic(Characteristic* characteristic, bool block)
{
    try
    {
        LOG("Polling for characteristic...");

        auto start_time = std::chrono::steady_clock::now();
       

        while (block && std::chrono::steady_clock::now() - start_time < timeout)
        {
            {
        
                lock_guard<mutex> lock(characteristicQueueLock);
                if (characteristicScanError)
                {
                    LOG_ERROR("Characteristic scan error detected.");
                    return ScanStatus::Failed;
                }

                if (!characteristicQueue.empty())
                {
                    *characteristic = characteristicQueue.front();
                    characteristicQueue.pop();

                    std::string message = "Characteristic available: UUID = " + ConvertHStringToString(winrt::to_hstring(characteristic->uuid)) + ", Description = " + ConvertHStringToString(winrt::to_hstring(characteristic->userDescription));
                    LOG(message.c_str());

                    return ScanStatus::Available;
                }

                if (characteristicScanFinished)
                {
                    LOG("Characteristic scan finished.");
                    return ScanStatus::Finished;
                }

                LOG("Characteristic scan still processing.");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small sleep to avoid tight loops
        }

        LOG_ERROR("Polling for characteristic timed out.");
        return ScanStatus::Failed;
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during PollCharacteristic: " + std::string(ex.what());
        LOG_ERROR(error_message.c_str());
        return ScanStatus::Failed;
    }
}


#pragma region CONNECT_DEVICE



fire_and_forget ConnectDeviceAsync(wchar_t* deviceId)
{
    LOG("ConnectDeviceAsync started.");
    {
        std::lock_guard<std::mutex> lock(connectionStatusLock);
        deviceConnectionStatusMap[std::wstring(deviceId)] = ConnectionStatus::Connecting;
    }
    try
    {
        auto start_time = std::chrono::steady_clock::now();
        BluetoothLEDevice device = nullptr;
        LOG("Initiating BluetoothLEDevice::FromIdAsync.");

        {
            std::lock_guard<std::mutex> lock(deviceLock);
            device = co_await BluetoothLEDevice::FromIdAsync(deviceId);
            deviceMap[std::wstring(deviceId)] = { device };
        }
       

        if (device == nullptr)
        {
            LOG_ERROR("BluetoothLEDevice::FromIdAsync returned nullptr. Could not connect to the device.");
            {
                std::lock_guard<std::mutex> lock(connectionStatusLock);
                deviceConnectionStatusMap[std::wstring(deviceId)] = ConnectionStatus::Failed;
            }
        }
        else
        {
            LOG("Device connected successfully.");
            {
                std::lock_guard<std::mutex> lock(connectionStatusLock);
                deviceConnectionStatusMap[std::wstring(deviceId)] = ConnectionStatus::Connected;
            }

            device.ConnectionStatusChanged([deviceId](BluetoothLEDevice const& sender, IInspectable const&)
                {
                    if (sender.ConnectionStatus() == BluetoothConnectionStatus::Disconnected)
                    {
                        LOG("Device disconnected.");
                        {
                            std::lock_guard<std::mutex> lock(connectionStatusLock);
                            deviceConnectionStatusMap[std::wstring(deviceId)] = ConnectionStatus::Disconnected;
                        }
                    }
                });
        }

        if (std::chrono::steady_clock::now() - start_time > timeout)
        {
            LOG_ERROR("Device connection timed out.");
            {
                std::lock_guard<std::mutex> lock(connectionStatusLock);
                deviceConnectionStatusMap[std::wstring(deviceId)] = ConnectionStatus::Timeout;
            }
        }
    }
    catch (const std::exception& ex)
    {
        LOG_ERROR("Exception occurred: ", ex.what());
        {
            std::lock_guard<std::mutex> lock(connectionStatusLock);
            deviceConnectionStatusMap[std::wstring(deviceId)] = ConnectionStatus::Unknown_Error;
        }
    }
    isConnecting = false;
    LOG("ConnectDeviceAsync finished.");
}



bool ConnectDevice(wchar_t* deviceId)
{
    if (isConnecting)
    {
        LOG_ERROR("Connection attempt already in progress.");
        return false;
    }

    isConnecting = true;
    LOG("ConnectDevice started.");

    ConnectDeviceAsync(deviceId);
    return true;

}

ConnectionStatus PollConnectionStatusForDevice(wchar_t* deviceId)
{
    try
    {
        return deviceConnectionStatusMap[std::wstring(deviceId)];
    }
    catch (exception)
    {
        return ConnectionStatus::Unknown_Error;
    }
}

bool DisconnectDevice(wchar_t* deviceId)
{
    if (deviceMap[std::wstring(deviceId)].device == nullptr)
    {
        LOG_ERROR("Device not connected");
        return false;
    }

    
    LOG("DisconnectDevice started.");
    deviceMap[std::wstring(deviceId)].device.Close();
    deviceMap[std::wstring(deviceId)] = {};


    return true;  // Return true to indicate the connection process has started
}


#pragma endregion


fire_and_forget SubscribeCharacteristicAsync(const wchar_t* deviceId, const wchar_t* serviceId, const wchar_t* characteristicId, bool* result, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
{
    try
    {
        LOG("Starting characteristic subscription...");

        auto start_time = std::chrono::steady_clock::now();

        auto characteristic = co_await retrieveCharacteristic(deviceId, serviceId, characteristicId);

        // Check for timeout
        if (std::chrono::steady_clock::now() - start_time > timeout)
        {
            LOG_ERROR("Characteristic retrieval timed out.");
            if (result != nullptr)
            {
                *result = false;
            }
            co_return;
        }

        if (characteristic == nullptr)
        {
            LOG_ERROR("Failed to retrieve characteristic for subscription.");
            if (result != nullptr)
            {
                *result = false;
            }
            co_return;
        }

        LOG("Retrieved characteristic, attempting to subscribe...");

        GattCharacteristicProperties properties = characteristic.CharacteristicProperties();
        if ((properties & GattCharacteristicProperties::Notify) == GattCharacteristicProperties::None &&
            (properties & GattCharacteristicProperties::Indicate) == GattCharacteristicProperties::None)
        {
            LOG_ERROR("Characteristic does not support Notify or Indicate. Subscription aborted.");
            if (result != nullptr)
            {
                *result = false;
            }
            co_return;
        }

        auto status = co_await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue::Notify);

        // Check for timeout
        if (std::chrono::steady_clock::now() - start_time > timeout)
        {
            LOG_ERROR("Characteristic subscription timed out.");
            if (result != nullptr)
            {
                *result = false;
            }
            co_return;
        }

        if (status != GattCommunicationStatus::Success)
        {
           
            LOG_ERROR("Error subscribing to characteristic with UUID: ", guid_to_string(characteristic.Uuid()), ", Status: ", std::to_string(static_cast<int>(status)));
            if (result != nullptr)
            {
                *result = false;
            }
        }
        else
        {
            std::string message = "Successfully subscribed to characteristic with UUID: " + guid_to_string(characteristic.Uuid());
            LOG(message.c_str());

            auto subscription = std::make_unique<Subscription>();
            subscription->characteristic = characteristic;

            subscription->revoker = characteristic.ValueChanged(auto_revoke, [deviceId, serviceId, characteristicId](GattCharacteristic const&, GattValueChangedEventArgs const& args)
                {
                    try
                    {
                        LOG("ValueChanged event triggered.");

                        BLEData bleData;
                        bleData.size = args.CharacteristicValue().Length();
                        memcpy(bleData.buf, args.CharacteristicValue().data(), bleData.size);

                        LOG("Data received from ValueChanged event, populating BLEData.");

                        // Populate the BLEData with device, service, and characteristic IDs
                        wcscpy_s(bleData.deviceId, sizeof(bleData.deviceId) / sizeof(wchar_t), deviceId);
                        wcscpy_s(bleData.serviceUuid, sizeof(bleData.serviceUuid) / sizeof(wchar_t), serviceId);
                        wcscpy_s(bleData.characteristicUuid, sizeof(bleData.characteristicUuid) / sizeof(wchar_t), characteristicId);

                        {
                            std::lock_guard<std::mutex> lock(dataQueueLock);
                            dataQueue.push(bleData);
                            LOG("Data enqueued into dataQueue.");
                            dataQueueSignal.notify_one();
                        }

                        std::string dataMessage = "Data received: Size = " + std::to_string(bleData.size);
                        LOG(dataMessage.c_str());
                    }
                    catch (const std::exception& ex)
                    {
                        std::string error_message = "Error during ValueChanged event handling: " + std::string(ex.what());
                        LOG_ERROR(error_message.c_str());
                    }
                });

            {
                std::lock_guard<std::mutex> lock(subscribeQueueLock);
                subscriptions.push_back(std::move(subscription));
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
        LOG_ERROR(error_message.c_str());
        if (result != nullptr)
        {
            *result = false;
        }
    }

    LOG("Characteristic subscription process completed.");
    subscribeQueueSignal.notify_one();
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

    LOG("Subscription to characteristic ", ConvertHStringToString(winrt::to_hstring(characteristicId)) + " was " + (result ? "successful" : "unsuccessful"));

    return result;
}

bool PollData(BLEData* data, bool block)
{
    try
    {
        LOG("Polling for BLE data...");

        std::unique_lock<std::mutex> lock(dataQueueLock);

        if (block)
        {
            dataQueueSignal.wait(lock, [] { return !dataQueue.empty() || IsQuitFlagSet(); });
        }

        if (IsQuitFlagSet())
        {
            LOG("Quit flag set, aborting PollData.");
            return false;
        }

        if (!dataQueue.empty())
        {
            *data = std::move(dataQueue.front());
            dataQueue.pop();

            std::string message = "Data available: Device ID = " + ConvertHStringToString(data->deviceId) +
                ", Service UUID = " + ConvertHStringToString(data->serviceUuid) +
                ", Characteristic UUID = " + ConvertHStringToString(data->characteristicUuid) +
                ", Data Size = " + std::to_string(data->size);
            LOG(message.c_str());

            return true;
        }

        LOG("No data available.");
        return false;
    }
    catch (const std::exception& ex)
    {
        std::string error_message = "Error during PollData: " + std::string(ex.what());
        LOG_ERROR(error_message.c_str());
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
                LOG_ERROR("%s:%d Error writing value to characteristic with uuid %s", __FILE__, __LINE__, data.characteristicUuid);
            }
            else if (result) {
                *result = true;
            }
        }
    }
    catch (hresult_error& ex) {
        LOG_ERROR("%s:%d SendDataAsync catch: %s", __FILE__, __LINE__, ex.message().c_str());
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



fire_and_forget ReadCharacteristicDataAsync(wchar_t* deviceId, wchar_t* serviceId, wchar_t* characteristicId, bool* result, bool block) {
    try {
        LOG("Started ReadCharacteristicData...");

        auto characteristic = co_await retrieveCharacteristic(deviceId, serviceId, characteristicId);
        if (characteristic == nullptr) {
            LOG_ERROR("Failed to retrieve characteristic for data read.");
            if (result != nullptr) {
                *result = false;
            }
            co_return;
        }

        // Read the characteristic value
        auto readResult = co_await characteristic.ReadValueAsync();
        if (readResult.Status() != GattCommunicationStatus::Success) {
            std::string message = "Error reading characteristic data with UUID: " + ConvertHStringToString(winrt::to_hstring(characteristic.Uuid())) + ", Status: " + std::to_string(static_cast<int>(readResult.Status()));
            LOG_ERROR(message.c_str());
            if (result != nullptr) {
                *result = false;
            }
        }
        else {
            // Process the data
            auto buffer = readResult.Value();
            BLEData bleData;
            bleData.size = buffer.Length();
            memcpy(bleData.buf, buffer.data(), bleData.size);

            // Populate the BLEData with device, service, and characteristic IDs
            wcscpy_s(bleData.deviceId, sizeof(bleData.deviceId) / sizeof(wchar_t), deviceId);
            wcscpy_s(bleData.serviceUuid, sizeof(bleData.serviceUuid) / sizeof(wchar_t), serviceId);
            wcscpy_s(bleData.characteristicUuid, sizeof(bleData.characteristicUuid) / sizeof(wchar_t), characteristicId);

            // Lock the queue and add data, with queue size management
            {
                lock_guard<mutex> lock(dataQueueLock);
                if (dataQueue.size() > 1000) { // Limit queue size to 1000 entries, adjust as needed
                    LOG("Data queue full, dropping oldest data.");
                    dataQueue.pop(); // Drop oldest data if queue is full
                }
                dataQueue.push(bleData);
                LOG("Data enqueued into dataQueue.");
                dataQueueSignal.notify_one();
            }

            std::string dataMessage = "Data received: Size = " + std::to_string(bleData.size);
            LOG(dataMessage.c_str());

            if (result != nullptr) {
                *result = true;
            }
        }
    }
    catch (const std::exception& ex) {
        std::string error_message = "Error during ReadCharacteristicDataAsync: " + std::string(ex.what());
        LOG_ERROR(error_message.c_str());
        if (result != nullptr) {
            *result = false;
        }
    }
    catch (...) {
        LOG_ERROR("Unknown error occurred during ReadCharacteristicDataAsync.");
        if (result != nullptr) {
            *result = false;
        }
    }

    if (result != nullptr) {
        if (block) {
            dataQueueSignal.notify_one();
        }
    }

    LOG("ReadCharacteristicData completed.");
}

bool ReadCharacteristicData(wchar_t* deviceId, wchar_t* serviceId, wchar_t* characteristicId, bool block) {
    bool result = false;

    {
        unique_lock<mutex> lock(dataQueueLock);
        ReadCharacteristicDataAsync(deviceId, serviceId, characteristicId, block ? &result : nullptr, block);

        if (block) {
            dataQueueSignal.wait(lock); // Wait for data to be enqueued
        }
    }

    std::string message = "ReadCharacteristicData from characteristic " + ConvertHStringToString(winrt::to_hstring(characteristicId)) + " was " + (result ? "successful" : "unsuccessful");
    LOG(message.c_str());

    return result;
}



void SetQuitFlag()
{
    std::lock_guard<std::mutex> lock(quitLock);
    quitFlag = true;
    // Notify all condition variables
    deviceQueueSignal.notify_all();
    serviceQueueSignal.notify_all();
    characteristicQueueSignal.notify_all();
    subscribeQueueSignal.notify_all();
    dataQueueSignal.notify_all();
}




void Quit() {
    LOG("Cleaning up and shutting down...");
    SetQuitFlag();

    StopDeviceScan();
    // Stop and clean up the DeviceWatcher
    if (deviceWatcher != nullptr) {
        LOG("Stopping DeviceWatcher...");
        deviceWatcher.Stop();
        deviceWatcher = nullptr;
    }

    // Clean up BluetoothLEDevice connections
    {
        std::unique_lock<std::shared_mutex> lock(cacheMutex);
        for (auto& [_, deviceEntry] : cache) {
            if (deviceEntry.device != nullptr) {
                LOG("Closing active BluetoothLEDevice connection...");
                deviceEntry.device.Close();
                deviceEntry.device = nullptr;
            }
        }
        cache.clear();
    }
    {
        // Reset connection state
        LOG("About to acquire connectLock...");
        {
            std::lock_guard<std::mutex> lock(connectLock);
            for (auto const& [id, deviceCache] : deviceMap)
            {
                auto bluetoothLeDevice = deviceCache.device;
                if (bluetoothLeDevice != nullptr) {
                    LOG("Closing active BluetoothLEDevice connection...");
                    bluetoothLeDevice.Close();
                }
                LOG("BluetoothLEDevice connection reset.");
            }
            deviceMap.clear();
        }
        LOG("Released connectLock.");
    }
    

    // Clean up subscriptions
    {
        std::lock_guard<std::mutex> lock(subscribeQueueLock);
        for (auto& subscription : subscriptions) {
            subscription->revoker.revoke();
            // No need to call delete, unique_ptr will handle it
        }
        subscriptions.clear(); // This will delete all Subscription objects
    }
    ResetPluginState();
    LOG("Cleanup completed.");
}

