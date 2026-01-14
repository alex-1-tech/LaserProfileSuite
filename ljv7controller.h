#pragma once

/**
 * @file    LJV7Controller.h
 * @brief   Wrapper class for Keyence LJV7 laser controller DLL
 * @author  alex-1-tech
 * @date    2025
 */

#include <windows.h>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>

#include "lib/LJV7_IF.h"
#include "lib/LJV7_ErrorCode.h"

/**
 * @brief Exception class for LJV7 controller errors
 */
class LJException : public std::runtime_error {
public:
    explicit LJException(const std::string& msg) : std::runtime_error(msg) {}
};

/**
 * @class LJController
 * @brief C++ wrapper for Keyence LJV7 laser measurement controller
 *
 * Provides RAII-style management of LJV7 DLL functions with:
 * - Dynamic DLL loading (explicit or implicit)
 * - Thread-safe operation
 * - USB and Ethernet connectivity
 * - Standard and high-speed measurement modes
 * - Error handling via exceptions
 * - Callback support for real-time data
 */
class LJController {
public:
    /// @brief Callback type for high-speed profile data
    using RawProfileCallback = std::function<void(const BYTE* buffer, DWORD unitSize, DWORD count, DWORD notify)>;

    explicit LJController(const std::wstring& dllPath = L"");  ///< Constructor with optional DLL path
    ~LJController();                                          ///< Destructor with cleanup

    void Initialize();                                        ///< Initialize LJV7 library
    void Finalize();                                          ///< Finalize LJV7 library

    bool IsLoaded() const;                                    ///< Check if DLL is loaded

    void UsbOpen(LONG deviceId = 0);                          ///< Open USB connection to device
    void EthernetOpen(LONG deviceId, const LJV7IF_ETHERNET_CONFIG& cfg);  ///< Open Ethernet connection
    void CommClose(LONG deviceId = 0);                        ///< Close communication

    void StartMeasure(LONG deviceId = 0);                     ///< Start standard measurement
    void StopMeasure(LONG deviceId = 0);                      ///< Stop standard measurement
    void Trigger(LONG deviceId = 0);                          ///< Send trigger to device

    std::vector<LJV7IF_MEASURE_DATA> GetMeasurementValue(LONG deviceId = 0);  ///< Get measurement values

    std::vector<DWORD> GetProfile(LONG deviceId, const LJV7IF_GET_PROFILE_REQ& req,
                                  LJV7IF_GET_PROFILE_RSP& rsp, LJV7IF_PROFILE_INFO& info);  ///< Get profile data

    void SetHighSpeedCallback(RawProfileCallback cb);               ///< Set callback for high-speed data

    void HighSpeedInitUSB(LONG deviceId = 0, DWORD dwUser = 0);     ///< Initialize USB high-speed mode
    void HighSpeedInitEthernet(LONG deviceId, DWORD dwUser = 0);    ///< Initialize Ethernet high-speed mode
    void PreStartHighSpeed(LONG deviceId, const LJV7IF_HIGH_SPEED_PRE_START_REQ& req);  ///< Prepare high-speed mode
    void StartHighSpeed(LONG deviceId = 0);                         ///< Start high-speed data acquisition
    void StopHighSpeed(LONG deviceId = 0);                          ///< Stop high-speed data acquisition
    void FinalizeHighSpeed(LONG deviceId = 0);                      ///< Finalize high-speed mode

    std::string ErrorCodeToString(LONG code) const;           ///< Convert error code to string

private:
    HMODULE hModule_;                                         ///< DLL module handle
    bool useExplicitLoad_;                                    ///< True if DLL was explicitly loaded
    std::mutex dllMutex_;                                     ///< Mutex for thread-safe DLL operations

    // Function pointers for DLL functions
    typedef LONG (__stdcall *tLJV7IF_Initialize)(void);
    typedef LONG (__stdcall *tLJV7IF_Finalize)(void);
    typedef DWORD (__stdcall *tLJV7IF_GetVersion)(void);
    typedef LONG (__stdcall *tLJV7IF_UsbOpen)(LONG);
    typedef LONG (__stdcall *tLJV7IF_EthernetOpen)(LONG, LJV7IF_ETHERNET_CONFIG*);
    typedef LONG (__stdcall *tLJV7IF_CommClose)(LONG);
    typedef LONG (__stdcall *tLJV7IF_StartMeasure)(LONG);
    typedef LONG (__stdcall *tLJV7IF_StopMeasure)(LONG);
    typedef LONG (__stdcall *tLJV7IF_Trigger)(LONG);
    typedef LONG (__stdcall *tLJV7IF_GetMeasurementValue)(LONG, LJV7IF_MEASURE_DATA*);
    typedef LONG (__stdcall *tLJV7IF_GetProfile)(LONG, LJV7IF_GET_PROFILE_REQ*, LJV7IF_GET_PROFILE_RSP*, LJV7IF_PROFILE_INFO*, DWORD*, DWORD);
    typedef LONG (__stdcall *tLJV7IF_HighSpeedDataUSBCommunicationInitalize)(LONG);
    typedef LONG (__stdcall *tLJV7IF_HighSpeedDataEthernetCommunicationInitalize)(LONG);
    typedef LONG (__stdcall *tLJV7IF_PreStartHighSpeedDataCommunication)(LONG, LJV7IF_HIGH_SPEED_PRE_START_REQ*);
    typedef LONG (__stdcall *tLJV7IF_StartHighSpeedDataCommunication)(LONG, void*);
    typedef LONG (__stdcall *tLJV7IF_StopHighSpeedDataCommunication)(LONG);
    typedef LONG (__stdcall *tLJV7IF_HighSpeedDataCommunicationFinalize)(LONG);

    tLJV7IF_Initialize fpInitialize_;
    tLJV7IF_Finalize fpFinalize_;
    tLJV7IF_GetVersion fpGetVersion_;
    tLJV7IF_UsbOpen fpUsbOpen_;
    tLJV7IF_EthernetOpen fpEthernetOpen_;
    tLJV7IF_CommClose fpCommClose_;
    tLJV7IF_StartMeasure fpStartMeasure_;
    tLJV7IF_StopMeasure fpStopMeasure_;
    tLJV7IF_Trigger fpTrigger_;
    tLJV7IF_GetMeasurementValue fpGetMeasurementValue_;
    tLJV7IF_GetProfile fpGetProfile_;
    tLJV7IF_HighSpeedDataUSBCommunicationInitalize fpHSInitUSB_;
    tLJV7IF_HighSpeedDataEthernetCommunicationInitalize fpHSInitEthernet_;
    tLJV7IF_PreStartHighSpeedDataCommunication fpHSPreStart_;
    tLJV7IF_StartHighSpeedDataCommunication fpHSStart_;
    tLJV7IF_StopHighSpeedDataCommunication fpHSStop_;
    tLJV7IF_HighSpeedDataCommunicationFinalize fpHSFinalize_;

    static void __stdcall InternalHighSpeedCallback(BYTE* pBuffer, DWORD dwSize, DWORD dwCount, DWORD dwNotify, DWORD dwUser);  ///< Internal callback trampoline
    std::atomic<RawProfileCallback*> userCallbackPtr_;        ///< Atomic pointer to user callback
    DWORD hsUserParam_;                                       ///< User parameter for high-speed mode

    void LoadFunctions();                                     ///< Load DLL function pointers
    void EnsureLoaded();                                      ///< Ensure DLL is loaded before use
    void ThrowIfError(LONG rc, const std::string& prefix);    ///< Throw exception if error code != 0
};
