#include "LJV7Controller.h"
#include <sstream>
#include <iostream>
#include <cassert>

LJController::LJController(const std::wstring& dllPath)
    : hModule_(NULL),
    useExplicitLoad_(false),
    fpInitialize_(nullptr),
    fpFinalize_(nullptr),
    fpGetVersion_(nullptr),
    fpUsbOpen_(nullptr),
    fpEthernetOpen_(nullptr),
    fpCommClose_(nullptr),
    fpStartMeasure_(nullptr),
    fpStopMeasure_(nullptr),
    fpTrigger_(nullptr),
    fpGetMeasurementValue_(nullptr),
    fpGetProfile_(nullptr),
    fpHSInitUSB_(nullptr),
    fpHSInitEthernet_(nullptr),
    fpHSPreStart_(nullptr),
    fpHSStart_(nullptr),
    fpHSStop_(nullptr),
    fpHSFinalize_(nullptr),
    userCallbackPtr_(nullptr),
    hsUserParam_(0)
{
    if (!dllPath.empty()) {
        std::lock_guard<std::mutex> lk(dllMutex_);
        hModule_ = ::LoadLibraryW(dllPath.c_str());
        if (!hModule_) {
            std::stringstream ss;
            ss << "Не удалось загрузить DLL: " << std::string(dllPath.begin(), dllPath.end());
            throw LJException(ss.str());
        }
        useExplicitLoad_ = true;
        LoadFunctions();
    } else {
        useExplicitLoad_ = false;
    }
}

LJController::~LJController()
{
    try {
        Finalize();
    } catch(...) {
        //
    }
    if (useExplicitLoad_ && hModule_) {
        FreeLibrary(hModule_);
        hModule_ = NULL;
    }
}

bool LJController::IsLoaded() const {
    return hModule_ != NULL || !useExplicitLoad_;
}

void LJController::EnsureLoaded() {
    std::lock_guard<std::mutex> lk(dllMutex_);
    if (hModule_ == NULL && useExplicitLoad_) {
        throw LJException("DLL не загружена");
    }
    if (hModule_ == NULL && !useExplicitLoad_) {
        HMODULE m = ::LoadLibraryW(L"LJV7_IF.dll");
        if (!m) {
            throw LJException("Не удалось найти LJV7_IF.dll (попробуйте указать путь в конструкторе или положить DLL рядом с .exe)");
        }
        hModule_ = m;
        LoadFunctions();
        useExplicitLoad_ = true;
    }
}

void LJController::LoadFunctions() {
    assert(hModule_);
#define LOAD_FN(name) fp##name##_ = reinterpret_cast<t##name>(GetProcAddress(hModule_, #name)); \
    if (!fp##name##_) { std::stringstream ss; ss << "GetProcAddress failed: " << #name; throw LJException(ss.str()); }

    fpInitialize_ = reinterpret_cast<tLJV7IF_Initialize>(GetProcAddress(hModule_, "LJV7IF_Initialize"));
    fpFinalize_ = reinterpret_cast<tLJV7IF_Finalize>(GetProcAddress(hModule_, "LJV7IF_Finalize"));
    fpGetVersion_ = reinterpret_cast<tLJV7IF_GetVersion>(GetProcAddress(hModule_, "LJV7IF_GetVersion"));
    fpUsbOpen_ = reinterpret_cast<tLJV7IF_UsbOpen>(GetProcAddress(hModule_, "LJV7IF_UsbOpen"));
    fpEthernetOpen_ = reinterpret_cast<tLJV7IF_EthernetOpen>(GetProcAddress(hModule_, "LJV7IF_EthernetOpen"));
    fpCommClose_ = reinterpret_cast<tLJV7IF_CommClose>(GetProcAddress(hModule_, "LJV7IF_CommClose"));
    fpStartMeasure_ = reinterpret_cast<tLJV7IF_StartMeasure>(GetProcAddress(hModule_, "LJV7IF_StartMeasure"));
    fpStopMeasure_ = reinterpret_cast<tLJV7IF_StopMeasure>(GetProcAddress(hModule_, "LJV7IF_StopMeasure"));
    fpTrigger_ = reinterpret_cast<tLJV7IF_Trigger>(GetProcAddress(hModule_, "LJV7IF_Trigger"));
    fpGetMeasurementValue_ = reinterpret_cast<tLJV7IF_GetMeasurementValue>(GetProcAddress(hModule_, "LJV7IF_GetMeasurementValue"));
    fpGetProfile_ = reinterpret_cast<tLJV7IF_GetProfile>(GetProcAddress(hModule_, "LJV7IF_GetProfile"));
    fpHSInitUSB_ = reinterpret_cast<tLJV7IF_HighSpeedDataUSBCommunicationInitalize>(GetProcAddress(hModule_, "LJV7IF_HighSpeedDataUSBCommunicationInitalize"));
    fpHSInitEthernet_ = reinterpret_cast<tLJV7IF_HighSpeedDataEthernetCommunicationInitalize>(GetProcAddress(hModule_, "LJV7IF_HighSpeedDataEthernetCommunicationInitalize"));
    fpHSPreStart_ = reinterpret_cast<tLJV7IF_PreStartHighSpeedDataCommunication>(GetProcAddress(hModule_, "LJV7IF_PreStartHighSpeedDataCommunication"));
    fpHSStart_ = reinterpret_cast<tLJV7IF_StartHighSpeedDataCommunication>(GetProcAddress(hModule_, "LJV7IF_StartHighSpeedDataCommunication"));
    fpHSStop_ = reinterpret_cast<tLJV7IF_StopHighSpeedDataCommunication>(GetProcAddress(hModule_, "LJV7IF_StopHighSpeedDataCommunication"));
    fpHSFinalize_ = reinterpret_cast<tLJV7IF_HighSpeedDataCommunicationFinalize>(GetProcAddress(hModule_, "LJV7IF_HighSpeedDataCommunicationFinalize"));

    if (!fpInitialize_ || !fpFinalize_ || !fpGetVersion_ || !fpUsbOpen_ || !fpCommClose_ || !fpGetMeasurementValue_) {
        throw LJException("Некоторые требуемые функции не найдены в DLL");
    }
}

void LJController::Initialize() {
    EnsureLoaded();
    LONG rc = fpInitialize_();
    if (rc != 0) {
        ThrowIfError(rc, "LJV7IF_Initialize");
    }
}

void LJController::Finalize() {
    std::lock_guard<std::mutex> lk(dllMutex_);
    if (fpFinalize_) {
        fpFinalize_();
    }
}

void LJController::UsbOpen(LONG deviceId) {
    EnsureLoaded();
    LONG rc = fpUsbOpen_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_UsbOpen");
}

void LJController::EthernetOpen(LONG deviceId, const LJV7IF_ETHERNET_CONFIG& cfg) {
    EnsureLoaded();
    LONG rc = fpEthernetOpen_(deviceId, const_cast<LJV7IF_ETHERNET_CONFIG*>(&cfg));
    if (rc != 0) ThrowIfError(rc, "LJV7IF_EthernetOpen");
}

void LJController::CommClose(LONG deviceId) {
    EnsureLoaded();
    fpCommClose_(deviceId);
}

void LJController::StartMeasure(LONG deviceId) {
    EnsureLoaded();
    LONG rc = fpStartMeasure_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_StartMeasure");
}

void LJController::StopMeasure(LONG deviceId) {
    EnsureLoaded();
    LONG rc = fpStopMeasure_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_StopMeasure");
}

void LJController::Trigger(LONG deviceId) {
    EnsureLoaded();
    LONG rc = fpTrigger_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_Trigger");
}

std::vector<LJV7IF_MEASURE_DATA> LJController::GetMeasurementValue(LONG deviceId) {
    EnsureLoaded();
    std::vector<LJV7IF_MEASURE_DATA> out(16);
    LONG rc = fpGetMeasurementValue_(deviceId, out.data());
    if (rc != 0) ThrowIfError(rc, "LJV7IF_GetMeasurementValue");
    return out;
}

std::vector<DWORD> LJController::GetProfile(LONG deviceId, const LJV7IF_GET_PROFILE_REQ& req, LJV7IF_GET_PROFILE_RSP& rsp, LJV7IF_PROFILE_INFO& info) {
    EnsureLoaded();

    const size_t BUFSIZE = 1024 * 1024; // 1 MB в DWORD (4MB)
    std::vector<DWORD> buffer(BUFSIZE / sizeof(DWORD));
    DWORD dataSizeBytes = static_cast<DWORD>(buffer.size() * sizeof(DWORD));
    LJV7IF_GET_PROFILE_REQ localReq = req;
    LONG rc = fpGetProfile_(deviceId, &localReq, &rsp, &info, buffer.data(), dataSizeBytes);
    if (rc != 0) {
        if (rc == 0x8080) ThrowIfError(rc, "LJV7IF_GetProfile: Неверный режим (advanced/high-speed?)");
        ThrowIfError(rc, "LJV7IF_GetProfile");
    }
    return buffer;
}

// High speed
void LJController::SetHighSpeedCallback(RawProfileCallback cb) {
    // Save the pointer to std::function in heap and pass the pointer to atomic
    if (!cb) {
        RawProfileCallback* old = userCallbackPtr_.exchange(nullptr);
        if (old) { delete old; }
        return;
    }
    RawProfileCallback* p = new RawProfileCallback(std::move(cb));
    RawProfileCallback* old = userCallbackPtr_.exchange(p);
    if (old) delete old;
}

void LJController::HighSpeedInitUSB(LONG deviceId, DWORD dwUser) {
    EnsureLoaded();
    hsUserParam_ = dwUser;
    if (!fpHSInitUSB_) throw LJException("High-speed USB init function not found");
    LONG rc = fpHSInitUSB_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_HighSpeedDataUSBCommunicationInitalize");
    // Registration of the internal callback takes place in StartHighSpeed
}

void LJController::HighSpeedInitEthernet(LONG deviceId, DWORD dwUser) {
    EnsureLoaded();
    hsUserParam_ = dwUser;
    if (!fpHSInitEthernet_) throw LJException("High-speed Ethernet init function not found");
    LONG rc = fpHSInitEthernet_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_HighSpeedDataEthernetCommunicationInitalize");
}

void LJController::PreStartHighSpeed(LONG deviceId, const LJV7IF_HIGH_SPEED_PRE_START_REQ& req) {
    EnsureLoaded();
    if (!fpHSPreStart_) throw LJException("High-speed PreStart function not found");
    LJV7IF_HIGH_SPEED_PRE_START_REQ local = req;
    LONG rc = fpHSPreStart_(deviceId, &local);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_PreStartHighSpeedDataCommunication");
}

void LJController::StartHighSpeed(LONG deviceId) {
    EnsureLoaded();
    if (!fpHSStart_) throw LJException("High-speed Start function not found");
    // In the StartHighSpeedDataCommunication library, it accepts a parameter function in some implementations.
    // Here we pass the address of our trampoline (C-style), if required.
    LONG rc = fpHSStart_(deviceId, reinterpret_cast<void*>(&LJController::InternalHighSpeedCallback));
    if (rc != 0) ThrowIfError(rc, "LJV7IF_StartHighSpeedDataCommunication");
}

void LJController::StopHighSpeed(LONG deviceId) {
    EnsureLoaded();
    if (!fpHSStop_) throw LJException("High-speed Stop function not found");
    LONG rc = fpHSStop_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_StopHighSpeedDataCommunication");
}

void LJController::FinalizeHighSpeed(LONG deviceId) {
    EnsureLoaded();
    if (!fpHSFinalize_) throw LJException("High-speed Finalize function not found");
    LONG rc = fpHSFinalize_(deviceId);
    if (rc != 0) ThrowIfError(rc, "LJV7IF_HighSpeedDataCommunicationFinalize");
}

// Static trampoline
void __stdcall LJController::InternalHighSpeedCallback(BYTE* pBuffer, DWORD dwSize, DWORD dwCount, DWORD dwNotify, DWORD dwUser) {
    extern LJController* g_last_instance;
    if (!g_last_instance) return;
    RawProfileCallback* pfn = g_last_instance->userCallbackPtr_.load();
    if (!pfn) return;
    try {
        (*pfn)(pBuffer, dwSize, dwCount, dwNotify);
    } catch (...) {
        //
    }
}

// A small global variable for callback binding is a limitation of the current implementation.
LJController* g_last_instance = nullptr;

// Auxiliary
void LJController::ThrowIfError(LONG rc, const std::string& prefix) {
    std::stringstream ss;
    ss << prefix << " вернул код: 0x" << std::hex << (rc & 0xFFFF);
    throw LJException(ss.str());
}

std::string LJController::ErrorCodeToString(LONG code) const {
    std::stringstream ss;
    ss << "0x" << std::hex << (code & 0xFFFF);
    return ss.str();
}
