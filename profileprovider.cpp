#define NOMINMAX    ///< Disable Windows min/max macros to avoid conflicts with std::min/max
#include <windows.h>

#include "profileprovider.h"
#include "ljv7controller.h"

#include <iostream>
#include <limits>

#undef min  ///< Clean up Windows min macro
#undef max  ///< Clean up Windows max macro

QVector<ProfilePoint> ProfileProvider::loadProfile()
{
    QVector<ProfilePoint> result;

    try
    {
        std::cout << "[INFO] Initializing Keyence controller\n";

        LJController lj(L"C:/Users/chist/projects/KeyEnceTests/lib/LJV7_IF.dll");
        lj.Initialize();

        const LONG deviceId = 0;    ///< Default device ID for single-device setups
        lj.UsbOpen(deviceId);

        LJV7IF_GET_PROFILE_REQ req = {};
        req.byTargetBank = LJV7IF_PROFILE_BANK_ACTIVE;  ///< Use active profile bank
        req.byPosMode    = LJV7IF_PROFILE_POS_CURRENT;  ///< Current position mode
        req.byGetProfCnt = 1;                           ///< Request single profile
        req.byErase      = 0;                           ///< Don't erase after reading

        LJV7IF_GET_PROFILE_RSP rsp = {};  ///< Response structure
        LJV7IF_PROFILE_INFO info  = {};   ///< Profile metadata structure

        // Request raw profile data from device
        auto rawBuf = lj.GetProfile(deviceId, req, rsp, info);

        if (rsp.byGetProfCnt == 0)
        {
            std::cerr << "[ERROR] Profile not received\n";
            return {};
        }

        const double to_mm = 1e-5;  ///< Conversion factor: 0.1μm → mm
        const LONG INVALID_RAW = std::numeric_limits<LONG>::min();

        double xStart = info.lXStart * to_mm;
        double xPitch = info.lXPitch * to_mm;

        const uint8_t* base =
            reinterpret_cast<const uint8_t*>(rawBuf.data());
        const LONG* zRaw =
            reinterpret_cast<const LONG*>(base + sizeof(LJV7IF_PROFILE_HEADER));

        for (int i = 0; i < info.wProfDataCnt; ++i)
        {
            ProfilePoint p;
            p.x = xStart + i * xPitch;

            if (zRaw[i] == INVALID_RAW)
            {
                p.z = -21474.8;   ///< Visualization sentinel for invalid points
                p.valid = false;
            }
            else
            {
                p.z = zRaw[i] * to_mm;
                p.valid = true;
            }

            result.append(p);
        }

        std::cout << "[INFO] Total points loaded: " << result.size() << "\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[EXCEPTION] " << e.what() << "\n";
    }

    return result;
}
