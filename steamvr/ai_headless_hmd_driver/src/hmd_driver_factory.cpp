#include "device_provider.h"
#include "openvr_driver.h"

#include <cstring>

#if defined( _WIN32 )
#define AI_HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#else
#define AI_HMD_DLL_EXPORT extern "C"
#endif

AiTrackingDeviceProvider g_device_provider;

AI_HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
    if ( 0 == std::strcmp( vr::IServerTrackedDeviceProvider_Version, pInterfaceName ) )
    {
        return &g_device_provider;
    }

    if ( pReturnCode )
    {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }

    return nullptr;
}

