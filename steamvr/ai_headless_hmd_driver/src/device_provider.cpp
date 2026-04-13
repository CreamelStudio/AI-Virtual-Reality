#include "device_provider.h"

#include "driverlog.h"

vr::EVRInitError AiTrackingDeviceProvider::Init( vr::IVRDriverContext *pDriverContext )
{
    VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );

    hmd_device_ = std::make_unique< AiTrackingHmdDeviceDriver >();
    if ( !vr::VRServerDriverHost()->TrackedDeviceAdded(
             hmd_device_->GetSerialNumber().c_str(),
             vr::TrackedDeviceClass_HMD,
             hmd_device_.get() ) )
    {
        DriverLog( "Failed to create AI Tracking HMD device." );
        return vr::VRInitError_Driver_Unknown;
    }

    return vr::VRInitError_None;
}

const char *const *AiTrackingDeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void AiTrackingDeviceProvider::RunFrame()
{
    if ( hmd_device_ != nullptr )
    {
        hmd_device_->RunFrame();
    }

    vr::VREvent_t event{};
    while ( vr::VRServerDriverHost()->PollNextEvent( &event, sizeof( vr::VREvent_t ) ) )
    {
        if ( hmd_device_ != nullptr )
        {
            hmd_device_->ProcessEvent( event );
        }
    }
}

bool AiTrackingDeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

void AiTrackingDeviceProvider::EnterStandby()
{
}

void AiTrackingDeviceProvider::LeaveStandby()
{
}

void AiTrackingDeviceProvider::Cleanup()
{
    hmd_device_ = nullptr;
}

