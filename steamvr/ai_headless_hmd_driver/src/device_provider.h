#pragma once

#include <memory>

#include "hmd_device_driver.h"
#include "openvr_driver.h"

class AiTrackingDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
    vr::EVRInitError Init( vr::IVRDriverContext *pDriverContext ) override;
    const char *const *GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;
    void Cleanup() override;

private:
    std::unique_ptr< AiTrackingHmdDeviceDriver > hmd_device_;
};

