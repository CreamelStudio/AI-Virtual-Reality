#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "openvr_driver.h"

struct AiTrackingDisplayConfiguration
{
    int32_t window_x;
    int32_t window_y;
    int32_t window_width;
    int32_t window_height;
    int32_t render_width;
    int32_t render_height;
    float display_frequency;
    float vsync_to_photons;
};

struct AiTrackingNetworkConfiguration
{
    uint16_t udp_port;
    int32_t stale_pose_milliseconds;
};

struct UdpPoseState
{
    float px = 0.0f;
    float py = 1.65f;
    float pz = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    bool connected = false;
    std::chrono::steady_clock::time_point last_packet = std::chrono::steady_clock::now();
};

class AiTrackingDisplayComponent : public vr::IVRDisplayComponent
{
public:
    explicit AiTrackingDisplayComponent( const AiTrackingDisplayConfiguration &config );

    bool IsDisplayOnDesktop() override;
    bool IsDisplayRealDisplay() override;
    void GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight ) override;
    void GetEyeOutputViewport(
        vr::EVREye eEye,
        uint32_t *pnX,
        uint32_t *pnY,
        uint32_t *pnWidth,
        uint32_t *pnHeight ) override;
    void GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom ) override;
    vr::DistortionCoordinates_t ComputeDistortion( vr::EVREye eEye, float fU, float fV ) override;
    void GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight ) override;
    bool ComputeInverseDistortion(
        vr::HmdVector2_t *pResult,
        vr::EVREye eEye,
        uint32_t unChannel,
        float fU,
        float fV ) override;

private:
    AiTrackingDisplayConfiguration config_;
};

class AiTrackingHmdDeviceDriver : public vr::ITrackedDeviceServerDriver
{
public:
    AiTrackingHmdDeviceDriver();
    ~AiTrackingHmdDeviceDriver() override;

    vr::EVRInitError Activate( uint32_t unObjectId ) override;
    void EnterStandby() override;
    void *GetComponent( const char *pchComponentNameAndVersion ) override;
    void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize ) override;
    vr::DriverPose_t GetPose() override;
    void Deactivate() override;

    const std::string &GetSerialNumber() const;
    void RunFrame();
    void ProcessEvent( const vr::VREvent_t &event );

private:
    void StartThreads();
    void StopThreads();
    void PoseUpdateThread();
    void UdpReceiveThread();
    void LoadSettings();
    UdpPoseState SnapshotPoseState() const;

    std::unique_ptr< AiTrackingDisplayComponent > display_component_;

    std::string model_number_;
    std::string serial_number_;
    AiTrackingDisplayConfiguration display_configuration_{};
    AiTrackingNetworkConfiguration network_configuration_{};

    std::atomic< bool > is_active_{ false };
    std::atomic< uint32_t > device_index_{ vr::k_unTrackedDeviceIndexInvalid };
    std::thread pose_update_thread_;
    std::thread udp_receive_thread_;

    mutable std::mutex pose_mutex_;
    UdpPoseState pose_state_{};
};

