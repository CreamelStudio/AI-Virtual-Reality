#include "hmd_device_driver.h"

#include "driverlog.h"

#include <WinSock2.h>
#include <WS2tcpip.h>
#include <cstdio>
#include <cstring>

#pragma comment( lib, "Ws2_32.lib" )

static const char *k_main_settings_section = "driver_aitrackinghmd";
static const char *k_display_settings_section = "aitrackinghmd_display";
static const char *k_network_settings_section = "aitrackinghmd_network";

AiTrackingDisplayComponent::AiTrackingDisplayComponent( const AiTrackingDisplayConfiguration &config )
    : config_( config )
{
}

bool AiTrackingDisplayComponent::IsDisplayOnDesktop()
{
    return true;
}

bool AiTrackingDisplayComponent::IsDisplayRealDisplay()
{
    return false;
}

void AiTrackingDisplayComponent::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
    *pnWidth = static_cast< uint32_t >( config_.render_width );
    *pnHeight = static_cast< uint32_t >( config_.render_height );
}

void AiTrackingDisplayComponent::GetEyeOutputViewport(
    vr::EVREye eEye,
    uint32_t *pnX,
    uint32_t *pnY,
    uint32_t *pnWidth,
    uint32_t *pnHeight )
{
    *pnY = 0;
    *pnWidth = static_cast< uint32_t >( config_.window_width / 2 );
    *pnHeight = static_cast< uint32_t >( config_.window_height );
    *pnX = eEye == vr::Eye_Left ? 0 : static_cast< uint32_t >( config_.window_width / 2 );
}

void AiTrackingDisplayComponent::GetProjectionRaw(
    vr::EVREye eEye,
    float *pfLeft,
    float *pfRight,
    float *pfTop,
    float *pfBottom )
{
    *pfLeft = -1.0f;
    *pfRight = 1.0f;
    *pfTop = -1.0f;
    *pfBottom = 1.0f;
}

vr::DistortionCoordinates_t AiTrackingDisplayComponent::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
    vr::DistortionCoordinates_t coordinates{};
    coordinates.rfBlue[ 0 ] = fU;
    coordinates.rfBlue[ 1 ] = fV;
    coordinates.rfGreen[ 0 ] = fU;
    coordinates.rfGreen[ 1 ] = fV;
    coordinates.rfRed[ 0 ] = fU;
    coordinates.rfRed[ 1 ] = fV;
    return coordinates;
}

void AiTrackingDisplayComponent::GetWindowBounds(
    int32_t *pnX,
    int32_t *pnY,
    uint32_t *pnWidth,
    uint32_t *pnHeight )
{
    *pnX = config_.window_x;
    *pnY = config_.window_y;
    *pnWidth = static_cast< uint32_t >( config_.window_width );
    *pnHeight = static_cast< uint32_t >( config_.window_height );
}

bool AiTrackingDisplayComponent::ComputeInverseDistortion(
    vr::HmdVector2_t *pResult,
    vr::EVREye eEye,
    uint32_t unChannel,
    float fU,
    float fV )
{
    return false;
}

AiTrackingHmdDeviceDriver::AiTrackingHmdDeviceDriver()
{
    LoadSettings();
    display_component_ = std::make_unique< AiTrackingDisplayComponent >( display_configuration_ );
}

AiTrackingHmdDeviceDriver::~AiTrackingHmdDeviceDriver()
{
    StopThreads();
}

void AiTrackingHmdDeviceDriver::LoadSettings()
{
    char model_buffer[ 1024 ];
    vr::VRSettings()->GetString( k_main_settings_section, "model_number", model_buffer, sizeof( model_buffer ) );
    model_number_ = model_buffer;

    char serial_buffer[ 1024 ];
    vr::VRSettings()->GetString( k_main_settings_section, "serial_number", serial_buffer, sizeof( serial_buffer ) );
    serial_number_ = serial_buffer;

    display_configuration_.window_x =
        vr::VRSettings()->GetInt32( k_display_settings_section, "window_x" );
    display_configuration_.window_y =
        vr::VRSettings()->GetInt32( k_display_settings_section, "window_y" );
    display_configuration_.window_width =
        vr::VRSettings()->GetInt32( k_display_settings_section, "window_width" );
    display_configuration_.window_height =
        vr::VRSettings()->GetInt32( k_display_settings_section, "window_height" );
    display_configuration_.render_width =
        vr::VRSettings()->GetInt32( k_display_settings_section, "render_width" );
    display_configuration_.render_height =
        vr::VRSettings()->GetInt32( k_display_settings_section, "render_height" );
    display_configuration_.display_frequency =
        vr::VRSettings()->GetFloat( k_display_settings_section, "display_frequency" );
    display_configuration_.vsync_to_photons =
        vr::VRSettings()->GetFloat( k_display_settings_section, "vsync_to_photons" );

    network_configuration_.udp_port = static_cast< uint16_t >(
        vr::VRSettings()->GetInt32( k_network_settings_section, "udp_port" ) );
    network_configuration_.stale_pose_milliseconds =
        vr::VRSettings()->GetInt32( k_network_settings_section, "stale_pose_milliseconds" );

    DriverLog( "AI Tracking HMD Model Number: %s", model_number_.c_str() );
    DriverLog( "AI Tracking HMD Serial Number: %s", serial_number_.c_str() );
    DriverLog( "AI Tracking HMD UDP Port: %u", network_configuration_.udp_port );
}

vr::EVRInitError AiTrackingHmdDeviceDriver::Activate( uint32_t unObjectId )
{
    device_index_ = unObjectId;
    is_active_ = true;

    vr::PropertyContainerHandle_t container =
        vr::VRProperties()->TrackedDeviceToPropertyContainer( device_index_ );

    vr::VRProperties()->SetStringProperty( container, vr::Prop_ModelNumber_String, model_number_.c_str() );
    vr::VRProperties()->SetStringProperty( container, vr::Prop_SerialNumber_String, serial_number_.c_str() );
    vr::VRProperties()->SetStringProperty(
        container,
        vr::Prop_InputProfilePath_String,
        "{aitrackinghmd}/input/aitrackinghmd_profile.json" );

    const float ipd = vr::VRSettings()->GetFloat( vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float );
    vr::VRProperties()->SetFloatProperty( container, vr::Prop_UserIpdMeters_Float, ipd );
    vr::VRProperties()->SetFloatProperty( container, vr::Prop_DisplayFrequency_Float, display_configuration_.display_frequency );
    vr::VRProperties()->SetFloatProperty(
        container,
        vr::Prop_SecondsFromVsyncToPhotons_Float,
        display_configuration_.vsync_to_photons );
    vr::VRProperties()->SetFloatProperty( container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f );
    vr::VRProperties()->SetBoolProperty( container, vr::Prop_IsOnDesktop_Bool, true );
    vr::VRProperties()->SetBoolProperty( container, vr::Prop_DisplayDebugMode_Bool, true );
    vr::VRProperties()->SetBoolProperty( container, vr::Prop_DeviceProvidesBatteryStatus_Bool, false );
    vr::VRProperties()->SetBoolProperty( container, vr::Prop_DeviceCanPowerOff_Bool, false );

    StartThreads();
    return vr::VRInitError_None;
}

void AiTrackingHmdDeviceDriver::EnterStandby()
{
    DriverLog( "AI Tracking HMD entering standby." );
}

void *AiTrackingHmdDeviceDriver::GetComponent( const char *pchComponentNameAndVersion )
{
    if ( std::strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) == 0 )
    {
        return display_component_.get();
    }

    return nullptr;
}

void AiTrackingHmdDeviceDriver::DebugRequest(
    const char *pchRequest,
    char *pchResponseBuffer,
    uint32_t unResponseBufferSize )
{
    if ( unResponseBufferSize >= 1 )
    {
        pchResponseBuffer[ 0 ] = 0;
    }
}

UdpPoseState AiTrackingHmdDeviceDriver::SnapshotPoseState() const
{
    std::lock_guard< std::mutex > lock( pose_mutex_ );
    return pose_state_;
}

vr::DriverPose_t AiTrackingHmdDeviceDriver::GetPose()
{
    vr::DriverPose_t pose = { 0 };
    pose.qWorldFromDriverRotation.w = 1.0f;
    pose.qDriverFromHeadRotation.w = 1.0f;
    pose.qRotation.w = 1.0f;
    pose.result = vr::TrackingResult_Running_OK;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.shouldApplyHeadModel = true;

    UdpPoseState state = SnapshotPoseState();
    const auto age_ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::steady_clock::now() - state.last_packet );

    if ( !state.connected || age_ms.count() > network_configuration_.stale_pose_milliseconds )
    {
        pose.vecPosition[ 0 ] = 0.0f;
        pose.vecPosition[ 1 ] = 1.65f;
        pose.vecPosition[ 2 ] = 0.0f;
        pose.qRotation.x = 0.0f;
        pose.qRotation.y = 0.0f;
        pose.qRotation.z = 0.0f;
        pose.qRotation.w = 1.0f;
        return pose;
    }

    pose.vecPosition[ 0 ] = state.px;
    pose.vecPosition[ 1 ] = state.py;
    pose.vecPosition[ 2 ] = state.pz;
    pose.qRotation.x = state.qx;
    pose.qRotation.y = state.qy;
    pose.qRotation.z = state.qz;
    pose.qRotation.w = state.qw;
    return pose;
}

void AiTrackingHmdDeviceDriver::Deactivate()
{
    StopThreads();
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

const std::string &AiTrackingHmdDeviceDriver::GetSerialNumber() const
{
    return serial_number_;
}

void AiTrackingHmdDeviceDriver::RunFrame()
{
}

void AiTrackingHmdDeviceDriver::ProcessEvent( const vr::VREvent_t &event )
{
}

void AiTrackingHmdDeviceDriver::StartThreads()
{
    if ( !is_active_ )
    {
        return;
    }

    pose_update_thread_ = std::thread( &AiTrackingHmdDeviceDriver::PoseUpdateThread, this );
    udp_receive_thread_ = std::thread( &AiTrackingHmdDeviceDriver::UdpReceiveThread, this );
}

void AiTrackingHmdDeviceDriver::StopThreads()
{
    if ( is_active_.exchange( false ) )
    {
        if ( udp_receive_thread_.joinable() )
        {
            udp_receive_thread_.join();
        }
        if ( pose_update_thread_.joinable() )
        {
            pose_update_thread_.join();
        }
    }
}

void AiTrackingHmdDeviceDriver::PoseUpdateThread()
{
    while ( is_active_ )
    {
        const uint32_t device_index = device_index_.load();
        if ( device_index != vr::k_unTrackedDeviceIndexInvalid )
        {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
                device_index,
                GetPose(),
                sizeof( vr::DriverPose_t ) );
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
    }
}

void AiTrackingHmdDeviceDriver::UdpReceiveThread()
{
    WSADATA wsa_data{};
    if ( WSAStartup( MAKEWORD( 2, 2 ), &wsa_data ) != 0 )
    {
        DriverLog( "WSAStartup failed for AI Tracking HMD driver." );
        return;
    }

    SOCKET socket_handle = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
    if ( socket_handle == INVALID_SOCKET )
    {
        DriverLog( "Failed to create UDP socket for AI Tracking HMD driver." );
        WSACleanup();
        return;
    }

    DWORD timeout_ms = 500;
    setsockopt(
        socket_handle,
        SOL_SOCKET,
        SO_RCVTIMEO,
        reinterpret_cast< const char * >( &timeout_ms ),
        sizeof( timeout_ms ) );

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl( INADDR_ANY );
    address.sin_port = htons( network_configuration_.udp_port );

    if ( bind( socket_handle, reinterpret_cast< sockaddr * >( &address ), sizeof( address ) ) == SOCKET_ERROR )
    {
        DriverLog( "Failed to bind UDP socket for AI Tracking HMD driver on port %u.", network_configuration_.udp_port );
        closesocket( socket_handle );
        WSACleanup();
        return;
    }

    char buffer[ 256 ];
    while ( is_active_ )
    {
        const int received = recv( socket_handle, buffer, sizeof( buffer ) - 1, 0 );
        if ( received <= 0 )
        {
            continue;
        }

        buffer[ received ] = '\0';
        UdpPoseState next_pose{};
        int connected = 0;
        const int parsed = std::sscanf(
            buffer,
            "%f %f %f %f %f %f %f %d",
            &next_pose.px,
            &next_pose.py,
            &next_pose.pz,
            &next_pose.qx,
            &next_pose.qy,
            &next_pose.qz,
            &next_pose.qw,
            &connected );
        if ( parsed < 8 )
        {
            continue;
        }

        next_pose.connected = connected != 0;
        next_pose.last_packet = std::chrono::steady_clock::now();
        {
            std::lock_guard< std::mutex > lock( pose_mutex_ );
            pose_state_ = next_pose;
        }
    }

    closesocket( socket_handle );
    WSACleanup();
}

