#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace booster {
namespace robot {
namespace audio {

inline constexpr int32_t kAudioErrNotSupported = 7001;

enum class AudioSourceType : int32_t {
    kPcmFile = 0,
    kWavFile = 1,
    kPcmStream = 2,
    kMp3File = 3,
};

enum class PlayerState : int32_t {
    kIdle = 0,
    kReady = 1,
    kPlaying = 2,
    kPaused = 3,
    kStopped = 4,
    kCompleted = 5,
    kError = 6,
};

enum class PlayerPriority : int32_t {
    kLow = 0,
    kMedium = 1,
    kHigh = 2,
};

enum class RecorderState : int32_t {
    kIdle = 0,
    kReady = 1,
    kRecording = 2,
    kPaused = 3,
    kStopped = 4,
    kError = 5,
};

enum class AudioCaptureStreamState : int32_t {
    kIdle = 0,
    kReady = 1,
    kStreaming = 2,
    kPaused = 3,
    kStopped = 4,
    kError = 5,
};

enum class AudioDeviceQueryType : int32_t {
    GET_DEVICES_INPUTS = 0,
    GET_DEVICES_OUTPUTS = 1,
};

enum class AudioDeviceDirection : int32_t {
    INPUT = 0,
    OUTPUT = 1,
};

enum class AudioDeviceTransport : int32_t {
    BUILTIN = 0,
    USB = 1,
    BLUETOOTH = 2,
    VIRTUAL = 3,
    UNKNOWN = 4,
};

enum class AudioDeviceBackendAffinity : int32_t {
    PULSE = 0,
    BOOSTERAEC_ARRAY = 1,
    ALSA_DIAGNOSTIC = 2,
};

struct PcmFormat {
    int32_t sample_rate_hz{16000};
    int32_t channels{1};
    int32_t bits_per_sample{16};
};

struct PlayerInitOptions {
    AudioSourceType source_type{AudioSourceType::kPcmFile};
    std::string source_uri;
    int32_t sample_rate_hz{16000};
    int32_t channels{1};
    int32_t bits_per_sample{16};
    PlayerPriority priority{PlayerPriority::kMedium};
};

struct RecorderInitOptions {
    std::string output_path;
    int32_t sample_rate_hz{16000};
    int32_t channels{1};
    int32_t bits_per_sample{16};
};

struct AudioCaptureStreamOptions {
    bool enable_raw_pcm{true};
    bool enable_naec_pcm{false};
    PcmFormat requested_raw_format{16000, 1, 16};
};

struct PlayerInfo {
    PlayerState state{PlayerState::kIdle};
    int64_t played_bytes{0};
    int64_t total_bytes{0};
    float volume{1.0F};
};

struct RecorderInfo {
    RecorderState state{RecorderState::kIdle};
    int64_t captured_bytes{0};
};

struct AudioCaptureFrame {
    int64_t frame_seq{0};
    int64_t timestamp_ms{0};

    bool raw_valid{false};
    PcmFormat raw_format;
    int32_t raw_frame_samples_per_channel{0};
    std::vector<int16_t> raw_pcm;

    bool naec_valid{false};
    PcmFormat naec_format;
    int32_t naec_frame_samples_per_channel{0};
    std::vector<int16_t> naec_pcm;
};

struct AudioCaptureStreamInfo {
    AudioCaptureStreamState state{AudioCaptureStreamState::kIdle};
    bool raw_enabled{false};
    bool naec_enabled{false};
    PcmFormat actual_raw_format;
    PcmFormat actual_naec_format;
    int64_t published_frames{0};
    int64_t dropped_frames{0};
};

struct AudioDeviceInfo {
    std::string device_id;
    std::string display_name;
    AudioDeviceDirection direction{AudioDeviceDirection::INPUT};
    AudioDeviceTransport transport{AudioDeviceTransport::UNKNOWN};
    AudioDeviceBackendAffinity backend_affinity{AudioDeviceBackendAffinity::PULSE};
    bool is_available{false};
    bool is_system_default{false};
    bool supports_input{false};
    bool supports_output{false};
    std::string provider_name;
    std::string native_id;
    std::string metadata_json;
};

struct AudioError {
    int32_t ret_code{0};
    std::string ret_msg;
    int32_t error_category{0};
    int32_t error_detail{0};
};

enum class BluetoothScanState : int32_t {
    kIdle = 0,
    kScanning = 1,
};

enum class BluetoothDeviceState : int32_t {
    kUnknown = 0,
    kPaired = 1,
    kConnecting = 2,
    kConnected = 3,
    kDisconnecting = 4,
    kFailed = 5,
};

enum class BluetoothMajorClass : int32_t {
    kAudio = 0,
    kPeripheral = 1,
    kPhone = 2,
    kComputer = 3,
    kOther = 4,
};

enum class BluetoothAudioProfile : int32_t {
    kNone = 0,
    kA2dpSink = 1,
    kA2dpSource = 2,
    kHfpHeadset = 3,
    kHfpAg = 4,
};

struct BluetoothDeviceInfo {
    std::string address;
    std::string name;
    int16_t rssi{0};
    BluetoothDeviceState state{BluetoothDeviceState::kUnknown};
    BluetoothMajorClass major_class{BluetoothMajorClass::kOther};
    bool paired{false};
    bool trusted{false};
    bool connected{false};
    bool is_audio_sink{false};
    bool is_audio_source{false};
    bool is_hfp_capable{false};
    std::vector<BluetoothAudioProfile> connected_profiles;
    std::string linked_pulse_sink_id;
    std::string linked_pulse_source_id;
    int64_t last_seen_ms{0};
};

struct BluetoothScanOptions {
    int32_t timeout_ms{30000};
    bool audio_only{true};
};

struct BluetoothConnectOptions {
    std::string address;
    bool auto_pair{true};
    bool make_default{true};
    BluetoothAudioProfile preferred_profile{BluetoothAudioProfile::kNone};
    int32_t timeout_ms{15000};
};

struct BluetoothConnectResult {
    BluetoothDeviceInfo device;
    std::string pulse_sink_id;
    std::string pulse_source_id;
    BluetoothAudioProfile active_profile{BluetoothAudioProfile::kNone};
    bool pulse_endpoint_ready{false};
    bool default_sink_applied{false};
    bool default_source_applied{false};
    int32_t default_route_error_code{0};
    std::string default_route_error_msg;
};

using PlayerStateCallback = std::function<void(PlayerState)>;
using PlayerProgressCallback = std::function<void(const PlayerInfo&)>;
using PlayerCompletionCallback = std::function<void(const PlayerInfo&)>;
using PlayerErrorCallback = std::function<void(const AudioError&)>;

using RecorderStateCallback = std::function<void(RecorderState)>;
using RecorderProgressCallback = std::function<void(const RecorderInfo&)>;
using RecorderErrorCallback = std::function<void(const AudioError&)>;

using AudioCaptureFrameCallback = std::function<void(const AudioCaptureFrame&)>;
using AudioCaptureStreamStateCallback = std::function<void(AudioCaptureStreamState)>;
using AudioCaptureStreamErrorCallback = std::function<void(const AudioError&)>;

} // namespace audio
} // namespace robot
} // namespace booster
