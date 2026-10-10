#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <booster/robot/audio/audio_types.h>

namespace booster {
namespace robot {
namespace audio {

class AudioManager;

class AudioCaptureStream {
public:
    explicit AudioCaptureStream(AudioManager &audio_manager);
    ~AudioCaptureStream();

    AudioCaptureStream(const AudioCaptureStream &) = delete;
    AudioCaptureStream &operator=(const AudioCaptureStream &) = delete;

    int32_t Init(const AudioCaptureStreamOptions &options);
    int32_t Start();
    int32_t Pause();
    int32_t Stop();
    int32_t Destroy();

    int32_t GetInfo(AudioCaptureStreamInfo *out) const;
    int32_t SetPreferredDevice(const std::string &device_id);

    int64_t GetSessionId() const;
    AudioCaptureStreamState GetCachedState() const;

    void SetFrameCallback(AudioCaptureFrameCallback callback);
    void SetStateCallback(AudioCaptureStreamStateCallback callback);
    void SetErrorCallback(AudioCaptureStreamErrorCallback callback);

private:
    struct StateData;

    void EnsureErrorListenerRegistered();

    static AudioCaptureStreamState ParseCaptureStreamState(int32_t raw_state);

    AudioManager *audio_manager_{nullptr};
    int64_t error_listener_id_{0};
    std::shared_ptr<StateData> state_;
};

} // namespace audio
} // namespace robot
} // namespace booster
