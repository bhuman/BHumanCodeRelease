#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <booster/robot/audio/audio_types.h>

namespace booster {
namespace robot {
namespace audio {

class AudioManager;

class AudioRecorder {
public:
    explicit AudioRecorder(AudioManager &audio_manager);
    ~AudioRecorder();

    AudioRecorder(const AudioRecorder &) = delete;
    AudioRecorder &operator=(const AudioRecorder &) = delete;

    int32_t Init(const RecorderInitOptions &options);
    int32_t Start();
    int32_t Pause();
    int32_t Stop();
    int32_t Destroy();
    int32_t Release();

    int32_t GetInfo(RecorderInfo *out) const;
    int32_t SetPreferredDevice(const std::string &device_id);

    int64_t GetSessionId() const;
    RecorderState GetCachedState() const;

    void SetStateCallback(RecorderStateCallback callback);
    void SetProgressCallback(RecorderProgressCallback callback);
    void SetErrorCallback(RecorderErrorCallback callback);

private:
    struct StateData;

    void EnsureTopicListenersRegistered();
    int32_t WaitForCachedState(
        RecorderState expected_state,
        int attempts = 10,
        int64_t poll_interval_ms = 200) const;
    static RecorderState ParseRecorderState(int32_t raw_state);

    AudioManager *audio_manager_{nullptr};
    int64_t progress_listener_id_{0};
    int64_t error_listener_id_{0};
    std::shared_ptr<StateData> state_;
};

} // namespace audio
} // namespace robot
} // namespace booster
