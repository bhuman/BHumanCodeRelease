#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <booster/robot/audio/audio_types.h>

namespace booster {
namespace robot {
namespace audio {

class AudioManager;

class AudioPlayer {
public:
    explicit AudioPlayer(AudioManager &audio_manager);
    ~AudioPlayer();

    AudioPlayer(const AudioPlayer &) = delete;
    AudioPlayer &operator=(const AudioPlayer &) = delete;

    int32_t Init(const PlayerInitOptions &options);
    int32_t Start();
    int32_t Pause();
    int32_t Stop();
    int32_t Reset();
    int32_t Destroy();
    int32_t Release();

    int32_t SetVolume(float volume);
    int32_t SetPreferredDevice(const std::string &device_id);
    int32_t GetInfo(PlayerInfo *out) const;
    int32_t PushPcmStream(const uint8_t *data, size_t size);

    int64_t GetSessionId() const;
    PlayerState GetCachedState() const;

    void SetStateCallback(PlayerStateCallback callback);
    void SetProgressCallback(PlayerProgressCallback callback);
    void SetCompletionCallback(PlayerCompletionCallback callback);
    void SetErrorCallback(PlayerErrorCallback callback);

private:
    struct StateData;

    void EnsureTopicListenersRegistered();

    static PlayerState ParsePlayerState(int32_t raw_state);

    AudioManager *audio_manager_{nullptr};
    int64_t progress_listener_id_{0};
    int64_t error_listener_id_{0};
    std::shared_ptr<StateData> state_;
};

} // namespace audio
} // namespace robot
} // namespace booster
