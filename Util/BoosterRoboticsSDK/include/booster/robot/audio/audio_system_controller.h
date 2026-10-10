#pragma once

#include <cstdint>

namespace booster {
namespace robot {
namespace audio {

class AudioManager;

class AudioSystemController {
public:
    explicit AudioSystemController(AudioManager &audio_manager);
    ~AudioSystemController();

    int32_t SetSystemVolume(float volume);
    int32_t GetSystemVolume(float *volume) const;

    int32_t SetSystemMute(bool mute);
    int32_t GetSystemMute(bool *mute) const;

private:
    AudioManager *audio_manager_{nullptr};
};

} // namespace audio
} // namespace robot
} // namespace booster
