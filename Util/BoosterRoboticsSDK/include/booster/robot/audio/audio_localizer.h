#pragma once

#include <cstdint>

namespace booster {
namespace robot {
namespace audio {

class AudioManager;

class AudioLocalizer {
public:
    explicit AudioLocalizer(AudioManager &audio_manager);
    ~AudioLocalizer();

    int32_t GetDoaAngle(int *angle_deg) const;

private:
    AudioManager *audio_manager_{nullptr};
};

} // namespace audio
} // namespace robot
} // namespace booster
