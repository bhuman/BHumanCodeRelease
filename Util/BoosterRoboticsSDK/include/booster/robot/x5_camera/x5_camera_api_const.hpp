#ifndef BOOSTER_ROBOTICS_SDK_X5_CAMERA_API_CONST_HPP
#define BOOSTER_ROBOTICS_SDK_X5_CAMERA_API_CONST_HPP

#include <string>

namespace booster {
namespace robot {
namespace x5_camera {

static const std::string kTopicX5CameraControlMode = "rt/X5CameraControl";

enum class CameraSetMode {
    kCameraModeNormal = 0,
    kCameraModeHighResolution = 1,
    kCameraModeNormalEnable = 2,
    kCameraModeHighResolutionEnable = 3,
};

enum class CameraControlStatus {
    kCameraStatusNormal = 0,
    kCameraStatusHighResolution = 1,
    kCameraStatusError = 2,
    kCameraStatusNull = 3,
};

}
}
} // namespace booster::robot

#endif
