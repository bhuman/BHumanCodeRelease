#pragma once

#include <string>

#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {
namespace camera {

// Aligned with Camera.idl RPC numbering (separate band from Vision 3000+).
enum class CameraApiId : int64_t {
    kGetCameras = 3100,
};

} // namespace camera
} // namespace robot
} // namespace booster
