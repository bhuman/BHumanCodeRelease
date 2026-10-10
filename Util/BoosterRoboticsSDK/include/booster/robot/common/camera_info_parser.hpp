#pragma once

/**
 * Parse Camera RPC JSON bodies into booster_interface::msg::Camera / CameraList.
 */

#include <booster/idl/camera/Camera.h>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

void CameraFromRpcJson(booster_interface::msg::Camera &camera,
                      const nlohmann::json &j);

/** Accepts a JSON array or `{ "cameras": [ ... ] }` (GetCameras RPC body). */
void CameraListFromRpcJson(booster_interface::msg::CameraList &list,
                           const nlohmann::json &json);

} // namespace robot
} // namespace booster
