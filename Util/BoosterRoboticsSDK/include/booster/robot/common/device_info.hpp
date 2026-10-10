#pragma once

#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {
namespace b1 {

// Unified JSON-backed device / catalog info (IMU sensors, hands, robot model, and
// future kinds e.g. cameras). kind_ is set by B1LocoClient; wire RPC body lives in json_ only.
// ToJson() wraps as { "kind": int, "body": json_ }; FromJson accepts that or a bare RPC body.

enum class DeviceInfoKind {
    kUnknown = -1, // default / not yet classified; bare RPC body in FromJson has no kind field
    kSensors = 0,
    kHands = 1,
    kRobotModel = 2,
    kCamera = 3, // reserved for future device-info RPCs
};

class DeviceInfo {
public:
    DeviceInfo() = default;

    void FromJson(nlohmann::json &json) {
        if (json.contains("kind") && json.contains("body")) {
            kind_ = static_cast<DeviceInfoKind>(json.at("kind").get<int>());
            json_ = json.at("body");
        } else {
            kind_ = DeviceInfoKind::kUnknown;
            json_ = json;
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json out;
        out["kind"] = static_cast<int>(kind_);
        out["body"] = json_;
        return out;
    }

public:
    DeviceInfoKind kind_ = DeviceInfoKind::kUnknown;
    nlohmann::json json_;
};

} // namespace b1
} // namespace robot
} // namespace booster
