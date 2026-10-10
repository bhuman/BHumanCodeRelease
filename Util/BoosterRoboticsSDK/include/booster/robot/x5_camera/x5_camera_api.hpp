#ifndef BOOSTER_ROBOTICS_SDK_X5_CAMERA_API_HPP
#define BOOSTER_ROBOTICS_SDK_X5_CAMERA_API_HPP

#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>
#include <booster/robot/x5_camera/x5_camera_api_const.hpp>

namespace booster {
namespace robot {
namespace x5_camera {

enum class X5CameraApiId{
    kChangeMode = 5001,
    kGetStatus = 5002,
};

class ChangeModeParameter {
public:
    ChangeModeParameter() = default;
    ChangeModeParameter(CameraSetMode mode) :
        mode_(mode) {
    }

public:
    void FromJson(nlohmann::json &json) {
        mode_ = static_cast<CameraSetMode>(json["mode"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["mode"] = static_cast<int>(mode_);
        return json;
    }

public:
    CameraSetMode mode_;
};

class GetStatusResponse {
public:
    GetStatusResponse() = default;
    GetStatusResponse(CameraControlStatus status) :
        status_(status) {
    }

public:
    void FromJson(nlohmann::json &json) {
        status_ = static_cast<CameraControlStatus>(json["status"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["status"] = static_cast<int>(status_);
        return json;
    }

public:
    CameraControlStatus status_;
};

}
}
} // namespace booster::robot

#endif
