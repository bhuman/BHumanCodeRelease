#pragma once

#include <string>
#include <vector>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {
namespace vision {

// API ID 定义
enum class VisionApiId {
    kStartVisionService = 3000,
    kStopVisionService = 3001,
    kGetDetectionObject = 3002
};

// --------------------------------------------------------
// 1. 启动服务的参数类
// --------------------------------------------------------
class StartVisionServiceParameter {
public:
    StartVisionServiceParameter() = default;
    StartVisionServiceParameter(bool pos, bool color, bool face) :
        enable_position_(pos), enable_color_(color), enable_face_detection_(face) {
    }

    void FromJson(const nlohmann::json &json) {
        if (json.contains("enable_position")) enable_position_ = json["enable_position"];
        if (json.contains("enable_color")) enable_color_ = json["enable_color"];
        if (json.contains("enable_face_detection")) enable_face_detection_ = json["enable_face_detection"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["enable_position"] = enable_position_;
        json["enable_color"] = enable_color_;
        json["enable_face_detection"] = enable_face_detection_;
        return json;
    }

public:
    bool enable_position_ = false;
    bool enable_color_ = false;
    bool enable_face_detection_ = false;
};

class GetDetectionObjectParameter {
public:
    GetDetectionObjectParameter() = default;
    explicit GetDetectionObjectParameter(float ratio) :
        focus_ratio_(ratio) {
    }

    void FromJson(const nlohmann::json &json) {
        if (json.contains("focus_ratio")) focus_ratio_ = json["focus_ratio"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["focus_ratio"] = focus_ratio_;
        return json;
    }

public:
    float focus_ratio_ = 0.33f;
};

class DetectResults {
public:
    DetectResults() = default;

    void FromJson(const nlohmann::json &json) {
        if (json.contains("xmin")) xmin_ = json["xmin"];
        if (json.contains("ymin")) ymin_ = json["ymin"];
        if (json.contains("xmax")) xmax_ = json["xmax"];
        if (json.contains("ymax")) ymax_ = json["ymax"];

        if (json.contains("position")) position_ = json["position"].get<std::vector<float>>();
        if (json.contains("tag")) tag_ = json["tag"];
        if (json.contains("conf")) conf_ = json["conf"];
        if (json.contains("rgb_mean")) rgb_mean_ = json["rgb_mean"].get<std::vector<int32_t>>();
    }

    // 序列化：C++ 对象转为 JSON
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["xmin"] = xmin_;
        json["ymin"] = ymin_;
        json["xmax"] = xmax_;
        json["ymax"] = ymax_;
        json["position"] = position_;
        json["tag"] = tag_;
        json["conf"] = conf_;
        json["rgb_mean"] = rgb_mean_;
        return json;
    }

public:
    int64_t xmin_ = 0;
    int64_t ymin_ = 0;
    int64_t xmax_ = 0;
    int64_t ymax_ = 0;
    std::vector<float> position_;
    std::string tag_;
    float conf_ = 0.0f;
    std::vector<int32_t> rgb_mean_;
};

}
}
} // namespace booster::robot::vision