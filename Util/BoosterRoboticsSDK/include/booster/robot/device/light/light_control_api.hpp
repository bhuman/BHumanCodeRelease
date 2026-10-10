#pragma once

#include <string>
#include <vector>

#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {
namespace light {

enum class LightApiId {
    kSetLEDLightColor = 2000,
    kStopLEDLightControl = 2001,
    kSetLEDLightColors = 2002
};

class SetLEDLightColorParameter {
public:
    SetLEDLightColorParameter() = default;
    SetLEDLightColorParameter(uint8_t r, uint8_t g, uint8_t b) :
        r_(r), g_(g), b_(b) {
    }
    SetLEDLightColorParameter(const std::string &color) {
        bool success = false;

        if (color.size() == 7 && color[0] == '#') {
            try {
                int r_val = std::stoi(color.substr(1, 2), nullptr, 16);
                int g_val = std::stoi(color.substr(3, 2), nullptr, 16);
                int b_val = std::stoi(color.substr(5, 2), nullptr, 16);

                r_ = static_cast<uint8_t>(r_val);
                g_ = static_cast<uint8_t>(g_val);
                b_ = static_cast<uint8_t>(b_val);
                success = true;
            } catch (const std::exception &e) {
                success = false;
            }
        }
    }

    void FromJson(nlohmann::json &json) {
        r_ = json["r"];
        g_ = json["g"];
        b_ = json["b"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["r"] = r_;
        json["g"] = g_;
        json["b"] = b_;
        return json;
    }

public:
    uint8_t r_ = 0;
    uint8_t g_ = 0;
    uint8_t b_ = 0;
};

class SetLEDLightColorsParameter {
public:
    SetLEDLightColorsParameter() = default;
    explicit SetLEDLightColorsParameter(const std::vector<SetLEDLightColorParameter> &colors) :
        colors_(colors) {
    }

    void FromJson(nlohmann::json &json) {
        colors_.clear();
        for (const auto &color_json : json["colors"]) {
            auto item_json = color_json;
            SetLEDLightColorParameter color;
            color.FromJson(item_json);
            colors_.push_back(color);
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["colors"] = nlohmann::json::array();
        for (const auto &color : colors_) {
            json["colors"].push_back(color.ToJson());
        }
        return json;
    }

public:
    std::vector<SetLEDLightColorParameter> colors_;
};

}
}
} // namespace booster::robot::light