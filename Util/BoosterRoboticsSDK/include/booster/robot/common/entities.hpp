#pragma once
#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

/**
 *  This class definition represents a 3D position in space with x, y, and z coordinates.
 *  x, y, and z are in meters.
 */
class Position {
public:
    Position() = default;
    Position(float x, float y, float z) :
        x_(x), y_(y), z_(z) {
    }

    void FromJson(nlohmann::json &json) {
        x_ = json["x"];
        y_ = json["y"];
        z_ = json["z"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["x"] = x_;
        json["y"] = y_;
        json["z"] = z_;
        return json;
    }

public:
    float x_ = 0.; // unit : m
    float y_ = 0.; // unit : m
    float z_ = 0.; // unit : m
};

/**
 *  This class definition represents a 3D orientation in space with roll, pitch, and yaw coordinates.
 *  roll, pitch, and yaw are in rad.
 */
class Orientation {
public:
    Orientation() = default;
    Orientation(float roll, float pitch, float yaw) :
        roll_(roll), pitch_(pitch), yaw_(yaw) {
    }

    void FromJson(nlohmann::json &json) {
        roll_ = json["roll"];
        pitch_ = json["pitch"];
        yaw_ = json["yaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["roll"] = roll_;
        json["pitch"] = pitch_;
        json["yaw"] = yaw_;
        return json;
    }

public:
    float roll_ = 0.;  // unit : rad
    float pitch_ = 0.; // unit : rad
    float yaw_ = 0.;   // unit : rad
};

/**
 *  This class definition represents a 3D pose in space with position and orientation.
 *  position and orientation are in meters and rad.
 */
class Posture {
public:
    Posture() = default;
    Posture(const Position &pos, const Orientation &orient) :
        position_(pos), orientation_(orient) {
    }

    void FromJson(nlohmann::json &json) {
        position_.FromJson(json["position"]);
        orientation_.FromJson(json["orientation"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["position"] = position_.ToJson();
        json["orientation"] = orientation_.ToJson();
        return json;
    }

public:
    Position position_;
    Orientation orientation_;
};

/**
 *  This class definition represents a 4D quaternion.
 *  quaternion is in x, y, z, w format.
 */
class Quaternion {
public:
    Quaternion() = default;
    Quaternion(float x, float y, float z, float w) :
        x_(x), y_(y), z_(z), w_(w) {}
    
    void FromJson(nlohmann::json &json) {
        x_ = json["x"];
        y_ = json["y"];
        z_ = json["z"];
        w_ = json["w"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["x"] = x_;
        json["y"] = y_;
        json["z"] = z_;
        json["w"] = w_;
        return json;
    }

public:
    float x_ = 0.;
    float y_ = 0.;
    float z_ = 0.;
    float w_ = 0.;
};

/**
 *  This class definition represents a 3D transform in space with position and orientation.
 */
class Transform {
public:
    Transform() = default;
    Transform(const Position &position, const Quaternion &orientation) :
        position_(position), orientation_(orientation) {
    }

    void FromJson(nlohmann::json &json) {
        position_.FromJson(json["position"]);
        orientation_.FromJson(json["orientation"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["position"] = position_.ToJson();
        json["orientation"] = orientation_.ToJson();
        return json;
    }

public:
    Position position_;
    Quaternion orientation_;

};

}
} // namespace booster::robot