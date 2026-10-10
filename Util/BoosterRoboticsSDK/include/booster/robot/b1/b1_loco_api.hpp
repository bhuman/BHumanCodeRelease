#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__

#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>
#include <booster/robot/common/device_info.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/common/entities.hpp>
#include <booster/robot/common/robot_shared.hpp>

namespace booster {
namespace robot {
namespace b1 {

/* service name */
const std::string LOCO_SERVICE_NAME = "loco";

/*API version*/
const std::string LOCO_API_VERSION = "1.0.0.1";

/*API ID */
enum class LocoApiId {
    kChangeMode = 2000,
    kMove = 2001,
    kRotateHead = 2004,
    kWaveHand = 2005,
    kRotateHeadWithDirection = 2006,
    kLieDown = 2007,
    kGetUp = 2008,
    kMoveHandEndEffector = 2009,
    kControlGripper = 2010,
    kGetFrameTransform = 2011,
    kSwitchHandEndEffectorControlMode = 2012,
    kControlDexterousHand = 2013,
    kHandshake = 2015,
    kDance = 2016,
    kGetMode = 2017,
    kGetStatus = 2018,
    kPushUp = 2019,
    kPlaySound = 2020,
    kStopSound = 2021,
    kGetRobotInfo = 2022,
    kStopHandEndEffector = 2023,
    kShoot = 2024,
    kGetUpWithMode = 2025,
    kZeroTorqueDrag = 2026,
    kRecordTrajectory = 2027,
    kReplayTrajectory = 2028,
    kWholeBodyDance = 2029,
    kUpperBodyCustomControl = 2030,
    kResetOdometry = 2031,
    kLoadCustomTrainedTraj = 2032,
    kActivateCustomTrainedTraj = 2033,
    kUnloadCustomTrainedTraj = 2034,
    kEnterWBCGait = 2035,
    kExitWBCGait = 2036,
    kMoveDualHandEndEffector = 2037,
    kVisualKick = 2038,
    kLionDancePrepare = 2039,
    kLionDanceStart = 2040,
    kLionDanceMove = 2041,
    kSwitchGait = 2042,
    kRotateHeadWithTime = 2043,
    kGetSensors = 2044,
    kGetHands = 2045,
    kGetRobotModel = 2046,
};

enum class GaitType {
    kWholeBodyHumanlikeGait = 0,
    kHalfBodyHumanlikeGait = 1,
    kHalfBodyHumanlikeGaitV2 = 2,
    kWholeBodyHumanlikeGaitV2 = 3,
};

class RotateHeadParameter {
public:
    RotateHeadParameter() = default;
    RotateHeadParameter(float pitch, float yaw) :
        pitch_(pitch),
        yaw_(yaw) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_ = json["pitch"];
        yaw_ = json["yaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["pitch"] = pitch_;
        json["yaw"] = yaw_;
        return json;
    }

public:
    float pitch_;
    float yaw_;
};

class RotateHeadWithTimeParameter {
public:
    RotateHeadWithTimeParameter() = default;

    RotateHeadWithTimeParameter(float pitch, float yaw, int time_millis) :
        pitch_(pitch),
        yaw_(yaw),
        time_millis_(time_millis) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_ = json["pitch"];
        yaw_ = json["yaw"];
        time_millis_ = json["time_millis"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;

        json["pitch"] = pitch_;
        json["yaw"] = yaw_;
        json["time_millis"] = time_millis_;
        return json;
    }

public:
    float pitch_;
    float yaw_;
    int time_millis_;
};

class ChangeModeParameter {
public:
    ChangeModeParameter() = default;
    ChangeModeParameter(booster::robot::RobotMode mode) :
        mode_(mode) {
    }

public:
    void FromJson(nlohmann::json &json) {
        mode_ = static_cast<booster::robot::RobotMode>(json["mode"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["mode"] = static_cast<int>(mode_);
        return json;
    }

public:
    booster::robot::RobotMode mode_;
};

class SwitchGaitParameter {
public:
    SwitchGaitParameter() = default;
    SwitchGaitParameter(GaitType gait_type) :
        gait_type_(gait_type) {
    }

public:
    void FromJson(nlohmann::json &json) {
        gait_type_ = static_cast<GaitType>(json["gait_type"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["gait_type"] = static_cast<int>(gait_type_);
        return json;
    }

public:
    GaitType gait_type_ = GaitType::kWholeBodyHumanlikeGait;
};

class GetModeResponse {
public:
    GetModeResponse() = default;

public:
    void FromJson(nlohmann::json &json) {
        mode_ = static_cast<booster::robot::RobotMode>(json["mode"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["mode"] = static_cast<int>(mode_);
        return json;
    }

public:
    booster::robot::RobotMode mode_;
};

class GetStatusResponse {
public:
    GetStatusResponse() = default;

    void FromJson(nlohmann::json &json) {
        current_mode_ = static_cast<booster::robot::RobotMode>(json["current_mode"]);
        current_body_control_ = static_cast<booster::robot::BodyControl>(json["current_body_control"]);
        std::vector<booster::robot::Action> actions_vec;
        for (const auto &item : json["current_actions"]) {
            actions_vec.push_back(static_cast<booster::robot::Action>(item));
        }
        current_actions_ = std::move(actions_vec);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["current_mode"] = static_cast<int>(current_mode_);
        json["current_body_control"] = static_cast<int>(current_body_control_);
        std::vector<int> actions_int_vec;
        for (const auto &action : current_actions_) {
            actions_int_vec.push_back(static_cast<int>(action));
        }
        json["current_actions"] = actions_int_vec;
        return json;
    }

public:
    booster::robot::RobotMode current_mode_;
    booster::robot::BodyControl current_body_control_;
    std::vector<booster::robot::Action> current_actions_;
};

class GetRobotInfoResponse {
public:
    GetRobotInfoResponse() = default;

    void FromJson(nlohmann::json &json) {
        name_ = json["name"];
        nickname_ = json["nickname"];
        version_ = json["version"];
        model_ = json["model"];
        serial_number_ = json["serial_number"];
        edition_ = json["edition"];
        region_ = json["region"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["name"] = name_;
        json["nickname"] = nickname_;
        json["version"] = version_;
        json["model"] = model_;
        json["serial_number"] = serial_number_;
        json["edition"] = edition_;
        json["region"] = region_;
        return json;
    }

public:
    std::string name_;          // official name for robot
    std::string nickname_;      // nickname for robot, user can set it through mobile app `Booster`
    std::string version_;       // firmware version
    std::string model_;         // model name for robot
    std::string serial_number_; // serial number for robot
    std::string edition_;       // edition for robot, e.g. K1 XXX, T1 XXX
    std::string region_;        // region for robot, e.g. CN, Global
};

class MoveParameter {
public:
    MoveParameter() = default;
    MoveParameter(float vx, float vy, float vyaw) :
        vx_(vx),
        vy_(vy),
        vyaw_(vyaw) {
    }

public:
    void FromJson(nlohmann::json &json) {
        vx_ = json["vx"];
        vy_ = json["vy"];
        vyaw_ = json["vyaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["vx"] = vx_;
        json["vy"] = vy_;
        json["vyaw"] = vyaw_;
        return json;
    }

public:
    float vx_;
    float vy_;
    float vyaw_;
};

class RotateHeadWithDirectionParameter {
public:
    RotateHeadWithDirectionParameter() = default;
    RotateHeadWithDirectionParameter(int pitch_direction, int yaw_direction) :
        pitch_direction_(pitch_direction),
        yaw_direction_(yaw_direction) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_direction_ = json["pitch_direction"];
        yaw_direction_ = json["yaw_direction"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["pitch_direction"] = pitch_direction_;
        json["yaw_direction"] = yaw_direction_;
        return json;
    }

public:
    int pitch_direction_;
    int yaw_direction_;
};

enum class GetUpVersion {
    kV1 = 0, // Initial/Base get-up behavior
    kV2 = 1, // BMM get-up behavior
};

class GetUpParameter {
public:
    GetUpParameter() = default;
    GetUpParameter(GetUpVersion version) :
        version_(version) {
    }

public:
    void FromJson(nlohmann::json &json) {
        if (json.contains("version")) {
            version_ = static_cast<GetUpVersion>(json["version"]);
        } else {
            version_ = GetUpVersion::kV1;
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["version"] = static_cast<int>(version_);
        return json;
    }

public:
    GetUpVersion version_ = GetUpVersion::kV1;
};

class GetUpWithModeParameter {
public:
    GetUpWithModeParameter() = default;
    GetUpWithModeParameter(booster::robot::RobotMode mode, GetUpVersion version = GetUpVersion::kV1) :
        mode_(mode), version_(version) {
    }

public:
    void FromJson(nlohmann::json &json) {
        mode_ = static_cast<booster::robot::RobotMode>(json["mode"]);
        if (json.contains("version")) {
            version_ = static_cast<GetUpVersion>(json["version"]);
        } else {
            version_ = GetUpVersion::kV1;
        }
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["mode"] = static_cast<int>(mode_);
        json["version"] = static_cast<int>(version_);
        return json;
    }

public:
    booster::robot::RobotMode mode_ = booster::robot::RobotMode::kUnknown;
    GetUpVersion version_ = GetUpVersion::kV1;
};

class WaveHandParameter {
public:
    WaveHandParameter() = default;
    WaveHandParameter(HandIndex hand_index, HandAction hand_action) :
        hand_index_(hand_index), hand_action_(hand_action) {
    }

public:
    void FromJson(nlohmann::json &json) {
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
        hand_action_ = static_cast<HandAction>(json["hand_action"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["hand_index"] = static_cast<int>(hand_index_);
        json["hand_action"] = static_cast<int>(hand_action_);
        return json;
    }

public:
    HandIndex hand_index_;
    HandAction hand_action_;
};

class HandshakeParameter {
public:
    HandshakeParameter() = default;
    HandshakeParameter(HandAction hand_action) :
        hand_action_(hand_action) {
    }

public:
    void FromJson(nlohmann::json &json) {
        hand_action_ = static_cast<HandAction>(json["hand_action"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["hand_action"] = static_cast<int>(hand_action_);
        return json;
    }

public:
    HandAction hand_action_;
};

class MoveHandEndEffectorParameter {
public:
    MoveHandEndEffectorParameter() = default;
    MoveHandEndEffectorParameter(
        const Posture &target_posture,
        int time_millis,
        HandIndex hand_index) :
        target_posture_(target_posture),
        time_millis_(time_millis),
        hand_index_(hand_index) {
        has_aux_ = false;
    }
    MoveHandEndEffectorParameter(
        const Posture &target_posture,
        const Posture &aux_posture,
        int time_millis,
        HandIndex hand_index) :
        target_posture_(target_posture),
        aux_posture_(aux_posture),
        time_millis_(time_millis),
        hand_index_(hand_index) {
        has_aux_ = true;
    }
    MoveHandEndEffectorParameter(
        const Posture &target_posture,
        int time_millis,
        HandIndex hand_index,
        bool new_version) :
        target_posture_(target_posture),
        time_millis_(time_millis),
        hand_index_(hand_index),
        new_version_(new_version) {
        has_aux_ = false;
    }

public:
    void FromJson(nlohmann::json &json) {
        target_posture_.FromJson(json["target_posture"]);
        has_aux_ = json["has_aux"];
        if (has_aux_) {
            aux_posture_.FromJson(json["aux_posture"]);
        }
        time_millis_ = json["time_millis"];
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
        new_version_ = json["new_version"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["target_posture"] = target_posture_.ToJson();
        if (has_aux_) {
            json["aux_posture"] = aux_posture_.ToJson();
        }
        json["time_millis"] = time_millis_;
        json["hand_index"] = static_cast<int>(hand_index_);
        json["has_aux"] = has_aux_;
        json["new_version"] = new_version_;
        return json;
    }

public:
    Posture target_posture_;
    Posture aux_posture_;
    int time_millis_ = 1000;
    HandIndex hand_index_;
    bool has_aux_ = false;
    bool new_version_ = false;
};

enum class GripperControlMode {
    // position mode, the gripper will stop when it reaches the target position,
    // or when it experiences a specific reaction force, this reaction force
    // specified in the motion parameter
    kPosition = 0,
    // force mode, if the gripper not reaches the target position, the gripper will
    // continue to move with the force specified in the motion parameter
    kForce = 1
};

/**
 *  This class definition represents a gripper motion parameter. Different grippers
 *  have different motion parameters.
 *
 *  The following parameters all represent a scaling factor. When using them, you
 *  need to convert them into the corresponding coefficients based on the specifications
 *  of the gripper you are using.
 *
 *  For the Inspire EG2-4C2 Gripper:
 *  - Maximum position: 77mm, Therefore, a position value of (0 ~ 1000) corresponds to (0 ~ 77 mm)
 *  - Maximum force: 2kg, Therefore, a force value of (0 ~ 1000) corresponds to (0 ~ 2 kg)
 *  - Speed unit: Not specified, The Inspire EG2-4C2 Gripper does not provide a unit, range,
 *    or dimension for speed. The speed can only be adjusted using values from 0 to 1000, which
 *    do not correspond to an absolute value
 *
 *  position: represents the gripper's opening value, ranging from 0 to 1000
 *  force: represents the gripper's force control value, ranging from 50 to 1000
 *  speed represents the gripper's opening and closing speed ranging from 1 to 1000
 */

class GripperMotionParameter {
public:
    GripperMotionParameter() = default;
    GripperMotionParameter(const int32_t position, const int32_t force, const int32_t speed) :
        position_(position), force_(force), speed_(speed) {
    }

    void FromJson(nlohmann::json &json) {
        position_ = json["position"];
        force_ = json["force"];
        speed_ = json["speed"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["position"] = position_;
        json["force"] = force_;
        json["speed"] = speed_;
        return json;
    }

public:
    int32_t position_ = 0;
    int32_t force_ = 0;
    int32_t speed_ = 0;
};

class ControlGripperParameter {
public:
    ControlGripperParameter() = default;
    ControlGripperParameter(const GripperMotionParameter &motion_param, GripperControlMode mode, HandIndex hand_index) :
        motion_param_(motion_param), mode_(mode), hand_index_(hand_index) {
    }

public:
    void FromJson(nlohmann::json &json) {
        motion_param_.FromJson(json["motion_param"]);
        mode_ = static_cast<GripperControlMode>(json["mode"]);
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["motion_param"] = motion_param_.ToJson();
        json["mode"] = static_cast<int>(mode_);
        json["hand_index"] = static_cast<int>(hand_index_);
        return json;
    }

public:
    GripperMotionParameter motion_param_;
    GripperControlMode mode_;
    HandIndex hand_index_;
};

class MoveDualHandEndEffectorParameter {
public:
    MoveDualHandEndEffectorParameter() = default;
    MoveDualHandEndEffectorParameter(
        const Posture &left_target_posture,
        const Posture &right_target_posture,
        int time_millis) :
        left_target_posture_(left_target_posture),
        right_target_posture_(right_target_posture),
        time_millis_(time_millis) {
    }

public:
    void FromJson(nlohmann::json &json) {
        left_target_posture_.FromJson(json["left_target_posture"]);
        right_target_posture_.FromJson(json["right_target_posture"]);
        time_millis_ = json["time_millis"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["left_target_posture"] = left_target_posture_.ToJson();
        json["right_target_posture"] = right_target_posture_.ToJson();
        json["time_millis"] = time_millis_;
        return json;
    }

public:
    Posture left_target_posture_;
    Posture right_target_posture_;
    int time_millis_ = 1000;
};

class GetFrameTransformParameter {
public:
    GetFrameTransformParameter() = default;
    GetFrameTransformParameter(const Frame &src, const Frame &dst) :
        src_(src), dst_(dst) {
    }

public:
    void FromJson(nlohmann::json &json) {
        src_ = static_cast<Frame>(json["src"]);
        dst_ = static_cast<Frame>(json["dst"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["src"] = static_cast<int>(src_);
        json["dst"] = static_cast<int>(dst_);
        return json;
    }

public:
    Frame src_;
    Frame dst_;
};

class SwitchHandEndEffectorControlModeParameter {
public:
    SwitchHandEndEffectorControlModeParameter() = default;
    SwitchHandEndEffectorControlModeParameter(bool switch_on) :
        switch_on_(switch_on) {
    }

public:
    void FromJson(nlohmann::json &json) {
        switch_on_ = json["switch_on"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["switch_on"] = switch_on_;
        return json;
    }

public:
    bool switch_on_;
};

/**
 * This class definition represents a dexterous finger parameter. Different
 * dexterous hands have different motion parameters.
 *
 * The following parameters all represent a scaling factor. When using them, you
 * need to convert them into the corresponding coefficients based on the
 * specifications of the dexterous hand you are using.
 *
 * For the Inspire RH56 Dexterous Hand:
 * - seq: 0~5, represents the sequence of the fingers, as follows:
 * -- 0.Little finger, 1.Ring finger, 2.Middle finger, 3.Index finger, 4.Thumb bending, 5.Thumb rotation
 * - angle: 0~1000, 0 represents the fully closed state, 1000 represents the
 * fully open state. Different fingers have different ranges.
 * -- The range 0~1000 of the thumb rotation corresponds to 90-165°
 * -- The range 0~1000 of the thumb bending corresponds to -130~53.6°
 * -- The range 0~1000 of the other fingers corresponds to 19°-176.7°
 * - force: represents the finger's force value, Different fingers have
 * different ranges.
 * -- The range 0~1500 of the thumb rotation and bending corresponds to 0~1.5kg
 * -- The range 0~1000 of the other fingers corresponds to 0~1kg
 * - speed: 0~1000, Speed unit: Not specified, The Inspire RH56 dexterous hand
 * does not provide a unit, range, or dimension for speed. The speed can only be
 * adjusted using values from 0 to 1000, which do not correspond to an absolute
 * value
 *
 */

class DexterousFingerParameter {
public:
    DexterousFingerParameter() = default;
    DexterousFingerParameter(const int32_t seq, const int32_t angle,
                             const int32_t force, const int32_t speed) :
        seq_(seq),
        angle_(angle),
        force_(force), speed_(speed) {
    }

    void FromJson(nlohmann::json &json) {
        seq_ = json["seq"];
        angle_ = json["angle"];
        force_ = json["force"];
        speed_ = json["speed"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["seq"] = seq_;
        json["angle"] = angle_;
        json["force"] = force_;
        json["speed"] = speed_;
        return json;
    }

public:
    int32_t seq_ = -1;
    int32_t angle_ = 0;
    int32_t force_ = 0;
    int32_t speed_ = 0;
};

/**
 * This class definition represents a dexterous hand parameter.
 * finger_params: represents the parameters of each finger, It is a vector of `DexterousFingerParameter`
 * hand_index: represents the hand index, which can be `kLeftHand` or `kRightHand`
 *
 */

class ControlDexterousHandParameter {
public:
    ControlDexterousHandParameter() = default;
    ControlDexterousHandParameter(
        const std::vector<DexterousFingerParameter> &finger_params, HandIndex hand_index, BoosterHandType hand_type = BoosterHandType::kInspireHand) :
        finger_params_(finger_params),
        hand_index_(hand_index),
        hand_type_(hand_type) {
    }

    void FromJson(nlohmann::json &json) {
        for (auto &finger_param : json["finger_params"]) {
            DexterousFingerParameter param;
            param.FromJson(finger_param);
            finger_params_.push_back(param);
        }
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
        hand_type_ = static_cast<BoosterHandType>(json["hand_type"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        for (auto &finger_param : finger_params_) {
            json["finger_params"].push_back(finger_param.ToJson());
        }
        json["hand_index"] = static_cast<int>(hand_index_);
        json["hand_type"] = static_cast<int>(hand_type_);
        return json;
    }

public:
    std::vector<DexterousFingerParameter> finger_params_;
    HandIndex hand_index_;
    BoosterHandType hand_type_;
};

enum class DanceId {
    kNewYear = 0,
    kNezha = 1,
    kTowardsFuture = 2,
    kDabbingGesture = 3,
    kUltramanGesture = 4,
    kRespectGesture = 5,
    kCheeringGesture = 6,
    kLuckyCatGesture = 7,
    kStop = 1000,
};

/**
 * This class definition represents a dance parameter.
 * dance_id: represents the dance ID, which can be found in the `DanceId` enum
 */

class DanceParameter {
public:
    DanceParameter() = default;
    DanceParameter(DanceId dance_id) :
        dance_id_(dance_id) {
    }

public:
    void FromJson(nlohmann::json &json) {
        dance_id_ = json["dance_id"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["dance_id"] = dance_id_;
        return json;
    }

public:
    DanceId dance_id_;
};

class PlaySoundParameter {
public:
    PlaySoundParameter() = default;
    PlaySoundParameter(const std::string &sound_file_path) :
        sound_file_path_(sound_file_path) {
    }

    void FromJson(nlohmann::json &json) {
        sound_file_path_ = json["sound_file_path"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["sound_file_path"] = sound_file_path_;
        return json;
    }

private:
    std::string sound_file_path_;
};

class ReplayTrajectoryParameter {
public:
    ReplayTrajectoryParameter() = default;
    ReplayTrajectoryParameter(const std::string &traj_file_path) :
        traj_file_path_(traj_file_path) {
    }

    void FromJson(nlohmann::json &json) {
        traj_file_path_ = json["traj_file_path"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["traj_file_path"] = traj_file_path_;
        return json;
    }

private:
    std::string traj_file_path_;
};

class ZeroTorqueDragParameter {
public:
    ZeroTorqueDragParameter() = default;
    ZeroTorqueDragParameter(bool enable) :
        enable_(enable) {
    }

    void FromJson(nlohmann::json &json) {
        enable_ = json["enable"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["enable"] = enable_;
        return json;
    }

private:
    bool enable_ = false;
};

class RecordTrajectoryParameter {
public:
    RecordTrajectoryParameter() = default;
    RecordTrajectoryParameter(bool enable) :
        enable_(enable) {
    }

    void FromJson(nlohmann::json &json) {
        enable_ = json["enable"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["enable"] = enable_;
        return json;
    }

private:
    bool enable_ = false;
};

enum class WholeBodyDanceId {
    kArbicDance = 0,
    kMichaelDance1 = 1,
    kMichaelDance2 = 2,
    kMichaelDance3 = 3,
    kBoxingStyleKick = 5,
    kRoundhouseKick = 6,
    kShanHeGuRenDance = 7,
    kGaiGeChunFengDance = 8,
    kMichaelDance1And2 = 9,
};

class WholeBodyDanceParameter {
public:
    WholeBodyDanceParameter() = default;
    WholeBodyDanceParameter(WholeBodyDanceId dance_id) :
        dance_id_(dance_id) {
    }

public:
    void FromJson(nlohmann::json &json) {
        dance_id_ = json["dance_id"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["dance_id"] = dance_id_;
        return json;
    }

public:
    WholeBodyDanceId dance_id_;
};

class UpperBodyCustomControlParameter {
public:
    UpperBodyCustomControlParameter() = default;
    UpperBodyCustomControlParameter(bool start) :
        start_(start) {
    }

    void FromJson(nlohmann::json &json) {
        start_ = json["start"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["start"] = start_;
        return json;
    }

public:
    bool start_;
};

// --- Custom Trained Trajectory Definitions ---
enum class JointOrder {
    kMuJoCo = 0,
    kIsaacLab = 1,
};

class CustomModelParams {
public:
    CustomModelParams() = default;

    CustomModelParams(const std::vector<double> &action_scale,
                      const std::vector<double> &kp,
                      const std::vector<double> &kd) :
        action_scale_(action_scale),
        kp_(kp), kd_(kd) {
    }

    void FromJson(nlohmann::json &json) {
        if (json.contains("action_scale")) {
            action_scale_ = json["action_scale"].get<std::vector<double>>();
        }
        if (json.contains("kp")) {
            kp_ = json["kp"].get<std::vector<double>>();
        }
        if (json.contains("kd")) {
            kd_ = json["kd"].get<std::vector<double>>();
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["action_scale"] = action_scale_;
        json["kp"] = kp_;
        json["kd"] = kd_;
        return json;
    }

public:
    std::vector<double> action_scale_;
    std::vector<double> kp_;
    std::vector<double> kd_;
};

class CustomModel {
public:
    CustomModel() = default;
    CustomModel(const std::string &file_path,
                const std::vector<CustomModelParams> &params,
                JointOrder joint_order) :
        file_path_(file_path),
        params_(params),
        joint_order_(joint_order) {
    }

    void FromJson(nlohmann::json &json) {
        file_path_ = json["file_path"];
        params_.clear();
        if (json.contains("params") && json["params"].is_array()) {
            for (auto &item : json["params"]) {
                CustomModelParams p;
                p.FromJson(item);
                params_.push_back(p);
            }
        }
        joint_order_ = static_cast<JointOrder>(json["joint_order"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["file_path"] = file_path_;
        json["params"] = nlohmann::json::array();
        for (const auto &p : params_) {
            json["params"].push_back(p.ToJson());
        }
        json["joint_order"] = static_cast<int>(joint_order_);
        return json;
    }

public:
    std::string file_path_;
    std::vector<CustomModelParams> params_;
    JointOrder joint_order_ = JointOrder::kMuJoCo;
};

class CustomTrainedTraj {
public:
    CustomTrainedTraj() = default;
    CustomTrainedTraj(const std::string &traj_file_path, const CustomModel &model) :
        traj_file_path_(traj_file_path),
        model_(model) {
    }

    void FromJson(nlohmann::json &json) {
        traj_file_path_ = json["traj_file_path"];
        model_.FromJson(json["model"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["traj_file_path"] = traj_file_path_;
        json["model"] = model_.ToJson();
        return json;
    }

public:
    std::string traj_file_path_;
    CustomModel model_;
};

class LoadCustomTrainedTrajResponse {
public:
    LoadCustomTrainedTrajResponse() = default;

    void FromJson(nlohmann::json &json) {
        tid_ = json["tid"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["tid"] = tid_;
        return json;
    }

public:
    std::string tid_;
};

class CustomTrainedTrajParameter {
public:
    CustomTrainedTrajParameter() = default;
    CustomTrainedTrajParameter(const std::string &tid) :
        tid_(tid) {
    }

    void FromJson(nlohmann::json &json) {
        tid_ = json["tid"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["tid"] = tid_;
        return json;
    }

public:
    std::string tid_;
};

enum class VisualKickVersion {
    kV1 = 0, // Initial/Base version
    kV2 = 1, // V2 version, stronger kicking force
};

class VisualKickParameter {
public:
    VisualKickParameter() = default;
    VisualKickParameter(bool start, VisualKickVersion version = VisualKickVersion::kV1) :
        start_(start), version_(version) {
    }

public:
    void FromJson(nlohmann::json &json) {
        start_ = json["start"];
        version_ = static_cast<VisualKickVersion>(json["version"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["start"] = start_;
        json["version"] = static_cast<int>(version_);
        return json;
    }

private:
    bool start_;
    VisualKickVersion version_;
};

class LionDancePrepareParameter {
public:
    LionDancePrepareParameter() = default;
    LionDancePrepareParameter(bool start) :
        start_(start) {
    }

    void FromJson(nlohmann::json &json) {
        start_ = json["start"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["start"] = start_;
        return json;
    }

private:
    bool start_;
};

class LionDanceMoveParameter {
public:
    LionDanceMoveParameter() = default;
    LionDanceMoveParameter(bool start) :
        start_(start) {
    }
    void FromJson(nlohmann::json &json) {
        start_ = json["start"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["start"] = start_;
        return json;
    }

private:
    bool start_;
};

class LionDanceStartParameter {
public:
    LionDanceStartParameter() = default;
    LionDanceStartParameter(int dance_idx) :
        dance_idx_(dance_idx) {
    }

    void FromJson(nlohmann::json &json) {
        dance_idx_ = json["dance_idx"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["dance_idx"] = dance_idx_;
        return json;
    }

private:
    int dance_idx_;
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__
