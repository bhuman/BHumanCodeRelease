#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>
#include "b1_loco_api.hpp"

namespace booster {
namespace robot {
namespace b1 {

class B1LocoClient {
public:
    B1LocoClient() = default;
    ~B1LocoClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    bool WaitForService(
        int64_t timeout_ms = booster::robot::RpcClient::kDefaultWaitForServiceTimeoutMs,
        bool require_response_path = true);
    /**
     * @brief Send API request to B1 robot
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(LocoApiId api_id, const std::string &param);
    int32_t SendApiRequest(LocoApiId api_id, const std::string &param, int64_t timeout_ms);
    int32_t SendApiRequestFireAndForget(LocoApiId api_id, const std::string &param);

    /**
     * @brief Send API request to B1 robot with response
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     * @param resp [out] A reference to a Response object where the API's response will be stored.
     * This parameter is modified by the function to contain the result of the API call
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequestWithResponse(LocoApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief Change robot mode
     *
     * @param mode robot mode, options are: kDamping, kPrepare, kWalking
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ChangeMode(RobotMode mode) {
        ChangeModeParameter change_mode(mode);
        std::string param = change_mode.ToJson().dump();
        return SendApiRequest(LocoApiId::kChangeMode, param);
    }

    /**
     * @brief Get current robot mode
     *
     * @param[out] get_mode_response Reference to store the response data, including:
     * - current_mode (RobotMode enum value)
     *
     * @return 0 if success, otherwise return error code
     * @see ChangeMode() for mode switching API
     * @see RobotMode enum for available mode definitions
     */
    int32_t GetMode(GetModeResponse &get_mode_response) {
        std::string param{};
        Response resp;
        int32_t ret = SendApiRequestWithResponse(LocoApiId::kGetMode,
                                                 param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        get_mode_response.FromJson(body_json);
        return ret;
    }

    /**
     * @brief Get current robot status
     *
     * @param[out] get_status_response Reference to store the response data, including:
     * - current_mode (RobotMode enum value)
     * - current_body_control (BodyControl enum value)
     * - current_actions (vector of Action enum values)
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t GetStatus(GetStatusResponse &get_status_response) {
        std::string param{};
        Response resp;
        int32_t ret = SendApiRequestWithResponse(LocoApiId::kGetStatus,
                                                 param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        get_status_response.FromJson(body_json);
        return ret;
    }

    /**
     * @brief Get robot info
     *
     * @param[out] get_robot_info_response Reference to store the response data, including:
     * - name (std::string)
     * - nickname (std::string)
     * - version (std::string)
     * - model (std::string)
     * - serial_number (std::string)
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t GetRobotInfo(GetRobotInfoResponse &get_robot_info_response) {
        std::string param{};
        Response resp;
        int32_t ret = SendApiRequestWithResponse(LocoApiId::kGetRobotInfo,
                                                 param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        get_robot_info_response.FromJson(body_json);
        return ret;
    }

    /**
     * @brief Query IMU devices (static catalog from robot_config). RPC JSON uses field "imus".
     * @param[out] info kind_ is kSensors; json_ holds RPC body. Use booster::robot::ImuInfoListFromDeviceInfo(list, info)
     * (see device_info_parser.hpp).
     * @return 0 if success, otherwise error code
     */
    int32_t GetSensors(DeviceInfo &info) {
        std::string param{};
        Response resp;
        int32_t ret =
            SendApiRequestWithResponse(LocoApiId::kGetSensors, param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        info.FromJson(body_json);
        info.kind_ = DeviceInfoKind::kSensors;
        return ret;
    }

    /**
     * @brief Query hand end-effector catalog from robot_config (static). RPC JSON uses field
     * "hands"; use booster::robot::HandListFromDeviceInfo(list, info) (see device_info_parser.hpp).
     *
     * @param[out] info kind_ is kHands; json_ holds RPC body (field "hands").
     * @return 0 if success, otherwise error code
     */
    int32_t GetHands(DeviceInfo &info) {
        std::string param{};
        Response resp;
        int32_t ret =
            SendApiRequestWithResponse(LocoApiId::kGetHands, param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        info.FromJson(body_json);
        info.kind_ = DeviceInfoKind::kHands;
        return ret;
    }

    /**
     * @brief Query static robot model (URDF-derived via RBDL on the motion stack).
     * RPC JSON nests model under "robot_model". Use booster::robot::RobotModelFromDeviceInfo(model, info)
     * (see device_info_parser.hpp).
     *
     * @param[out] info kind_ is kRobotModel; json_ holds RPC body (key "robot_model").
     * @return 0 if success, otherwise error code
     */
    int32_t GetRobotModel(DeviceInfo &info) {
        std::string param{};
        Response resp;
        int32_t ret =
            SendApiRequestWithResponse(LocoApiId::kGetRobotModel, param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        info.FromJson(body_json);
        info.kind_ = DeviceInfoKind::kRobotModel;
        return ret;
    }

    /**
     * @brief Move robot
     *
     * @param vx linear velocity in x direction, unit: m/s
     * @param vy linear velocity in y direction, unit: m/s
     * @param vyaw angular velocity, unit: rad/s
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t Move(float vx, float vy, float vyaw) {
        MoveParameter move(vx, vy, vyaw);
        std::string param = move.ToJson().dump();
        return SendApiRequest(LocoApiId::kMove, param);
    }

    int32_t MoveCommand(float vx, float vy, float vyaw) {
        MoveParameter move(vx, vy, vyaw);
        std::string param = move.ToJson().dump();
        return SendApiRequestFireAndForget(LocoApiId::kMove, param);
    }

    /**
     * @brief Robot rotates its head
     *
     * @param pitch pitch angle, unit: rad
     * @param yaw yaw angle, unit: rad
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RotateHead(float pitch, float yaw) {
        RotateHeadParameter head_ctrl(pitch, yaw);
        std::string param = head_ctrl.ToJson().dump();
        return SendApiRequest(LocoApiId::kRotateHead, param);
    }

    /**
     * @brief Robot rotates its head to target position within specific time
     *
     * @param pitch target pitch angle, unit: rad
     * @param yaw target yaw angle, unit: rad
     * @param time_millis duration to reach the target, unit: ms
     *
     * @return 0 if success, otherwise return error code
     */

    int32_t RotateHeadWithTime(float pitch, float yaw, int time_millis) {
        RotateHeadWithTimeParameter head_ctrl(pitch, yaw, time_millis);
        std::string param = head_ctrl.ToJson().dump();
        return SendApiRequest(LocoApiId::kRotateHeadWithTime, param);
    }

    /**
     * @brief Robot waves its hand
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t WaveHand(HandAction action) {
        WaveHandParameter wave_hand(kRightHand, action);
        std::string param = wave_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kWaveHand, param);
    }

    /**
     * @brief The robot rotates its head at an appropriate speed,
     * and the head does not rotate beyond its limit
     *
     * @param pitch_direction pitch direction, options are: 1(left), 0(no rotation), -1(right)
     * @param yaw_direction yaw direction, options are: 1(down), 0(no rotation), -1(up)
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RotateHeadWithDirection(int pitch_direction, int yaw_direction) {
        RotateHeadWithDirectionParameter head_ctrl(pitch_direction, yaw_direction);
        std::string param = head_ctrl.ToJson().dump();
        return SendApiRequest(LocoApiId::kRotateHeadWithDirection, param);
    }

    /**
     * @brief The robot lies down on its back
     * @warning This API is unstable and may change in future releases.
     * @return 0 if success, otherwise return error code
     */
    int32_t LieDown() {
        return SendApiRequest(LocoApiId::kLieDown, "");
    }

    /**
     * @brief The robot gets up
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t GetUp(GetUpVersion version = GetUpVersion::kV1) {
        GetUpParameter param(version);
        std::string json = param.ToJson().dump();
        return SendApiRequest(LocoApiId::kGetUp, json);
    }

    /**
     * @brief The robot gets up to specified mode, either kWalking or kSoccer
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t GetUpWithMode(booster::robot::RobotMode mode, GetUpVersion version = GetUpVersion::kV1) {
        GetUpWithModeParameter param(mode, version);
        std::string json = param.ToJson().dump();
        return SendApiRequest(LocoApiId::kGetUpWithMode, json);
    }

    /**
     * @brief The robot executes a powerful kicking motion.
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t Shoot() {
        return SendApiRequest(LocoApiId::kShoot, "");
    }

    /**
     * @brief Move hand end-effector to a target posture(position & orientation) with an auxiliary point
     *
     * @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach.
     * It contains position & orientation.
     * @param aux_posture Represents the auxiliary point on the end-effector's motion arc trajectory
     * @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     * @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t MoveHandEndEffectorWithAux(const Posture &target_posture, const Posture &aux_posture, int time_millis, HandIndex hand_index) {
        MoveHandEndEffectorParameter move_hand(target_posture, aux_posture, time_millis, hand_index);
        std::string param = move_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveHandEndEffector, param);
    }

    /**
     * @brief Move hand end-effector with a target posture(position & orientation)
     * @deprecated **This API is deprecated and will be removed in future versions.**
     * Please use the new API `MoveHandEndEffectorV2` instead.
     * @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach.
     * It contains position & orientation,
     * @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     * @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
     *
     * @return 0 if success, otherwise return error code
     *
     * @details
     * **Reason for deprecation**: This API is deprecated due to an implicit rotational offset (rot) being applied to the target orientation.
     * The final orientation is calculated as orientation = rot * offset, which contradicts the parameter description of `target_posture`.
     */
    int32_t MoveHandEndEffector(const Posture &target_posture, int time_millis, HandIndex hand_index) {
        MoveHandEndEffectorParameter move_hand(target_posture, time_millis, hand_index, false);
        std::string param = move_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveHandEndEffector, param);
    }

    /**
     * @brief Move hand end-effector with a target posture(position & orientation)
     *
     * @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach.
     * It contains position & orientation.
     * @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     * @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t MoveHandEndEffectorV2(const Posture &target_posture, int time_millis, HandIndex hand_index) {
        MoveHandEndEffectorParameter move_hand(target_posture, time_millis, hand_index, true);
        std::string param = move_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveHandEndEffector, param);
    }

    /**
     * @brief Move dual hand end-effector to target postures(position & orientation)
     *
     * @param left_target_posture Represents the target posture in base frame (torso frame) that the left hand end-effector should reach.
     * It contains position & orientation.
     * @param right_target_posture Represents the target posture in base frame (torso frame) that the right hand end-effector should reach.
     * It contains position & orientation.
     * @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t MoveDualHandEndEffector(
        const Posture &left_target_posture,
        const Posture &right_target_posture,
        int time_millis) {
        MoveDualHandEndEffectorParameter move_dual_hand(
            left_target_posture,
            right_target_posture,
            time_millis);
        std::string param = move_dual_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveDualHandEndEffector, param);
    }

    /**
     * @brief Stop hand end-effector movement
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StopHandEndEffector() {
        return SendApiRequest(LocoApiId::kStopHandEndEffector, "");
    }

    /**
     * @brief Control gripper
     *
     * @param motion_param motion parameter, include position, force, velocity, see `GripperMotionParameter`
     * @param mode gripper control mode, options are: kPosition, kForce, see `GripperControlMode`
     * @param hand_index hand index, options are: kLeftHand, kRightHand
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ControlGripper(const GripperMotionParameter &motion_param, GripperControlMode mode, HandIndex hand_index) {
        ControlGripperParameter control_gripper(motion_param, mode, hand_index);
        std::string param = control_gripper.ToJson().dump();
        return SendApiRequest(LocoApiId::kControlGripper, param);
    }

    /**
     * @brief Get frame transform
     *
     * @param src source frame
     * @param dst destination frame
     * @param transform [out] calculated transform
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t GetFrameTransform(Frame src, Frame dst, Transform &transform) {
        GetFrameTransformParameter frame_transform(src, dst);
        std::string param = frame_transform.ToJson().dump();
        Response resp;
        int32_t ret = SendApiRequestWithResponse(LocoApiId::kGetFrameTransform, param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        transform.FromJson(body_json);
        return 0;
    }

    /**
     * @brief Switch hand end-effector control mode
     *
     * @param switch_on true to switch on, false to switch off
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SwitchHandEndEffectorControlMode(bool switch_on) {
        SwitchHandEndEffectorControlModeParameter switch_param(switch_on);
        std::string param = switch_param.ToJson().dump();
        return SendApiRequest(LocoApiId::kSwitchHandEndEffectorControlMode, param);
    }

    /**
     * @brief Handshake
     *
     * @param action whether to start handshake action, options are: kHandOpen, kHandClose
     *
     */
    int32_t Handshake(HandAction action) {
        HandshakeParameter handshake(action);
        std::string param = handshake.ToJson().dump();
        return SendApiRequest(LocoApiId::kHandshake, param);
    }

    /**
     * @brief Control dexterous hand
     *
     * @param finger_params finger parameters, include position, force, speed, see `DexterousFingerParameter`
     * @param hand_index hand index, options are: kLeftHand, kRightHand
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ControlDexterousHand(const std::vector<DexterousFingerParameter> &finger_params, HandIndex hand_index, BoosterHandType hand_type = BoosterHandType::kInspireHand) {
        ControlDexterousHandParameter control_dexterous_hand(finger_params, hand_index, hand_type);
        std::string param = control_dexterous_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kControlDexterousHand, param);
    }

    /**
     * Note: an unstable interface
     * @brief Make the robot perform a dance.
     *
     * @param dance_id The identifier of the dance to be performed
     *
     * @return int32_t Returns 0 if successful, otherwise returns an error code
     */
    int32_t Dance(DanceId dance_id) {
        DanceParameter dance(dance_id);
        std::string param = dance.ToJson().dump();
        return SendApiRequest(LocoApiId::kDance, param);
    }

    /**
     * @brief Play sound in specific path
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t PlaySound(const std::string &sound_file_path) {
        PlaySoundParameter play_sound(sound_file_path);
        std::string param = play_sound.ToJson().dump();
        return SendApiRequest(LocoApiId::kPlaySound, param);
    }

    /**
     * @brief Stop sound
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StopSound() {
        return SendApiRequest(LocoApiId::kStopSound, "");
    }

    /**
     * @brief Enable or disable zero torque drag, depending on active state
     *
     * @param active true to enable, false to disable
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ZeroTorqueDrag(bool active) {
        ZeroTorqueDragParameter zero_torque_drag(active);
        std::string param = zero_torque_drag.ToJson().dump();
        return SendApiRequest(LocoApiId::kZeroTorqueDrag, param);
    }

    /**
     * @brief Start or stop recording trajectory, depending on active state
     *
     * @param active true to start recording, false to stop
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RecordTrajectory(bool active) {
        RecordTrajectoryParameter record_trajectory(active);
        std::string param = record_trajectory.ToJson().dump();
        return SendApiRequest(LocoApiId::kRecordTrajectory, param);
    }

    /**
     * @brief Replay trajectory
     *
     * @param active true to start replaying, false to stop
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ReplayTrajectory(std::string path) {
        ReplayTrajectoryParameter replay_trajectory(path);
        std::string param = replay_trajectory.ToJson().dump();
        return SendApiRequest(LocoApiId::kReplayTrajectory, param);
    }

    /**
     * @brief Make the robot perform a whole body dance.
     *
     * @param dance_id The identifier of the whole body dance to be performed
     *
     * @return int32_t Returns 0 if successful, otherwise returns an error code
     */
    int32_t WholeBodyDance(WholeBodyDanceId dance_id) {
        WholeBodyDanceParameter dance_param(dance_id);
        std::string param = dance_param.ToJson().dump();
        return SendApiRequest(LocoApiId::kWholeBodyDance, param);
    }

    /**
     * @brief Make the UpperBodyCustomControl action start or stop.
     *
     * @param start true to start the action, false to stop it
     *
     * @return int32_t Returns 0 if successful, otherwise returns an error code
     */
    int32_t UpperBodyCustomControl(bool start) {
        UpperBodyCustomControlParameter upper_body_param(start);
        std::string param = upper_body_param.ToJson().dump();
        return SendApiRequest(LocoApiId::kUpperBodyCustomControl, param);
    }

    /**
     * @brief Reset the robot's odometry
     * @return 0 if success, otherwise return error code
     */
    int32_t ResetOdometry() {
        return SendApiRequest(LocoApiId::kResetOdometry, "");
    }

    /**
     * @brief Load a custom lab model with configuration parameters.
     * Only model_name, model_file_path, and traj_file_path are required.
     *
     * @param traj The custom trained trajectory configuration
     * @param[out] tid The trajectory ID returned by the robot
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t LoadCustomTrainedTraj(const CustomTrainedTraj &traj, std::string &tid) {
        std::string param = traj.ToJson().dump();
        Response resp;
        int32_t ret = SendApiRequestWithResponse(LocoApiId::kLoadCustomTrainedTraj, param, resp);
        if (ret != 0) {
            return ret;
        }

        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        LoadCustomTrainedTrajResponse load_resp;
        load_resp.FromJson(body_json);
        tid = load_resp.tid_;

        return 0;
    }

    /**
     * @brief Activate a loaded custom trained trajectory
     *
     * @param tid The trajectory ID to activate
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ActivateCustomTrainedTraj(const std::string &tid) {
        CustomTrainedTrajParameter param_obj(tid);
        std::string param = param_obj.ToJson().dump();
        return SendApiRequest(LocoApiId::kActivateCustomTrainedTraj, param);
    }

    /**
     * @brief Unload a custom trained trajectory
     *
     * @param tid The trajectory ID to unload
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t UnloadCustomTrainedTraj(const std::string &tid) {
        CustomTrainedTrajParameter param_obj(tid);
        std::string param = param_obj.ToJson().dump();
        return SendApiRequest(LocoApiId::kUnloadCustomTrainedTraj, param);
    }

    /**
     * @brief Enter wbc gait
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t EnterWBCGait() {
        return SendApiRequest(LocoApiId::kEnterWBCGait, "");
    }

    /**
     * @brief Exit WBC Gait
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ExitWBCGait() {
        return SendApiRequest(LocoApiId::kExitWBCGait, "");
    }

    /**
     * @brief side-foot kick
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t VisualKick(bool start, VisualKickVersion version = VisualKickVersion::kV1) {
        VisualKickParameter parameter(start, version);
        std::string param = parameter.ToJson().dump();
        return SendApiRequest(LocoApiId::kVisualKick, param);
    }

    /**
     * @brief Lion dance prepare
     *
     * @details State transition rules:
     * - Allowed entry from: Walking mode.
     * - Allowed transition to: LionDanceMove or LionDanceStart.
     *
     * @param start True to enter Lion Dance Move gesture, false to exit.
     * @return 0 if success, otherwise return error code
     */
    int32_t LionDancePrepare(bool start) {
        LionDancePrepareParameter param(start);
        std::string json = param.ToJson().dump();
        return SendApiRequest(LocoApiId::kLionDancePrepare, json);
    }

    /**
     * @brief Lion dance start
     *
     * @details State transition rules:
     * - Allowed entry from: LionDancePrepare or LionDanceMove ONLY.
     *
     * @warning
     * STRICTLY PROHIBITED to enter from any other state. Doing so will
     * cause abrupt joint jumping (discontinuity) in the arm actuators,
     * which poses a safety risk and may damage the hardware.
     *
     * @param dance_id The specific ID of the lion dance.
     * @return 0 if success, otherwise return error code
     */
    int32_t LionDanceStart(int dance_id) {
        LionDanceStartParameter param(dance_id);
        std::string json = param.ToJson().dump();
        return SendApiRequest(LocoApiId::kLionDanceStart, json);
    }

    /**
     * @brief Lion dance move
     *
     * @details State transition rules:
     * - Allowed entry from: LionDancePrepare and Walk Gait.
     * - Allowed transition to: LionDanceStart.
     *
     * @param start True to start movement synchronization, false to stop.
     * @return 0 if success, otherwise return error code
     */
    int32_t LionDanceMove(bool start) {
        LionDanceMoveParameter param{start};
        std::string json = param.ToJson().dump();
        return SendApiRequest(LocoApiId::kLionDanceMove, json);
    }

    /**
     * @brief Switch robot gait type
     *
     * @param gait_type gait type, options are:
     * kWholeBodyHumanlikeGait, kHalfBodyHumanlikeGait, kHalfBodyHumanlikeGaitV2
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SwitchGait(GaitType gait_type) {
        SwitchGaitParameter param(gait_type);
        std::string json = param.ToJson().dump();
        return SendApiRequest(LocoApiId::kSwitchGait, json);
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__
