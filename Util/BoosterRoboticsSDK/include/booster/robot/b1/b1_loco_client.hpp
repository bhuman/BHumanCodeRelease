#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>

#include "b1_loco_api.hpp"

using namespace booster::robot;

namespace booster {
namespace robot {
namespace b1 {

class B1LocoClient {
public:
    B1LocoClient() = default;
    ~B1LocoClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    /**
     * @brief Send API request to B1 robot
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(LocoApiId api_id, const std::string &param);

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
     *              - current_mode (RobotMode enum value)
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
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t LieDown() {
        return SendApiRequest(LocoApiId::kLieDown, "");
    }

    /**
     * @brief The robot gets up from a position lying on its back
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t GetUp() {
        return SendApiRequest(LocoApiId::kGetUp, "");
    }

    /**
     *  @brief Move hand end-effector to a target posture(position & orientation) with an auxiliary point
     *
     *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach.
     *  It contains position & orientation.
     *  @param aux_posture Represents the auxiliary point on the end-effector's motion arc trajectory
     *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
     *
     *  @return 0 if success, otherwise return error code
     */
    int32_t MoveHandEndEffectorWithAux(const Posture &target_posture, const Posture &aux_posture, int time_millis, HandIndex hand_index) {
        MoveHandEndEffectorParameter move_hand(target_posture, aux_posture, time_millis, hand_index);
        std::string param = move_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveHandEndEffector, param);
    }

    /**
     *  @brief Move hand end-effector with a target posture(position & orientation)
     *  @deprecated **This API is deprecated and will be removed in future versions.**
     *              Please use the new API `MoveHandEndEffectorV2` instead.
     *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach.
     *                        It contains position & orientation,
     *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
     *
     *  @return 0 if success, otherwise return error code
     *
     *  @details
     *  **Reason for deprecation**: This API is deprecated due to an implicit rotational offset (rot) being applied to the target orientation.
     *  The final orientation is calculated as orientation = rot * offset, which contradicts the parameter description of `target_posture`.
     */
    int32_t MoveHandEndEffector(const Posture &target_posture, int time_millis, HandIndex hand_index) {
        MoveHandEndEffectorParameter move_hand(target_posture, time_millis, hand_index, false);
        std::string param = move_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveHandEndEffector, param);
    }

    /**
     *  @brief Move hand end-effector with a target posture(position & orientation)
     *
     *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach.
     *                        It contains position & orientation.
     *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
     *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
     *
     *  @return 0 if success, otherwise return error code
     */
    int32_t MoveHandEndEffectorV2(const Posture &target_posture, int time_millis, HandIndex hand_index) {
        MoveHandEndEffectorParameter move_hand(target_posture, time_millis, hand_index, true);
        std::string param = move_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kMoveHandEndEffector, param);
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
    int32_t ControlDexterousHand(const std::vector<DexterousFingerParameter> &finger_params, HandIndex hand_index) {
        ControlDexterousHandParameter control_dexterous_hand(finger_params, hand_index);
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

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__