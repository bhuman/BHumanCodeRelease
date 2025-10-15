#ifndef BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP
#define BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP

#include <string>

namespace booster {
namespace robot {
namespace b1 {

static const std::string kTopicJointCtrl = "rt/joint_ctrl";
static const std::string kTopicLowState = "rt/low_state";
static const std::string kTopicFallDown = "rt/fall_down";
static const std::string kTopicOdometerState = "rt/odometer_state";
static const std::string kTopicBoosterHandData = "rt/booster_hand_data";
static const std::string kTopicTF = "rt/tf";

// TODO(@wuyuanye): 按照结构图，把电机的索引完善
enum class JointIndex {
    // head
    kHeadYaw = 0,
    kHeadPitch = 1,

    // Left arm
    kLeftShoulderPitch = 2,
    kLeftShoulderRoll = 3,
    kLeftElbowPitch = 4,
    kLeftElbowYaw = 5,

    // Right arm
    kRightShoulderPitch = 6,
    kRightShoulderRoll = 7,
    kRightElbowPitch = 8,
    kRightElbowYaw = 9,

    // waist
    kWaist = 10,

    // left leg
    kLeftHipPitch = 11,
    kLeftHipRoll = 12,
    kLeftHipYaw = 13,
    kLeftKneePitch = 14,
    kCrankUpLeft = 15,
    kCrankDownLeft = 16,

    // right leg
    kRightHipPitch = 17,
    kRightHipRoll = 18,
    kRightHipYaw = 19,
    kRightKneePitch = 20,
    kCrankUpRight = 21,
    kCrankDownRight = 22,
};

enum class JointIndexWith7DofArm {
    // head
    kHeadYaw = 0,
    kHeadPitch = 1,

    // Left arm
    kLeftShoulderPitch = 2,
    kLeftShoulderRoll = 3,
    kLeftElbowPitch = 4,
    kLeftElbowYaw = 5,
    kLeftWristPitch = 6,
    kLeftWristYaw = 7,
    kLeftHandRoll = 8,

    // Right arm
    kRightShoulderPitch = 9,
    kRightShoulderRoll = 10,
    kRightElbowPitch = 11,
    kRightElbowYaw = 12,
    kRightWristPitch = 13,
    kRightWristYaw = 14,
    kRightHandRoll = 15,

    // waist
    kWaist = 16,

    // left leg
    kLeftHipPitch = 17,
    kLeftHipRoll = 18,
    kLeftHipYaw = 19,
    kLeftKneePitch = 20,
    kCrankUpLeft = 21,
    kCrankDownLeft = 22,

    // right leg
    kRightHipPitch = 23,
    kRightHipRoll = 24,
    kRightHipYaw = 25,
    kRightKneePitch = 26,
    kCrankUpRight = 27,
    kCrankDownRight = 28
};

static const size_t kJointCnt = 23;
static const size_t kJointCnt7DofArm = 29;

enum HandIndex {
    kLeftHand = 0,
    kRightHand = 1,
};

enum HandAction {
    kHandOpen = 0,
    kHandClose = 1,
};

enum RemoteControllerEvent {
    NONE = 0, // no event
    AXIS = 0x600, // axis motion
    HAT = 0x602, // hat position change
    BUTTON_DOWN = 0x603, // button pressed
    BUTTON_UP = 0x604, // button released
    REMOVE = 0x606 // device has been removed
};

}
}
} // namespace booster::robot::b1

#endif // BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP
