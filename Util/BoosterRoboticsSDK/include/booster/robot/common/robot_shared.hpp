#pragma once

namespace booster {
namespace robot {

/*Robot mode */
enum class RobotMode {
    kUnknown = -1, // For error handling
    kDamping = 0,  // All motor enter damping mode, robot will fall down if it is not supported
    kPrepare = 1,  // Prepare mode, the robot keeps standing on both feet and can switch to walking mode
    kWalking = 2,  // Walking mode, in walking mode, the robot can move, rotate, kick the ball, etc.
    kCustom = 3,   // Custom mode, in custom mode, the robot can do some custom actions
};

enum class Frame {
    kUnknown = -1, // For error handling
    kBody = 0,     
    kHead = 1,
    kLeftHand = 2,
    kRightHand = 3,
    kLeftFoot = 4,
    kRightFoot = 5,
};

}
} // namespace booster::robot
