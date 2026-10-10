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
    kSoccer = 4,   // Soccer mode, in soccer mode, the robot can perform some soccer actions
};

enum class BodyControl {
    kUnknown = 0,
    kDamping = 1,               // Only active in damping mode, damping mode has no other body control
    kPrepare = 2,               // Only active in prepare mode, prepare mode has no other body control
    kHumanlikeGait = 3,         // Humanlike gait body control, in humanlike gait, the robot can walk like a human
    kProneBody = 4,             // Prone body control, in prone body control, the robot can lie down、push-up
    kSoccerGait = 5,            // Soccer gait body control, in soccer gait, the robot can move faster and do some soccer player like actions
    kCustom = 6,                // Only active in custom mode, custom mode has no other body control
    kGetUp = 7,                 // Get up body control, in get up body control, the robot can get up from lying down
    kWholeBodyDance = 8,        // Whole body dance body control, in whole body dance, the robot can do some whole body dance actions
    kShoot = 9,                 // Shoot body control, in shoot body control, the robot can scored with a powerful shot
    kInsideFoot = 10,           // History caused name, actually is visual kick body control V2
    kGoalie = 11,               // Goalie body control, in goalie body control, the robot can perform goalie actions
    kWBCGait = 12,              // Whole body control gait body control, in WBC gait, the robot walking by whole body control
    kLionDancePreparePose = 13, // Lion dance prepare pose, in this body control, the robot will do lion dance prepare pose
    kVisualKickV1 = 14,         // Visual kick body control V1, in visual kick body control, the robot can do visual kick actions
};

enum class Action {
    kUnknown = 0,
    kHandShake = 1,
    kHandWave = 2,
    kHandControl = 3,
    kDanceNewYear = 4,
    kDanceNezha = 5,
    kDanceTowardsFuture = 6,
    kGestureDabbing = 7,
    kGestureUltraman = 8,
    kGestureRespect = 9,
    kGestureCheer = 10,
    kGestureLuckyCat = 11,
    kGestureBoxing = 12,
    kZeroTorqueDrag = 13,
    kRecordTraj = 14,
    kRunRecordedTraj = 15,
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

enum class ProneBodyControlPosture {
    kUnknown = 0,
    kInactive = 1,
    kPushUp = 2,
    kLieDown = 3,
    kSoccerLocomotion = 4,
    kSoccerKicking = 5,
};

enum class RobocupBehaviorStatus {
    RUNNING = 0,
    SHOOTING = 1,
    PASSING = 2,
};

}
} // namespace booster::robot
