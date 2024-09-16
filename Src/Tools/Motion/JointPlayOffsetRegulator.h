/**
 * @file JointPlayOffsetRegulator.h
 * This file implements a regulator for the offsets in the requested and measured positions of the supporting foot.
 *
 * The motors tend to execute the changes of the requests really good (unless the joint is stuck because
 * different forces like the weight of the robot), but the actually requested position not that good.
 *
 * For example assume the AnklePitch is at position -30deg and the request it at -25deg,
 * but the joint is currently stuck and can not move positive. If the request changes to -20deg (more positive),
 * then the measured position will not change. BUT if the request changes to -30deg (more negative),
 * then the measured position will approximately change by the requested change of -5deg. Meaning
 * the measured position will be -35deg afterwards, because that is how the motors work.
 *
 * To counter act the problem, the measured position offsets are used to add offsets to the request,
 * if the request planned to move in direction of the measured position. So in our example, the original
 * request could change until -30deg, an offset would be applied to keep the request at -25deg. Only when the request
 * reached < -30deg, like -31deg, the offset would stay at -5deg, resulting in a measured position of -31deg.
 * In case the real joint gets unstuck and moves to the requested position, the offset gets reduced.
 *
 * To prevent overbalancing/overadjusting, we use a neural network to predict the measured joint positions.
 * The usage of the neural network is in the provider of JointPlayTranslation (JointPlayTranslationProvider).
 * This helps to reduce the offsets in time.
 *
 * @author Philip Reichenberg
 */

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/JointPlayTranslation.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(JointPlayOffsetParameters,
{,
  (float) applyOffsetFactor, /**< Apply position offset with this scaling factor. */
  (float) applyOffsetFactorAnkle, /**< Apply position offset with this scaling factor, if the joint is an anklePitch one. */
});

struct JointPlayOffsetRegulator : std::array<Angle, Joints::numOfJoints>
{
  JointPlayOffsetRegulator(const JointRequest& request);

  /**
   * Calculate the new offsets
   * @param jointPlayTranslation The representation holding the information about the current measured offsets
   * @param request The joint request
   * @param isRightSupport Is the right foot the supporting one
   * @param params The parameters
   * @param appliedChanged Changes already applied by other balancing regulations
   * @param ratio % of the walk step
   */
  void update(const JointPlayTranslation& jointPlayTranslation, JointRequest& request,
              const bool isRightSupport, const JointPlayOffsetParameters& params,
              const JointAngles& appliedChanged, const JointAngles& appliedGyro, const float ratio);

private:
  /**
   * Last modified joint request
   */
  JointRequest lastRequest;
};
