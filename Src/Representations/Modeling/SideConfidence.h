/**
 * @file SideConfidence.h
 *
 * Declaration of struct SideConfidence.
 * @author Michel Bartsch, Thomas Münder
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Settings.h"

/**
 * @struct SideConfidence
 */
STREAMABLE(SideConfidence, COMMA public PureBHumanArbitraryMessageParticle<idSideConfidence>
{
  ENUM(ConfidenceState,
  {,
    CONFIDENT,
    ALMOST_CONFIDENT,
    UNSURE,
    CONFUSED,
  }); /**< Discrete states of confidence, mapped by provider */

  /** Draw representation. */
  void draw() const,

  (bool)(false) mirror,                          /**< Indicates whether ball model of others is mirrored to own ball model. */
  (ConfidenceState)(CONFIDENT) confidenceState,  /**< The state of confidence */
  (std::vector<int>) agreeMates,                 /** The robot numbers of the robots the agree with me regarding the side */
});

inline void SideConfidence::draw() const
{
  DEBUG_DRAWING3D("representation:SideConfidence", "robot")
  {
    static const ColorRGBA colors[numOfConfidenceStates] =
    {
      ColorRGBA::green,
      ColorRGBA(0, 192, 0),
      ColorRGBA::yellow,
      ColorRGBA::red
    };
    int pNumber = Global::getSettings().playerNumber;
    float centerDigit = (pNumber > 1) ? 50.f : 0;
    ROTATE3D("representation:SideConfidence", 0, 0, pi_2);
    DRAWDIGIT3D("representation:SideConfidence", pNumber, Vector3f(centerDigit, 0.f, 500.f), 80, 5, colors[confidenceState]);
  }
}
