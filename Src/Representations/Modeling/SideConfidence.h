/**
 * @file SideConfidence.h
 *
 * Declaration of struct SideConfidence.
 * @author Michel Bartsch, Thomas Münder
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"

/**
 * @struct SideConfidence
 */
STREAMABLE(SideConfidence, COMMA public BHumanMessageParticle<idSideConfidence>
{
  ENUM(ConfidenceState,
  {,
    CONFIDENT,
    ALMOST_CONFIDENT,
    UNSURE,
    CONFUSED,
  }); /**< Discrete states of confidence, mapped by provider */

  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  void operator << (const BHumanMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  void verify() const;
  /** Draw representation. */
  void draw() const,

  (float)(1) sideConfidence,                     /**< Am I mirrored because of field symmetry (0 = no idea, 1 = absolute sure I am right). */
  (bool)(false) mirror,                          /**< Indicates whether ball model of others is mirrored to own ball model. */
  (ConfidenceState)(CONFIDENT) confidenceState,  /**< The state of confidence */
  (std::vector<int>) agreeMates,                 /** The robot numbers of the robots the agree with me regarding the side */
});

inline void SideConfidence::operator >> (BHumanMessage& m) const
{
  m.theBSPLStandardMessage.currentSideConfidence = static_cast<int8_t>(sideConfidence * 100);

  m.theBHumanArbitraryMessage.queue.out.bin << *this;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
}

inline void SideConfidence::operator << (const BHumanMessage& m)
{
  sideConfidence = static_cast<float>(m.theBSPLStandardMessage.currentSideConfidence) / 100.f;
  mirror = false;
  confidenceState = CONFIDENT;
  agreeMates.clear();
}

inline bool SideConfidence::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> *this;

  return true;
}

inline void SideConfidence::verify() const
{
  ASSERT(sideConfidence >= 0.f);
  ASSERT(sideConfidence <= 1.f);
}

inline void SideConfidence::draw() const
{
  DEBUG_DRAWING3D("representation:SideConfidence", "robot")
  {
    static const ColorRGBA colors[numOfConfidenceStates] =
    {
      ColorRGBA::green,
      ColorRGBA(0, 128, 0),
      ColorRGBA::yellow,
      ColorRGBA::red
    };
    int pNumber = Global::getSettings().playerNumber;
    float centerDigit = (pNumber > 1) ? 50.f : 0;
    ROTATE3D("representation:SideConfidence", 0, 0, pi_2);
    DRAWDIGIT3D("representation:SideConfidence", pNumber, Vector3f(centerDigit, 0.f, 500.f), 80, 5, colors[confidenceState]);
  }

  DEBUG_DRAWING("representation:SideConfidence", "drawingOnField")
  {
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    DRAWTEXT("representation:SideConfidence", xPosOwnFieldBorder + 200, yPosRightFieldBorder + 100, (xPosOwnFieldBorder / -5200.f) * 140, ColorRGBA::red, "Sideconfidence: " << sideConfidence);
  }
}
