/**
 * @file BehaviorControl2015.cpp
 * Implementation of a C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "Libraries.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/InStreams.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX Behavior2015::Behavior::
#endif
#include "Tools/Cabsl.h" // include last, because macros might mix up other header files

namespace Behavior2015
{
  /**
   * The wrapper for the behavior options.
   */
  class Behavior : public Cabsl<Behavior>, public Libraries
  {
#include "Options.h"
  public:
    /**
     * Constructor.
     * @param base The blackboard configured for this module.
     */
    Behavior(const BehaviorControl2015Base& base, BehaviorData& behaviorData)
      : Cabsl<Behavior>(&behaviorData.theActivationGraph),
        Libraries(base, behaviorData) {}

    /**
     * Executes one behavior cycle.
     * @param roots A set of root options. They must be parameterless.
     */
    void execute(const std::vector<OptionInfos::Option>& roots)
    {
      beginFrame(theFrameInfo.time);
      preProcessLibraries();

      for(std::vector<Behavior::OptionInfos::Option>::const_iterator i = roots.begin(); i != roots.end(); ++i)
        Cabsl<Behavior>::execute(*i);

      postProcessLibraries();
      endFrame();
    }
  };
}

using namespace Behavior2015;

/**
 * @class BehaviorControl2015
 * A C-based state machine behavior control module.
 */
class BehaviorControl2015 : public BehaviorControl2015Base
{
  STREAMABLE(Parameters,
  {
    /** Helper for streaming a vector of enums that are defined in another class. */
    struct OptionInfos : Behavior::OptionInfos { using Options = std::vector<Option>;},

    ((OptionInfos) Options) roots, /**< All options that function as behavior roots. */
  });

  void update(ActivationGraph& activationGraph)
  {
    Parameters p(parameters); // make a copy, to make "unchanged" work
    MODIFY("parameters:BehaviorControl2015", p);
    if(theFrameInfo.time)
    {
      theBehaviorStatus.role = theRole.role;

      theSPLStandardBehaviorStatus.walkingTo  = theRobotPose.translation;
      theSPLStandardBehaviorStatus.shootingTo = theRobotPose.translation;

      theBehavior->execute(p.roots);

      theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_DEFAULT;
      if(theSideConfidence.confidenceState == SideConfidence::CONFUSED)
        theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_LOST;
    }
  }

  /** Update the behavior led request by copying it from the behavior */
  void update(BehaviorLEDRequest& behaviorLEDRequest) {behaviorLEDRequest = theBehaviorLEDRequest;}

  /** Updates the motion request by copying it from the behavior */
  void update(BehaviorMotionRequest& behaviorMotionRequest) {(MotionRequest&) behaviorMotionRequest = theMotionRequest;}

  /** Update the behavior status by copying it from the behavior */
  void update(BehaviorStatus& behaviorStatus) {behaviorStatus = theBehaviorStatus;}

  /** Updates the head motion request by copying it from the behavior */
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = theHeadMotionRequest;}

  /** Updates the arm motion request by copying it from the behavior */
  void update(ArmMotionRequest& armMotionRequest) { armMotionRequest = theArmMotionRequest; }

  /** Updates the standard behavior status by copying it from the behavior */
  void update(SPLStandardBehaviorStatus& splStandardBehaviorStatus) {splStandardBehaviorStatus = theSPLStandardBehaviorStatus;}

  Parameters parameters; /**< The root options. */
  BehaviorLEDRequest theBehaviorLEDRequest;
  BehaviorStatus theBehaviorStatus;
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that will be set. */
  ArmMotionRequest theArmMotionRequest;
  MotionRequest theMotionRequest;
  SPLStandardBehaviorStatus theSPLStandardBehaviorStatus;
  BehaviorData behaviorData; /**< References to the representations above. */
  Behavior* theBehavior; /**< The behavior with all options and libraries. */

public:
  BehaviorControl2015()
    : behaviorData(const_cast<ActivationGraph&>(theActivationGraph),
                   theBehaviorLEDRequest,
                   theBehaviorStatus,
                   theHeadMotionRequest,
                   theArmMotionRequest,
                   theMotionRequest,
                   theSPLStandardBehaviorStatus) ,
      theBehavior(new Behavior(*this, behaviorData))
  {
    InMapFile stream("behaviorControl2015.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }

  ~BehaviorControl2015() {delete theBehavior;}
};

MAKE_MODULE(BehaviorControl2015, behaviorControl)
