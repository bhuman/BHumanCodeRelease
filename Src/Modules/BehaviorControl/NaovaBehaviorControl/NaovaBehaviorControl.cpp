/**
 * @file NaovaBehaviorControl.cpp
 * Implementation of a C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "Libraries.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/InStreams.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX NaovaBehavior::Behavior::
#endif
#include "Tools/Cabsl.h" // include last, because macros might mix up other header files

namespace NaovaBehavior
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
    Behavior(const NaovaBehaviorControlBase& base, BehaviorData& behaviorData)
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

using namespace NaovaBehavior;

/**
 * @class NaovaBehaviorControl
 * A C-based state machine behavior control module.
 */
class NaovaBehaviorControl : public NaovaBehaviorControlBase
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
    MODIFY("parameters:NaovaBehaviorControl", p);
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
  NaovaBehaviorControl()
    : behaviorData(const_cast<ActivationGraph&>(theActivationGraph),
                   theBehaviorLEDRequest,
                   theBehaviorStatus,
                   theHeadMotionRequest,
                   theArmMotionRequest,
                   theMotionRequest,
                   theSPLStandardBehaviorStatus) ,
      theBehavior(new Behavior(*this, behaviorData))
  {
    InMapFile stream("NaovaBehaviorControl.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }

  ~NaovaBehaviorControl() {delete theBehavior;}
};

MAKE_MODULE(NaovaBehaviorControl, behaviorControl)
