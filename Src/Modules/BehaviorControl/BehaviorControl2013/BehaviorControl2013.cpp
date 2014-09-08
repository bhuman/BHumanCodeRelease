/**
 * @file BehaviorControl2013.h
 * Implementation of a C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "Libraries.h"
#include "Tools/Team.h"
#include "Tools/Streams/InStreams.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX Behavior2013::Behavior
#endif
#include "Tools/Cabsl.h" // include last, because macros might mix up other header files

namespace Behavior2013
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
    Behavior(const BehaviorControl2013Base& base, BehaviorControlOutput& behaviorControlOutput)
    : Cabsl<Behavior>(&behaviorControlOutput.executionGraph),
      Libraries(base, behaviorControlOutput) {}

    /**
     * Executes one behavior cycle.
     * @param roots A set of root options. They must be parameterless.
     */
    void execute(const std::vector<OptionInfos::Option>& roots)
    {
      theOwnTeamInfo = BehaviorControl2013Base::theOwnTeamInfo;
      theRobotInfo = BehaviorControl2013Base::theRobotInfo;
      theGameInfo = BehaviorControl2013Base::theGameInfo;

      beginFrame(theFrameInfo.time);
      preProcessLibraries();

      for(std::vector<Behavior::OptionInfos::Option>::const_iterator i = roots.begin(); i != roots.end(); ++i)
      Cabsl<Behavior>::execute(*i);

      postProcessLibraries();
      endFrame();
    }
  };
}

using namespace Behavior2013;

/**
 * @class BehaviorControl2013
 * A C-based state machine behavior control module.
 */
class BehaviorControl2013 : public BehaviorControl2013Base
{
  STREAMABLE(Parameters,
  {
    /** Helper for streaming a vector of enums that are defined in another class. */
    struct OptionInfos : Behavior::OptionInfos {typedef std::vector<Option> Options;},

    (OptionInfos, Options) roots, /**< All options that function as behavior roots. */
  });

  void update(BehaviorControlOutput& behaviorControlOutput)
  {
    Parameters p(parameters); // make a copy, to make "unchanged" work
    MODIFY("parameters:BehaviorControl2013", p);
    if(theFrameInfo.time)
    {
      behaviorControlOutput.behaviorStatus.role = theRole.role;
      behaviorControlOutput.behaviorStatus.teamColor = theOwnTeamInfo.teamColor == TEAM_BLUE ? BehaviorStatus::blue : BehaviorStatus::red;

      TEAM_OUTPUT_FAST(idWalkTarget, bin, theRobotPose.translation);
      TEAM_OUTPUT_FAST(idKickTarget, bin, theRobotPose.translation);

      theBehavior->execute(p.roots);

      TEAM_OUTPUT_FAST(idTeammateBehaviorStatus, bin, behaviorControlOutput.behaviorStatus);
    }
  }

  /** Updates the motion request by copying from behavior control output */
  void update(MotionRequest& motionRequest) {motionRequest = theBehaviorControlOutput.motionRequest;}

  /** Updates the arm motion request by copying from behavior control output */
  void update(ArmMotionRequest& armMotionRequest) {armMotionRequest = theBehaviorControlOutput.armMotionRequest;}

  /** Updates the head motion request by copying from behavior control output */
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = theBehaviorControlOutput.headMotionRequest;}

  /** Update the behavior led request by copying from behavior control output */
  void update(BehaviorLEDRequest& behaviorLEDRequest) {behaviorLEDRequest = theBehaviorControlOutput.behaviorLEDRequest;}

  /** Update the behavior execution graph by copying from behavior control output */
  void update(ActivationGraph& executionGraph) {executionGraph = theBehaviorControlOutput.executionGraph;}

  Parameters parameters; /**< The root options. */
  Behavior* theBehavior; /**< The behavior with all options and libraries. */

public:
  BehaviorControl2013()
  : theBehavior(new Behavior(*this, const_cast<BehaviorControlOutput&>(theBehaviorControlOutput)))
  {
    InMapFile stream("behaviorControl2013.cfg");
    ASSERT(stream.exists());
    stream >> parameters;
  }

  ~BehaviorControl2013() {delete theBehavior;}
};

MAKE_MODULE(BehaviorControl2013, Behavior Control)
