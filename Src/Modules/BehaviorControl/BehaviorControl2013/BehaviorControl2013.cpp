/**
 * @file BehaviorControl2013.h
 * Implementation of a C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#include "Tools/Team.h"
#include "Tools/Streams/InStreams.h"
#include "Libraries.h"


namespace Behavior2013
{
  /**
   * The wrapper for the behavior options.
   */
  class Behavior : public Libraries
  {
    static PROCESS_WIDE_STORAGE(Behavior) _theInstance; /**< The instance of this behavior used. */

  public:
#include "Cabsl.h"

  private:
    OptionContext::StateType _stateType; /**< The state type of the last option called. */
    unsigned _lastFrameTime; /**< The time stamp of the last time the behavior was executed. */
    unsigned char _depth; /**< The depth level of the current option. Used for sending debug messages. */

#include "Options.h"

  public:
    /**
     * Constructor.
     * @param base The blackboard configured for this module.
     */
    Behavior(const BehaviorControl2013Base& base, BehaviorControlOutput& behaviorControlOutput) :
      Libraries(base, behaviorControlOutput)
    {
      _theInstance = this;
    }


    /* Destructor. */
    ~Behavior() {_theInstance = 0;}

    /**
     * Executes one behavior cycle.
     * @param roots A set of root options. They must be parameterless.
     */
    void update(const std::vector<OptionInfos::Option>& roots)
    {
      theOwnTeamInfo = BehaviorControl2013Base::theOwnTeamInfo;
      theRobotInfo = BehaviorControl2013Base::theRobotInfo;
      theGameInfo = BehaviorControl2013Base::theGameInfo;
      theActivationGraph.graph.clear();

      preProcessLibraries();

      for(std::vector<Behavior::OptionInfos::Option>::const_iterator i = roots.begin(); i != roots.end(); ++i)
        OptionInfos::execute(this, *i);

      postProcessLibraries();

      _lastFrameTime = theFrameInfo.time;
    }

    /**
     * The operator allocates a memory block that is zeroed.
     * Therefore, all members of this class are initialized with 0.
     * @attention This operator is only called if this class is instantiated by
     * a separate call to new, i.e. it cannot be created as a part of another class.
     * @param size The size of the block in bytes.
     * @return A pointer to the block.
     */
    static void* operator new(std::size_t size)
    {
      return calloc(1, size);
    }

    /**
     * The operator frees a memory block.
     * @param p The address of the block to free.
     */
    static void operator delete(void* p)
    {
      return free(p);
    }
  };

  PROCESS_WIDE_STORAGE(Behavior) Behavior::_theInstance;
  std::unordered_map<std::string, Behavior::OptionDescriptor*> Behavior::OptionInfos::optionsByName;
  std::vector<Behavior::OptionDescriptor> Behavior::OptionInfos::optionsByIndex;
  Behavior::OptionInfos collectOptions; /**< This global instantiation collects data about all options. */
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
      behaviorControlOutput.behaviorStatus.teamColor = theOwnTeamInfo.teamColor == TEAM_BLUE ? BehaviorStatus::blue : BehaviorStatus::red;

      theBehavior->update(p.roots);

      TEAM_OUTPUT_FAST(idTeamMateBehaviorStatus, bin, behaviorControlOutput.behaviorStatus);
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
