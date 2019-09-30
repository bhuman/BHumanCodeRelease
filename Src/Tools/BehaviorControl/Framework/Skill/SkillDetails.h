/**
 * @file SkillDetails.h
 *
 * This file declares macros and classes to declare skills.
 * DO NOT INCLUDE THIS FILE DIRECTLY FROM OUTSIDE THIS DIRECTORY!
 *
 * Skills are the parts of the robot behavior that have a well-defined task.
 * They can be composed of multiple sub-skills by calling them.
 *
 * The skill interface exposes its signature, i.e. which parameters it takes.
 * Example:
 *
 * SKILL_INTERFACE(WalkToPoint, (const Pose2f&) target, (float)(1.f) speed, (bool)(false) rough,
 *                              (bool)(false) disableObstacleAvoidance, (bool)(false) disableAligning);
 *
 * A skill implementation contains the actual behavior of the skill. In their declaration,
 * skill implementations are similar to modules.
 * Example:
 *
 * SKILL_IMPLEMENTATION(WalkToPointImpl,
 * {,
 *   IMPLEMENTS(WalkToPoint),   // The skill implements WalkToPoint.
 *   REQUIRES(ArmContactModel), // The skill can access the ArmContactModel and it is made sure ...
 *   REQUIRES(FootBumperState), // ... that this representation is updated before the BehaviorControl module is run.
 *   REQUIRES(GameInfo),        // ...
 *   CALLS(PathToTarget),       // The skill can call the PathToTarget skill.
 *   CALLS(WalkToTarget),       // ...
 *   DEFINES_PARAMETERS(
 *   {,
 *     (float)(1000.f) switchToLibWalkDistance, // Parameters are defined as in modules.
 *   }),
 * });
 *
 * This declaration generates a class called WalkToPointImplBase. The actual skill implementation
 * has to be derived from it:
 *
 * class WalkToPointImpl : public WalkToPointImplBase
 * {
 *   void execute(WalkToPoint& p) override;
 * };
 *
 * Optionally, the methods reset, isDone and isAborted can be overridden per implemented skill.
 *
 * In the implementation file, the existence of the skill implementation has to be announced:
 *
 * REGISTER_SKILL_IMPLEMENTATION(WalkToPointImpl);
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "SkillRegistryBase.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Tools/BehaviorControl/Framework/BehaviorContext.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/OutStreams.h"
#include <type_traits>

class SkillImplementationInterface;

class SkillInterface
{
public:
  /** Virtual destructor for polymorphism. */
  virtual ~SkillInterface() = default;
  /** Calls the \c preProcess method of the implementation for this skill. */
  virtual void preProcess() = 0;
  /** Calls the \c postProcess method of the implementation for this skill. */
  virtual void postProcess() = 0;

protected:
  /**
   * Sets the implementation of a skill interface.
   * @param implementation The implementation that should be called for calls to the interface.
   */
  virtual void setImplementation(SkillImplementationInterface* implementation) = 0;
  friend class SkillRegistryBase;
};

class SkillInterfaceCreator
{
public:
  /**
   * Constructor.
   * @param name The name of the skill interface
   * @param createNew A creator for the skill interface
   */
  SkillInterfaceCreator(const char* name, SkillInterface* (*createNew)()) :
    name(name), createNew(createNew)
  {}

private:
  const char* name; /**< The name of the skill interface. */
  SkillInterface* (*createNew)(); /**< A creator for the skill interface. */
  friend class SkillRegistryBase;
};

#define _SKILL_INTERFACE(registry, ...) \
  _SKILL_INTERFACE_I(_SKILL_INTERFACE_HAS_PARAMS(__VA_ARGS__), registry, __VA_ARGS__)
#define _SKILL_INTERFACE_I(hasParams, ...) _SKILL_INTERFACE_II(hasParams, (__VA_ARGS__))
#define _SKILL_INTERFACE_II(hasParams, params) _SKILL_INTERFACE_##hasParams params

#define _SKILL_INTERFACE_0(registry, name) _SKILL_INTERFACE_IV(1, registry, name, (f,), (f,), (f,), (f,))
#define _SKILL_INTERFACE_1(registry, name, ...) _SKILL_INTERFACE_III(_STREAM_TUPLE_SIZE(__VA_ARGS__, ignore), registry, name, __VA_ARGS__, ignore)

#define _SKILL_INTERFACE_DECL(seq) std::remove_const<std::remove_reference<decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type)>::type>::type _STREAM_VAR(seq);
#define _SKILL_INTERFACE_DECL2(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq);
#define _SKILL_INTERFACE_PARAM(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq) _STREAM_INIT(seq),
#define _SKILL_INTERFACE_PARAM2(seq) decltype(Streaming::TypeWrapper<_STREAM_DECL_I seq))>::type) _STREAM_VAR(seq) _STREAM_INIT(seq)
#define _SKILL_INTERFACE_ASSIGN(seq) this->_STREAM_VAR(seq) = _STREAM_VAR(seq);
#define _SKILL_INTERFACE_STREAM(seq) \
  _STREAM_JOIN(_SKILL_INTERFACE_STREAM_, _STREAM_SEQ_SIZE(seq))(seq) \
  { \
    struct _S : public Streamable \
    { \
      _SKILL_INTERFACE_DECL2(seq) \
      void serialize(In* in, Out* out) \
      { \
        auto _STREAM_VAR(seq) = this->_STREAM_VAR(seq); \
        _STREAM_SER(seq) \
      } \
      _S(_SKILL_INTERFACE_PARAM2(seq)) : _STREAM_VAR(seq)(_STREAM_VAR(seq)) {} \
    } _s(_STREAM_VAR(seq)); \
    OutMapMemory stream(true, 1024); \
    stream << _s; \
    _parameters.emplace_back(stream.data()); \
  }

/** If a default value exists, only stream parameters that are different from it. */
#define _SKILL_INTERFACE_STREAM_1(seq)
#define _SKILL_INTERFACE_STREAM_2(seq) if(!(_STREAM_VAR(seq) == _STREAM_INIT_I_2_I(seq)))

#define _SKILL_INTERFACE_III(n, registry, name, ...) _SKILL_INTERFACE_IV(n, registry, name, (_SKILL_INTERFACE_DECL, __VA_ARGS__), (_SKILL_INTERFACE_PARAM, __VA_ARGS__), (_SKILL_INTERFACE_ASSIGN, __VA_ARGS__), (_SKILL_INTERFACE_STREAM, __VA_ARGS__))
#define _SKILL_INTERFACE_IV(n, registry, name, params1, params2, params3, params4) \
  struct name \
  { \
    _STREAM_ATTR_##n params1 \
    mutable BehaviorContext _context; \
    void setState(const char* name) const \
    { \
      if(name != _context.stateName) \
      { \
        _context.stateName = name; \
        _context.stateStart = registry::theInstance->currentFrameTime; \
      } \
    } \
  protected: \
    name() {} \
  }; \
  class name##Skill final : name, public SkillInterface \
  { \
  public: \
    static SkillInterface* createNew() \
    { \
      return new name##Skill(); \
    } \
    struct Implementation \
    { \
      virtual void reset(const name&) {} \
      virtual void execute(const name&) = 0; \
      virtual bool isDone(const name&) const { return false; } \
      virtual bool isAborted(const name&) const { return false; } \
      virtual void preProcess(const name&) {} \
      virtual void postProcess(const name&) {} \
    }; \
    void operator ()(_STREAM_ATTR_##n params2 const void* _unused = nullptr) \
    { \
      _STREAM_ATTR_##n params3 \
      if(_context.lastFrame != registry::theInstance->lastFrameTime && _context.lastFrame != registry::theInstance->currentFrameTime) \
      { \
        _context.behaviorStart = registry::theInstance->currentFrameTime; \
        _context.stateName = nullptr; \
        theImplementation->reset(*this); \
      } \
      std::vector<std::string> _parameters; \
      _STREAM_ATTR_##n params4 \
      ActivationGraph& theActivationGraph = registry::theInstance->theActivationGraph; \
      const size_t activationGraphIndex = theActivationGraph.graph.size(); \
      theActivationGraph.graph.emplace_back(#name, ++theActivationGraph.currentDepth, "", registry::theInstance->currentFrameTime - _context.behaviorStart, 0, _parameters); \
      theImplementation->execute(*this); \
      if(_context.stateName) \
      { \
        theActivationGraph.graph[activationGraphIndex].state = _context.stateName; \
        theActivationGraph.graph[activationGraphIndex].stateTime = registry::theInstance->currentFrameTime - _context.stateStart; \
      } \
      --theActivationGraph.currentDepth; \
      _context.lastFrame = registry::theInstance->currentFrameTime; \
    } \
    bool isDone() const \
    { \
      return theImplementation->isDone(*this); \
    } \
    bool isAborted() const \
    { \
      return theImplementation->isAborted(*this); \
    } \
  private: \
    void preProcess() override \
    { \
      theImplementation->preProcess(*this); \
    } \
    void postProcess() override \
    { \
      theImplementation->postProcess(*this); \
    } \
    void setImplementation(SkillImplementationInterface* implementation) override \
    { \
      theImplementation = dynamic_cast<Implementation*>(implementation); \
    } \
    Implementation* theImplementation = nullptr; \
  };

/** Check whether a skill has parameters. */
#define _SKILL_INTERFACE_HAS_PARAMS(...) _SKILL_INTERFACE_HAS_PARAMS_I((__VA_ARGS__, _SKILL_INTERFACE_HAS_PARAMS_II))
#define _SKILL_INTERFACE_HAS_PARAMS_I(params) _STREAM_TUPLE_SIZE_II params
#define _SKILL_INTERFACE_HAS_PARAMS_II \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0

class SkillImplementationInterface
{
public:
  /** Virtual destructor for polymorphism. */
  virtual ~SkillImplementationInterface() = default;

protected:
  /** This method is executed each frame before any skill is called. */
  virtual void preProcess() {}
  /** This method is executed each frame after all skills have been called. */
  virtual void postProcess() {}
  /** This method is executed each frame to modify the parameters of a skill implementation. */
  virtual void modifyParameters() = 0;
  friend class SkillRegistryBase;
};

class SkillImplementationCreatorBase
{
public:
  struct Info
  {
    std::vector<const char*> requires; /**< The names of representations required by this skill implementation. */
    std::vector<SkillInterfaceCreator> implements; /**< Info about skills implemented by this skill implementation. */
    std::vector<const char*> calls; /**< The names of skills called by this skill implementation. */
  };

  /**
   * Constructor.
   * @param name The name of the skill that can be created by this instance.
   * @param getSkillInfo The function that returns the info.
   * @param next The creator that shall be next in the list.
   */
  SkillImplementationCreatorBase(const char* name, Info (*getSkillInfo)(), SkillImplementationCreatorBase* next) :
    name(name), getSkillInfo(getSkillInfo), next(next)
  {}

  /**
   * Adds the requirements of all skill implementations to the info of a module.
   * @param firstCreator The anchor of the creator list.
   * @param info The info to which the requirements are added.
   */
  static void addToModuleInfo(SkillImplementationCreatorBase* firstCreator, std::vector<ModuleBase::Info>& info);

protected:
  /**
   * Abstract method to create an instance of a skill.
   * @return A pointer to the new instance.
   */
  virtual SkillImplementationInterface* createNew() = 0;

private:
  const char* name; /**< The name of the skill that can be created by this instance. */
  Info (*getSkillInfo)(); /**< A function that returns the info of this skill. */
  SkillImplementationCreatorBase* next; /**< The next entry in the list of all skills. */
  friend class SkillRegistryBase;
};

template<typename Instance>
class SkillImplementationCreatorList : public SkillImplementationCreatorBase
{
public:
  /**
   * Constructor.
   * @param name The name of the skill that can be created by this instance.
   * @param getSkillInfo The function that returns the info.
   */
  SkillImplementationCreatorList(const char* name, Info (*getSkillInfo)()) :
    SkillImplementationCreatorBase(name, getSkillInfo, first)
  {
    first = this;
  }

  static SkillImplementationCreatorBase* first; /**< The head of the list of all skills available. */
};

template<typename Instance> SkillImplementationCreatorBase* SkillImplementationCreatorList<Instance>::first = nullptr;

template<typename Instance, typename S>
class SkillImplementationCreator : public SkillImplementationCreatorList<Instance>
{
public:
  /**
   * Constructor.
   * @param name The name of the skill implementation that can be created by this instance.
   * @param getSkillInfo The function that returns the info.
   */
  SkillImplementationCreator(const char* name, SkillImplementationCreatorBase::Info (*getSkillInfo)()) :
    SkillImplementationCreatorList<Instance>(name, getSkillInfo)
  {
  }

private:
  /**
   * Creates a new instance of this skill implementation.
   * @return A pointer to the new instance.
   */
  SkillImplementationInterface* createNew() override
  {
    return new S;
  }
};

#define _SKILL_IMPLEMENTATION_PARAMETERS(x) _MODULE_JOIN(_SKILL_IMPLEMENTATION_PARAMETERS_, x)
#define _SKILL_IMPLEMENTATION_PARAMETERS_IMPLEMENTS(type)
#define _SKILL_IMPLEMENTATION_PARAMETERS_REQUIRES(type)
#define _SKILL_IMPLEMENTATION_PARAMETERS_USES(type)
#define _SKILL_IMPLEMENTATION_PARAMETERS_MODIFIES(type)
#define _SKILL_IMPLEMENTATION_PARAMETERS_CALLS(type)
#define _SKILL_IMPLEMENTATION_PARAMETERS__MODULE_DEFINES_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;
#define _SKILL_IMPLEMENTATION_PARAMETERS__MODULE_LOADS_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;

#define _SKILL_IMPLEMENTATION_LOAD(x) _MODULE_JOIN(_SKILL_IMPLEMENTATION_LOAD_, x)
#define _SKILL_IMPLEMENTATION_LOAD_IMPLEMENTS(type)
#define _SKILL_IMPLEMENTATION_LOAD_REQUIRES(type)
#define _SKILL_IMPLEMENTATION_LOAD_USES(type)
#define _SKILL_IMPLEMENTATION_LOAD_MODIFIES(type)
#define _SKILL_IMPLEMENTATION_LOAD_CALLS(type)
#define _SKILL_IMPLEMENTATION_LOAD__MODULE_DEFINES_PARAMETERS(...)
#define _SKILL_IMPLEMENTATION_LOAD__MODULE_LOADS_PARAMETERS(...) loadModuleParameters(*this, skillName, fileName, "BehaviorControl/");

#define _SKILL_IMPLEMENTATION_DECLARE(x) _MODULE_JOIN(_SKILL_IMPLEMENTATION_DECLARE_, x)
#define _SKILL_IMPLEMENTATION_DECLARE_IMPLEMENTS(type) \
  protected: \
    using type = _SKILLS_NAMESPACE::type;
#define _SKILL_IMPLEMENTATION_DECLARE_REQUIRES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type);
#define _SKILL_IMPLEMENTATION_DECLARE_USES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type);
#define _SKILL_IMPLEMENTATION_DECLARE_MODIFIES(type) public: type& the##type = _SKILL_REGISTRY::theInstance->the##type;
#define _SKILL_IMPLEMENTATION_DECLARE_CALLS(type) public: _SKILLS_NAMESPACE::type##Skill& the##type##Skill = *_SKILL_REGISTRY::theInstance->getSkill<_SKILLS_NAMESPACE::type##Skill>(#type);
#define _SKILL_IMPLEMENTATION_DECLARE__MODULE_DEFINES_PARAMETERS(...)
#define _SKILL_IMPLEMENTATION_DECLARE__MODULE_LOADS_PARAMETERS(...)

#define _SKILL_IMPLEMENTATION_FREE(x) _MODULE_JOIN(_SKILL_IMPLEMENTATION_FREE_, x)
#define _SKILL_IMPLEMENTATION_FREE_IMPLEMENTS(type)
#define _SKILL_IMPLEMENTATION_FREE_REQUIRES(type) Blackboard::getInstance().free(#type);
#define _SKILL_IMPLEMENTATION_FREE_USES(type) Blackboard::getInstance().free(#type);
#define _SKILL_IMPLEMENTATION_FREE_MODIFIES(type)
#define _SKILL_IMPLEMENTATION_FREE_CALLS(type)
#define _SKILL_IMPLEMENTATION_FREE__MODULE_DEFINES_PARAMETERS(...)
#define _SKILL_IMPLEMENTATION_FREE__MODULE_LOADS_PARAMETERS(...)

#define _SKILL_IMPLEMENTATION_INFO(x) _MODULE_JOIN(_SKILL_IMPLEMENTATION_INFO_, x)
#define _SKILL_IMPLEMENTATION_INFO_IMPLEMENTS(type) _info.implements.emplace_back(#type, _SKILLS_NAMESPACE::type##Skill::createNew);
#define _SKILL_IMPLEMENTATION_INFO_REQUIRES(type) _info.requires.push_back(#type);
#define _SKILL_IMPLEMENTATION_INFO_USES(type)
#define _SKILL_IMPLEMENTATION_INFO_MODIFIES(type)
#define _SKILL_IMPLEMENTATION_INFO_CALLS(type) _info.calls.push_back(#type);
#define _SKILL_IMPLEMENTATION_INFO__MODULE_DEFINES_PARAMETERS(...)
#define _SKILL_IMPLEMENTATION_INFO__MODULE_LOADS_PARAMETERS(...)

#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS(x) _MODULE_JOIN(_SKILL_IMPLEMENTATION_IMPLEMENTATIONS_, x)
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS_IMPLEMENTS(type) public _SKILLS_NAMESPACE::type##Skill::Implementation,
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS_REQUIRES(type)
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS_USES(type)
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS_MODIFIES(type)
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS_CALLS(type)
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS__MODULE_DEFINES_PARAMETERS(...)
#define _SKILL_IMPLEMENTATION_IMPLEMENTATIONS__MODULE_LOADS_PARAMETERS(...)

/**
 * Helper for defining the skill implementation's base class.
 * @param name The name of the skill implementation.
 * @param header Normally, only an opening brace.
 * @param n The number of entries in the third parameter.
 * @param ... The requirements, implemented and called skills and parameter definitions.
 */
#define _SKILL_IMPLEMENTATION_I(registry, skills, name, header, n, ...) _SKILL_IMPLEMENTATION_II(name, header, n, (_SKILL_IMPLEMENTATION_PARAMETERS, __VA_ARGS__), (_SKILL_IMPLEMENTATION_LOAD, __VA_ARGS__), (_SKILL_IMPLEMENTATION_DECLARE, __VA_ARGS__), (_SKILL_IMPLEMENTATION_FREE, __VA_ARGS__), (_SKILL_IMPLEMENTATION_INFO, __VA_ARGS__), (_SKILL_IMPLEMENTATION_IMPLEMENTATIONS, __VA_ARGS__), (__VA_ARGS__))

/**
 * Generates the actual code of the skill implementation's base class.
 * It creates all the code and fills in data from the requirements and parameters defined.
 */
#define _SKILL_IMPLEMENTATION_II(theName, header, n, params, load, declare, free, info, implementations, tail) \
  namespace theName##Skill \
  { \
    _MODULE_ATTR_##n params \
    using Parameters = NoParameters; \
  } \
  class theName; \
  class theName##Base : public theName##Skill::Parameters COMMA _MODULE_ATTR_##n implementations public SkillImplementationInterface \
  _STREAM_UNWRAP header; \
  private: \
    using BaseType = theName##Base; \
    void modifyParameters() override \
    { \
      if(sizeof(NoParameters) < sizeof(theName##Skill::Parameters)) \
      { \
        Global::getDebugDataTable().updateObject("parameters:" #theName, *this, false); \
        DEBUG_RESPONSE_ONCE("debug data:parameters:" #theName) \
          OUTPUT(idDebugDataResponse, bin, "parameters:" #theName << TypeRegistry::demangle(typeid(theName##Skill::Parameters).name()) << *this); \
      } \
    } \
  public: \
    static SkillImplementationCreatorBase::Info getSkillInfo() \
    { \
      SkillImplementationCreatorBase::Info _info; \
      _MODULE_ATTR_##n info \
      return _info; \
    } \
  private: \
    _MODULE_ATTR_##n declare \
  public: \
    using Parameters = theName##Skill::Parameters; \
    theName##Base(const char* fileName = nullptr) \
    { \
      static const char* skillName = #theName; \
      static_cast<void>(skillName); \
      _MODULE_ATTR_##n load \
    } \
    ~theName##Base() \
    { \
      _MODULE_ATTR_##n free \
    } \
    theName##Base(const theName##Base&) = delete; \
    theName##Base& operator=(const theName##Base&) = delete; \
  protected: \
    _MODULE_ATTR_LAST_##n tail

/**
 * Generate a base class for a skill implementation.
 * See beginning of this file.
 * @param registry The skill registry from which to obtain skill handles.
 * @param skills The skill interface namespace to use.
 * @param name The name of the skill implementation.
 * @param header Normally, only an opening brace.
 * @param ... The requirements, implemented and called skills and parameter definitions.
 */
#define _SKILL_IMPLEMENTATION(registry, skills, name, header, ...) \
  _SKILL_IMPLEMENTATION_I(registry, skills, name, header, _MODULE_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)

/**
 * The macro creates a creator for the skill implementation.
 * See beginning of this file.
 * It has to be part of the implementation file.
 * @param instance The instantiation of the skill system to which this skill belongs.
 * @param skill The name of the skill implementation that can be created.
 */
#define _MAKE_SKILL_IMPLEMENTATION(instance, skill) \
  SkillImplementationCreator<instance, skill> the##skill##SkillImplementation(#skill, skill##Base::getSkillInfo);
