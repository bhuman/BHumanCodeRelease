/**
 * @file CardDetails.h
 *
 * Definition of the card handling scheme.
 * DO NOT INCLUDE THIS FILE DIRECTLY FROM OUTSIDE THIS DIRECTORY!
 *
 * This file provides the ability to specify the requirements of a card.
 * Example:
 *
 * CARD(MyPerfectBehavior,
 * {,
 *   REQUIRES(Role),                          // Has to be updated before
 *   REQUIRES(CameraMatrix),                  // Has to be updated before
 *   USES(RobotPose),                         // Is used, but has not to be updated before
 *   CALLS(PathToTarget),                     // The card can call the PathToTarget skill.
 *   CALLS(WalkToTarget),                     // ...
 *   DEFINES_PARAMETERS(                      // Has parameters that must have an initial value. If LOADS_PARAMETERS is used instead,
 *   {,                                       // they are loaded from a configuration file that has the same name as the card defined, but starts with lowercase letters.
 *     (float)(42.0f) focalLen,               // Has a parameter named focalLen of type float. By default it has the value 42.0f.
 *     (int)(21) resWidth,                    // All attributes are streamed and can be accessed by requesting 'parameters:MyPerfectBehavior'
 *   }),
 * });
 *
 * This block defines a base class MyPerfectBehaviorBase. The card MyPerfectBehavior has to be
 * derived from that class:
 *
 * class MyPerfectBehavior : public MyPerfectBehaviorBase
 * {
 *   bool preconditions() const override;
 *   bool postconditions() const override;
 *   void execute() override;
 * };
 *
 * In the implementation file, the existence of the card has to be announced:
 *
 * MAKE_CARD(MyPerfectBehavior)
 *
 * @author Jesse Richter-Klug
 * based on Module.h by @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/BehaviorControl/Framework/BehaviorContext.h"
#include <vector>

class CardBase;

class CardCreatorBase
{
public:
  struct Info
  {
    std::vector<const char*> requires; /**< The names of representations required by this card. */
    std::vector<const char*> calls; /**< The names of skills called by this card. */
  };

  /**
   * Constructor.
   * @param name The name of the module that can be created by this instance.
   * @param getCardInfo The function that returns the module info.
   * @param next The creator that shall be next in the list.
   */
  CardCreatorBase(const char* name, Info (*getCardInfo)(), CardCreatorBase* next) :
    name(name), getCardInfo(getCardInfo), next(next)
  {}

  /**
   * Adds the requirements of all cards to the info of a module.
   * @param firstCreator The anchor of the creator list.
   * @param info The info to which the requirements are added.
   */
  static void addToModuleInfo(CardCreatorBase* firstCreator, std::vector<ModuleBase::Info>& info);

  /**
   * Collects the list of skills that are called by cards.
   * @param firstCreator The anchor of the creator list.
   * @return A list of skills that are called by cards (there may be duplicates).
   */
  static std::vector<const char*> gatherSkillInfo(CardCreatorBase* firstCreator);

protected:
  /**
   * Abstract method to create an instance of a card.
   * @return A pointer to the new instance.
   */
  virtual CardBase* createNew() = 0;

private:
  const char* name; /**< The name of the card that can be created by this instance. */
  Info (*getCardInfo)(); /**< A function that returns the info of this card. */
  CardCreatorBase* next; /**< The next entry in the list of all cards. */
  friend class CardRegistryBase;
};

template<typename Instance>
class CardCreatorList : public CardCreatorBase
{
public:
  /**
   * Constructor.
   * @param name The name of the card that can be created by this instance.
   * @param getCardInfo The function that returns the info.
   */
  CardCreatorList(const char* name, Info (*getCardInfo)()) :
    CardCreatorBase(name, getCardInfo, first)
  {
    first = this;
  }

  static CardCreatorBase* first; /**< The head of the list of all cards available. */
};

template<typename Instance> CardCreatorBase* CardCreatorList<Instance>::first = nullptr;

template<typename Instance, typename M> class CardCreator : public CardCreatorList<Instance>
{
public:
  /**
   * Constructor.
   * @param name The name of the card that can be created by this instance.
   * @param getModuleInfo The function that returns the info.
   */
  CardCreator(const char* name, CardCreatorBase::Info (*getCardInfo)()) :
      CardCreatorList<Instance>(name, getCardInfo)
  {}

private:
  /**
   * Creates a new instance of this card.
   * @return A pointer to the new instance.
   */
  CardBase* createNew() override
  {
    return static_cast<CardBase*>(new M);
  }
};

#define _CARD_PARAMETERS(x) _MODULE_JOIN(_CARD_PARAMETERS_, x)
#define _CARD_PARAMETERS_REQUIRES(type)
#define _CARD_PARAMETERS_USES(type)
#define _CARD_PARAMETERS_CALLS(type)
#define _CARD_PARAMETERS__MODULE_DEFINES_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;
#define _CARD_PARAMETERS__MODULE_LOADS_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); using NoParameters = Params;

#define _CARD_LOAD(x) _MODULE_JOIN(_CARD_LOAD_, x)
#define _CARD_LOAD_REQUIRES(type)
#define _CARD_LOAD_USES(type)
#define _CARD_LOAD_CALLS(type)
#define _CARD_LOAD__MODULE_DEFINES_PARAMETERS(...)
#define _CARD_LOAD__MODULE_LOADS_PARAMETERS(...) loadModuleParameters(*this, cardName, fileName, "BehaviorControl/");

#define _CARD_DECLARE(x) _MODULE_JOIN(_CARD_DECLARE_, x)
#define _CARD_DECLARE_REQUIRES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type);
#define _CARD_DECLARE_USES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type);
#define _CARD_DECLARE_CALLS(type) public: _CARD_SKILLS_NAMESPACE::type##Skill& the##type##Skill = *_CARD_SKILL_REGISTRY::theInstance->getSkill<_CARD_SKILLS_NAMESPACE::type##Skill>(#type);
#define _CARD_DECLARE__MODULE_DEFINES_PARAMETERS(...)
#define _CARD_DECLARE__MODULE_LOADS_PARAMETERS(...)

#define _CARD_FREE(x) _MODULE_JOIN(_CARD_FREE_, x)
#define _CARD_FREE_REQUIRES(type) Blackboard::getInstance().free(#type);
#define _CARD_FREE_USES(type) Blackboard::getInstance().free(#type);
#define _CARD_FREE_CALLS(type)
#define _CARD_FREE__MODULE_DEFINES_PARAMETERS(...)
#define _CARD_FREE__MODULE_LOADS_PARAMETERS(...)

#define _CARD_INFO(x) _MODULE_JOIN(_CARD_INFO_, x)
#define _CARD_INFO_REQUIRES(type) _info.requires.push_back(#type);
#define _CARD_INFO_USES(type)
#define _CARD_INFO_CALLS(type) _info.calls.push_back(#type);
#define _CARD_INFO__MODULE_DEFINES_PARAMETERS(...)
#define _CARD_INFO__MODULE_LOADS_PARAMETERS(...)

/**
 * Helper for defining the card's base class.
 * @param name The name of the card.
 * @param header Normally, only an opening brace.
 * @param n The number of entries in the third parameter.
 * @param ... The requirements, called skills and parameter definitions.
 */
#define _CARD_I(name, cardType, header, n, ...) _CARD_II(name, cardType, header, n, (_CARD_PARAMETERS, __VA_ARGS__), (_CARD_LOAD, __VA_ARGS__), (_CARD_DECLARE, __VA_ARGS__), (_CARD_FREE, __VA_ARGS__), (_CARD_INFO, __VA_ARGS__), (__VA_ARGS__))

#ifdef __INTELLISENSE__
#define ALLOW_JUMP_TO_SKILL_DEFINITION using namespace _CARD_SKILLS_NAMESPACE;
#else
#define ALLOW_JUMP_TO_SKILL_DEFINITION
#endif

/**
 * Generates the actual code of the card's base class.
 * It creates all the code and fills in data from the requirements and parameters defined.
 */
#define _CARD_II(theName, cardType, header, n, params, load, declare, free, info, tail) \
  namespace theName##Card \
  { \
    _MODULE_ATTR_##n params \
    using Parameters = NoParameters; \
  } \
  class theName; \
  ALLOW_JUMP_TO_SKILL_DEFINITION \
  class theName##Base : public theName##Card::Parameters COMMA protected cardType \
  _STREAM_UNWRAP header; \
  private: \
    using BaseType = theName##Base; \
    void modifyParameters() override \
    { \
      if(sizeof(NoParameters) < sizeof(theName##Card::Parameters)) \
      { \
        Global::getDebugDataTable().updateObject("parameters:" #theName, *this, false); \
        DEBUG_RESPONSE_ONCE("debug data:parameters:" #theName) \
          OUTPUT(idDebugDataResponse, bin, "parameters:" #theName << TypeRegistry::demangle(typeid(theName##Card::Parameters).name()) << *this); \
      } \
    } \
  public: \
    static CardCreatorBase::Info getCardInfo() \
    { \
      CardCreatorBase::Info _info; \
      _MODULE_ATTR_##n info \
      return _info; \
    } \
  private: \
    _MODULE_ATTR_##n declare \
     template<typename Instance, typename M> friend class CardCreator; \
  public: \
    using Parameters = theName##Card::Parameters; \
    theName##Base(const char* fileName = nullptr) : \
      cardType(#theName) \
    { \
      static const char* cardName = #theName; \
      static_cast<void>(cardName); \
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
 * Generate a base class for a card.
 * See beginning of this file.
 * @param cardType The card type of this card.
 * @param registry The skill registry from which to obtain skill handles.
 * @param skills The skill interface namespace to use
 * @param name The name of the card.
 * @param header Normally, only an opening brace.
 * @param ... The requirements, called skills and parameter definitions.
 */
#define _CARD(cardType, registry, skills, name, header, ...) \
  _CARD_I(name, cardType, header, _MODULE_TUPLE_SIZE(__VA_ARGS__), __VA_ARGS__)

/**
 * The macro creates a creator for the card.
 * See beginning of this file.
 * It has to be part of the implementation file.
 * @param instance The instantiation of the card system to which this card belongs.
 * @param card The name of the card that can be created.
 */
#define _MAKE_CARD(instance, card) \
  CardCreator<instance, card> the##card##Card(#card, card##Base::getCardInfo);
