/**
 * @file BehaviorOptionRegistry.h
 *
 * This file declares a class to create and store instances of behavior options.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "BehaviorOption.h"

#include "Platform/BHAssert.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "Modules/BehaviorControl/BehaviorControl/Tools/HeadControlMode.h"

#include <vector>

class BehaviorOptionRegistry final
{
public:
  /**
   * Sets the instance pointer and the activation graph.
   * @param activationGraph A pointer to the activation graph.
   */
  BehaviorOptionRegistry(ActivationGraph* activationGraph);
  /** Resets the instance pointer. */
  ~BehaviorOptionRegistry();
  /** Calls modifyParameters on all registered behavior options. */
  void modifyAllParameters();
  /** Calls preProcess on all registered behavior options. */
  void preProcess();
  /** Calls postProcess on all registered behavior options. */
  void postProcess();
  /**
   * Gets an instance of a behavior option with a given name and creates it if it doesn't exist.
   * @tparam ReturnType The type (interface) to which the options are casted.
   * @param name The name of a behavior option.
   * @return A pointer to the instance (nullptr if the behavior option does not exist).
   */
  template<typename ReturnType>
  static ReturnType* getOption(const std::string& name)
  {
    ASSERT(theInstance != nullptr);
    for(auto& option : theInstance->behaviorOptions)
      if(name == option.behaviorOption->name)
      {
        if(option.instance == nullptr)
          option.instance = option.behaviorOption->createNew();
        ReturnType* instance = dynamic_cast<ReturnType*>(option.instance);
        if(instance == nullptr)
          FAIL("Behavior option " << name << " could not be created (or is not derived from correct class)!");
        return instance;
      }
    FAIL("Behavior option " << name << " does not exist!");
    return nullptr;
  }
  /**
   * Gets instances of behavior options with given names and creates them if they don't exist.
   * @tparam ReturnType The type (interface) to which the options are casted.
   * @param names A list of names of behavior options.
   * @return A list of pointers to the instances.
   */
  template<typename ReturnType>
  static std::vector<ReturnType*> getOptions(const std::vector<std::string>& names)
  {
    std::vector<ReturnType*> result;
    for(auto& name : names)
      result.push_back(getOption<ReturnType>(name));
    return result;
  }
  ActivationGraph* theActivationGraph = nullptr; /**< The activation graph that is modified by the behavior. */
  BehaviorStatus theBehaviorStatus; /**< The behavior status that is modified by the behavior. */
  ArmMotionRequest theArmMotionRequest; /**< The arm motion request that is modified by the behavior. */
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that is modified by the behavior. */
  MotionRequest theMotionRequest; /**< The motion request that is modified by the behavior. */
  HeadControl::Mode theHeadControlMode; /**< The head control mode that is modified by the behavior. */
  static thread_local BehaviorOptionRegistry* theInstance; /** The only instance of this class. */
private:
  struct BehaviorOptionState
  {
    BehaviorOptionBase* behaviorOption; /**< The creator for this option. */
    BehaviorOptionInterface* instance = nullptr; /**< The instance of this option (if it exists). */
    /**
     * Constructor.
     * @param behaviorOption The creator for this option.
     */
    BehaviorOptionState(BehaviorOptionBase* behaviorOption) :
      behaviorOption(behaviorOption)
    {
    }
  };
  std::vector<BehaviorOptionState> behaviorOptions; /**< The list of instantiated behavior options. */
};
