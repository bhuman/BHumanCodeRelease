/**
 * File:   PT2RobotModel.h
 * Author: arne
 *
 * Created on February 11, 2015, 10:14 AM
 */

#pragma once

#include <string>
#include "Representations/Sensing/JointDataPrediction.h"
#include "PT2.h"

/** 
 * A model that describes the robot's joint state based on the PT2 motor model.
 */
class PT2RobotModel
{
public:
  JointAngles joints;

private:
  std::vector<PT2> models; /**<One model for each motor */
  bool initialized = false;

public:
  PT2RobotModel(const std::string& configFile);

  /**
   * Execute the specified joint request on the model.
   */
  void update(const JointAngles& request, const float dt);

  void init(const JointDataPrediction& jointDataPrediction);
};
