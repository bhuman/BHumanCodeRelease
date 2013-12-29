
#pragma once

#include "HelpWidget.h"

class HelpModule : public SimRobot::Module
{
public:
  static SimRobot::Application* application;
  static HelpModule* module;

  HelpModule(SimRobot::Application& application);

private:
  HelpObject helpObject;

  virtual bool compile();
};
