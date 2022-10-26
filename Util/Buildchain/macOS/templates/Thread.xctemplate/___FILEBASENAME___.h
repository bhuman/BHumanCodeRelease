/**
 * @file ___FILENAME___
 *
 * This file declares a thread that <#...#>
 *
 * @author ___FULLUSERNAME___
 */

#pragma once

#include "Framework/FrameExecutionUnit.h"

class ___FILEBASENAME___ : public FrameExecutionUnit
{
public:
  bool beforeFrame() override;
  void beforeModules() override;
  void afterModules() override;
  bool afterFrame() override;
};
