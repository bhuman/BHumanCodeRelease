/**
 * @file ___FILENAME___
 *
 * This file declares a module that <#...#>
 *
 * @author ___FULLUSERNAME___
 */

#pragma once

#include "Representations/<#path#>/___VARIABLE_representation:identifier___.h"
#include "Tools/Module/Module.h"

MODULE(___FILEBASENAME___,
{,
  PROVIDES(___VARIABLE_representation:identifier___),
});

class ___FILEBASENAME___ : public ___FILEBASENAME___Base
{
  void update(___VARIABLE_representation:identifier___& the___VARIABLE_representation:identifier___);
};
