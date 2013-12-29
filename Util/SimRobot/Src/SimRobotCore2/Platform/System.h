/**
* @file System.h
* Declaration of class System
* @author Colin Graf
*/

#pragma once

/**
* @class System
* Collection of some basic platform-dependent system functions
*/
class System
{
public:
  /** Returns the current system time in msecs
  * @return the time
  */
  static unsigned int getTime();
};
