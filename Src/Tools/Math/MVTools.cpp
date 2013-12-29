/**
* @file MVTools.cpp
* Contains helper functions for Vector_n and Matrix_nxn
*
* @author <a href="mailto:stefanuhrig@gmx.net">Stefan Uhrig</a>
*/
//------------------------------------------------------------------------------
#include "MVTools.h"
#include <limits>
#include <cstring>
#include "Common.h"
//------------------------------------------------------------------------------
const char* MVException::getDescription() const
{
  switch(type)
  {
  case DivByPosZero:
    return "Division by (positive) zero";
  case DivByNegZero:
    return "Division by (negative) zero";
  case PosInfValue:
    return "Infinite (positive) value";
  case NegInfValue:
    return "Infinite (negative) value";
  case DetNegative:
    return "Determinant negative";
  case DetCloseToZero:
    return "Determinant close to zero";
  default:
    return "Unknown";
  }
}
//------------------------------------------------------------------------------
namespace MVTools
{
//------------------------------------------------------------------------------
const float maxPosFloat = 1e18f;
const float maxNegFloat = -1e18f;
const float minPosFloat = 1e-18f;
const float minNegFloat = -1e-18f;
const float maxExpFloat = 41.4465f;

const int maxPosInt = 2147483647;
const int maxNegInt = -2147483647;
const int minPosInt = 0;
const int minNegInt = 0;
//------------------------------------------------------------------------------
bool isNearZero(float value)
{
  if((value > minNegFloat) &&
     (value < minPosFloat))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearPosZero(float value)
{
  if((value >= 0.0) &&
     (value < minPosFloat))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearNegZero(float value)
{
  if((value > minNegFloat) &&
     (value <= 0.0))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearInf(float value)
{
  if((value > maxPosFloat) ||
     (value < maxNegFloat))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearPosInf(float value)
{
  if(value > maxPosFloat)
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearNegInf(float value)
{
  if(value < maxNegFloat)
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
float getMaxPosFloat()
{
  return maxPosFloat;
}
//------------------------------------------------------------------------------
float getMinPosFloat()
{
  return minPosFloat;
}
//------------------------------------------------------------------------------
float getMaxExpFloat()
{
  return maxExpFloat;
}
//------------------------------------------------------------------------------
bool isNearZero(int value)
{
  if((value > minNegInt) &&
     (value < minPosInt))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearPosZero(int value)
{
  if((value >= 0.0) &&
     (value < minPosInt))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearNegZero(int value)
{
  if((value > minNegInt) &&
     (value <= 0.0))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearInf(int value)
{
  if((value > maxPosInt) ||
     (value < maxNegInt))
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearPosInf(int value)
{
  if(value > maxPosInt)
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
bool isNearNegInf(int value)
{
  if(value < maxNegInt)
    return true;
  else
    return false;
}
//------------------------------------------------------------------------------
int getMaxPosInt()
{
  return maxPosInt;
}
//------------------------------------------------------------------------------
int getMinPosInt()
{
  return minPosInt;
}
//------------------------------------------------------------------------------
bool isNaN(float value)
{
  return isnan(value) != 0;
}

}
//------------------------------------------------------------------------------
