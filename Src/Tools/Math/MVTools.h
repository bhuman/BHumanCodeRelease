/**
* @file MVTools.h
* Contains helper functions for Vector_n and Matrix_nxn
*
* @author <a href="mailto:stefanuhrig@gmx.net">Stefan Uhrig</a>
*/
//------------------------------------------------------------------------------

#pragma once
//------------------------------------------------------------------------------
/**
* @class MVException
* Contains information about an excpetion that occured during calculations
*/
class MVException
{
public:
  /// type of exception
  enum ExceptionType
  {
    Unknown,
    DivByPosZero,
    DivByNegZero,
    PosInfValue,
    NegInfValue,
    DetNegative,
    DetCloseToZero
  };

  MVException() : type(Unknown) {}
  MVException(ExceptionType t) : type(t) {}

public:
  /**
  * Returns a description of the exception
  * @return description of exception
  */
  const char* getDescription() const;

public:
  /// type of exception
  ExceptionType type;
};
//------------------------------------------------------------------------------
/**
* Several helper functions that returns minimal and maximal values
* of data types. If you want to use Matrix_nxn and Vector_n with your own
* types you have to add the appropriate overloaded functions here.
* The maximal and minimal values should be set to values so that multiplication
* and division of values in the range [minValue;maxValue] can be represented
* by the datatype.
* If you don't care for under/overflows and under/overflows don't cause a crash
* you can set the maximal and minimal values available for the datatype.
*/
namespace MVTools
{
bool isNearZero(double value);
bool isNearPosZero(double value);
bool isNearNegZero(double value);

bool isNearInf(double value);
bool isNearPosInf(double value);
bool isNearNegInf(double value);

double getMaxExpDouble();
double getMaxPosDouble();
double getMinPosDouble();

bool isNearZero(float value);
bool isNearPosZero(float value);
bool isNearNegZero(float value);

bool isNearInf(float value);
bool isNearPosInf(float value);
bool isNearNegInf(float value);

float getMaxExpFloat();
float getMaxPosFloat();
float getMinPosFloat();

bool isNearZero(int value);
bool isNearPosZero(int value);
bool isNearNegZero(int value);

bool isNearInf(int value);
bool isNearPosInf(int value);
bool isNearNegInf(int value);

bool isNaN(float value);

} // namespace MVTools
//------------------------------------------------------------------------------
