/**
* @file Math/Common.h
*
* This contains some often used mathematical definitions and functions.
*
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
*/

#pragma once

#include <cmath>
#include <cstdlib> // std::abs(int)


#undef isnan
#define isnan(x) bhisnan(x)

inline bool bhisnan(float x)
{
  return (*(unsigned*) &x & 0x7fffffff) == 0x7fc00000;
}

/**
* Returns the sign of a value.
* \param a The value.
* \return The sign of \c a.
*/
template <class V> inline int sgn(const V& a) {return a < 0 ? -1 : ((a == 0) ? 0 : 1);}

/**
* Calculates the square of a value.
* \param a The value.
* \return The square of \c a.
*/
template <class V> inline V sqr(const V& a) {return a * a;}

/**
* Calculates the sec of a value.
* \param a The value.
* \return The sec of \c a.
*/
template <class V> inline V sec(const V& a) {return 1 / cos(a);}

/**
* Calculates the cosec of a value.
* \param a The value.
* \return The cosec of \c a.
*/
template <class V> inline V cosec(const V& a) {return 1 / sin(a);}

/** @name constants for some often used angles */
///@{
/** constant for a half circle*/
const float pi = 3.1415926535897932384626433832795f;
/** constant for a full circle*/
const float pi2 = 2.0f * pi;
/** constant for three quater circle*/
const float pi3_2 = 1.5f * pi;
/** constant for a quarter circle*/
const float pi_2 = 0.5f * pi;
/** constant for a 1 degree*/
const float pi_180 = pi / 180.0f;
/** constant for a 1/8 circle*/
const float pi_4 = pi * 0.25f;
/** constant for a 3/8 circle*/
const float pi3_4 = pi * 0.75f;
/** constant for an expression used by the gaussian function*/
const float sqrt2pi = std::sqrt(2.0f * pi);
/** constant for converting radians to degrees */
const float _180_pi = 180.0f / pi;
///@}

/** constant for one tLaue in ms */
const int tLaue = 8004;
/** constant for one laue in mm */
const float one_Laue = 1075.f;

/** Converts distance from millimeter to laue.
* \param distance distance coded in millimeter
* \return distance coded in laue
*/
inline float toLaue(float distance) {return distance / one_Laue;}

/** Converts distance from laue to millimeter.
* \param distance distance coded in laue
* \return distance coded in millimeter
*/
inline float fromLaue(float distance) {return distance * one_Laue;}

/**
* Converts angle from rad to degrees.
* \param angle code in rad
* \return angle coded in degrees
*/
template <class V> inline V toDegrees(const V& angle) {return angle * V(_180_pi);}

/** Converts angle from degrees to rad.
* \param degrees angle coded in degrees
* \return angle coded in rad
*/
inline float fromDegrees(int degrees) {return float(degrees) * float(pi_180);}

/** Converts angle from degrees to rad.
 * \param degrees angle coded in degrees
 * \return angle coded in rad
 */
template <class V> inline V fromDegrees(const V& degrees) {return degrees * V(pi_180);}

/**
* reduce angle to [-pi..+pi[
* \param data angle coded in rad
* \return normalized angle coded in rad
*/
template <class V> inline V normalize(const V& data)
{
  if(data < V(pi) && data >= -V(pi)) return data;
  V ndata = data - ((int)(data / V(pi2))) * V(pi2);
  if(ndata >= V(pi))
  {
    ndata -= V(pi2);
  }
  else if(ndata < -V(pi))
  {
    ndata += V(pi2);
  }
  return ndata;
}

/**
* Round to the next integer
* @param d A number
* @return The number as integer
*/
inline int roundNumberToInt(float d)
{
  return static_cast<int>(floor(d + 0.5f));
}

/**
* Round to the next integer but keep type
* @param d A number
* @return The number
*/
inline float roundNumber(float d)
{
  return std::floor(d + 0.5f);
}
