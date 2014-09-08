/**
* @file Math/BHMath.h
*
* This contains some often used mathematical definitions and functions.
*
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
*/

#pragma once

/**
* Returns the sign of a value.
* \param a The value.
* \return The sign of \c a.
*/
template <class V> inline V sgn(V a)
{
  return a < 0 ? V(-1) : a > 0 ? V(1) : 0;
}

/**
* Calculates the square of a value.
* \param a The value.
* \return The square of \c a.
*/
template <class V> inline V sqr(const V& a) {return a * a;}

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
/** constant for converting radians to degrees */
const float _180_pi = 180.0f / pi;
///@}

/**
* Converts angle from rad to degrees.
* \param angle code in rad
* \return angle coded in degrees
*/
template <class V> V toDegrees(V angle) {return angle * V(_180_pi);}

/** Converts angle from degrees to rad.
 * \param degrees angle coded in degrees
 * \return angle coded in rad
 */
template <class V> V fromDegrees(V degrees) {return degrees * V(pi_180);}

/** Converts angle from degrees to rad.
 * \param degrees angle coded in degrees
 * \return angle coded in rad
 */
inline float fromDegrees(int degrees) {return fromDegrees((float) degrees);}

/**
* reduce angle to [-pi..+pi[
* \param data angle coded in rad
* \return normalized angle coded in rad
*/
template <class V> inline V normalize(V data)
{
  if(data >= -V(pi) && data < V(pi))
    return data;
  else
  {
    data = data - ((int)(data / V(pi2))) * V(pi2);
    return data >= V(pi) ? data - V(pi2) : data < -V(pi) ? data + V(pi2) : data;
  }
}
