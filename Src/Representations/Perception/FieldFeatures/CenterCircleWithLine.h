/**
* @file CenterCircleWithLine.h
*
* Declaration of a struct that represents the combination of
* the center circle and the halfway line crossing the circle:
*
*                   _____
*                  *     *      (ugly, I know)
*                 /       \
*    ------------+----+----+------------
*                 \       /
*                  *     *
*                   -----
*
* @author Tim Laue
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
*/


#pragma once

#include "FieldFeature.h"

/**
* @struct CenterCircleWithLine
*
* The struct defines the pose of the center circle in field coordinates relative to the robot.
* The pose is defined as:
*  - translation => position of the center of the center circle
*  - rotation    => the direction of a vector from the center towards the center of the opponent goal
*/
STREAMABLE_WITH_BASE(CenterCircleWithLine, FieldFeature,
{
  void draw() const;
  VERIFY_FIELD_FEATURE;

  CenterCircleWithLine() = default;
  CenterCircleWithLine(const Pose2f& pose) : FieldFeature(pose) {};

  /**
  * Assignment operator for Pose2f objects.
  * Validity and other members are not set
  * @param other A Pose2f object
  * @return A reference to the object after the assignment
    */
  const CenterCircleWithLine& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    return *this;
  };

  /**
  * Returns 1 of the 2 global positions of this feature (in case of isValid == true).
  */
  const Pose2f getGlobalFeaturePosition() const override,
});
