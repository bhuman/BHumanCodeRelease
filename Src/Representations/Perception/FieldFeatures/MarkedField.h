/**
 * @file MarkedField.h
 * Declaration of a representations that holds all marked field percepts
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"
#include "FieldMarker.h"

/**
* @struct MarkedField
* defines the pose of the marked field in relativ field coords to the robot
* the marked field pose: position => middle of the field; rotation => looking in direction of a goal
*/
STREAMABLE_WITH_BASE(MarkedField, FieldFeature,
{
  void draw() const;//TODO

  /**
  * Assignment operator for Pose2f objects
  * @param other A Pose2f object
  * @return A reference to the object after the assignment
  */
  const MarkedField& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
  * returns 1 of the 2 global position of this feature (in case of isValid == true)
  */
  const Pose2f getGlobalFeaturePosition() const,
});