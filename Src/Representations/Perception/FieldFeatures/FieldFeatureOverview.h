/**
 * @file FieldFeatureOverview.h
 * Declaration of a struct that gives an overview over the perceived field features.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(FieldFeatureOverview,
{
  ENUM(Feature,
  {,
    centerCircleWithLine,
    penaltyMarkWithPenaltyAreaLine,
  });

  STREAMABLE_WITH_BASE(FieldFeatureStatus, Pose2f,
  {
    /**
     * Assignment operator for Pose2f objects.
     * @param other A Pose2f object
     * @return A reference to the object after the assignment
     */
    FieldFeatureStatus& operator=(const Pose2f& other)
    {
      static_cast<Pose2f&>(*this) = other;
      return *this;
    };
    ,
    (bool)(false) isValid,       //< Seen in current Frame
    (unsigned)(0) lastSeen,      //< the timestamp, when this pose was valid
  });

  /** Some plots with statistics about the overview */
  void draw() const,

  (FieldFeatureStatus) combinedStatus, // Pose will not be set
  (ENUM_INDEXED_ARRAY(FieldFeatureStatus, Feature)) statuses,
});
