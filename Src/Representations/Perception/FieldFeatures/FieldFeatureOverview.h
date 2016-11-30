/**
 * @file FieldFeatureOverview.h
 * Declaration of a struct gives an overview over the percepted fieldFeature
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(FieldFeatureOverview,
{
  ENUM(Feature,
  {,
    PenaltyArea,
    MidCircle,
    MidCorner,
    OuterCorner,
    GoalFeature,
    GoalFrame,
  });

  STREAMABLE_WITH_BASE(FieldFeatureStatus, Pose2f,
  {
    /**
    * Assignment operator for Pose2f objects
    * @param other A Pose2f object
    * @return A reference to the object after the assignment
    */
    FieldFeatureStatus& operator=(const Pose2f & other)
    {
      static_cast<Pose2f&>(*this) = other;
      return *this;
    };
    ,
    (bool) (false) isValid,
    (unsigned) (0) lastSeen,
  });
  ,
  (FieldFeatureStatus) combinedStatus,
  (ENUM_INDEXED_ARRAY(FieldFeatureStatus, Feature)) statuses,
});
