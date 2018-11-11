/**
 * @file FieldFeatureOverview.h
 * Declaration of a struct gives an overview over the percepted fieldFeature.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(FieldFeatureOverview, COMMA public PureBHumanArbitraryMessageParticle<idFieldFeatureOverview>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  ENUM(Feature,
  {,
    penaltyArea,
    midCircle,
    midCorner,
    outerCorner,
    goalFrame,
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
    (bool)(false) isRightSided,  //< just for FieldFeatures with a side (like OuterCorner)
    (unsigned)(0) lastSeen,      //< the timestamp, when this pose was valid
  });
  void draw() const,

  (FieldFeatureStatus) combinedStatus, // Pose will not be set
  (ENUM_INDEXED_ARRAY(FieldFeatureStatus, Feature)) statuses,
});
