#include "FieldFeatureOverviewProvider.h"

void FieldFeatureOverviewProvider::update(FieldFeatureOverview& fieldFeatureOverview)
{
  const FieldFeature* fieldFeatures[FieldFeatureOverview::numOfFeatures];
  fieldFeatures[FieldFeatureOverview::GoalFeature] = &theGoalFeature;
  fieldFeatures[FieldFeatureOverview::GoalFrame] = &theGoalFrame;
  fieldFeatures[FieldFeatureOverview::MidCircle] = &theMidCircle;
  fieldFeatures[FieldFeatureOverview::MidCorner] = &theMidCorner;
  fieldFeatures[FieldFeatureOverview::OuterCorner] = &theOuterCorner;
  fieldFeatures[FieldFeatureOverview::PenaltyArea] = &thePenaltyArea;

  fieldFeatureOverview.combinedStatus.isValid = false;
  FOREACH_ENUM((FieldFeatureOverview) Feature, i)
    if(((fieldFeatureOverview.statuses[i] = Pose2f(*fieldFeatures[i])).isValid = fieldFeatures[i]->isValid) && (fieldFeatureOverview.combinedStatus.isValid = true))
      fieldFeatureOverview.combinedStatus.lastSeen = fieldFeatureOverview.statuses[i].lastSeen = theFrameInfo.time;
}

MAKE_MODULE(FieldFeatureOverviewProvider, perception)
