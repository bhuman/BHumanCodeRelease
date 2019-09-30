#include "FieldFeatureOverviewProvider.h"

void FieldFeatureOverviewProvider::update(FieldFeatureOverview& fieldFeatureOverview)
{
  const FieldFeature* fieldFeatures[FieldFeatureOverview::numOfFeatures];
  fieldFeatures[FieldFeatureOverview::goalFrame] = &theGoalFrame;
  fieldFeatures[FieldFeatureOverview::midCircle] = &theMidCircle;
  fieldFeatures[FieldFeatureOverview::midCorner] = &theMidCorner;
  fieldFeatures[FieldFeatureOverview::outerCorner] = &theOuterCorner;
  fieldFeatures[FieldFeatureOverview::penaltyArea] = &thePenaltyArea;

  fieldFeatureOverview.combinedStatus.isValid = false;
  FOREACH_ENUM(FieldFeatureOverview::Feature, i)
    if((fieldFeatureOverview.statuses[i].isValid = fieldFeatures[i]->isValid) && (fieldFeatureOverview.combinedStatus.isValid = true))
    {
      // This does not overwrite isValid.
      fieldFeatureOverview.statuses[i] = Pose2f(*fieldFeatures[i]);
      fieldFeatureOverview.combinedStatus.lastSeen = fieldFeatureOverview.statuses[i].lastSeen = theFrameInfo.time;
    }

  fieldFeatureOverview.statuses[FieldFeatureOverview::outerCorner].isRightSided = theOuterCorner.isRightCorner;
}

MAKE_MODULE(FieldFeatureOverviewProvider, perception)
