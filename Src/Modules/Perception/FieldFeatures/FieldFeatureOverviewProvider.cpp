#include "FieldFeatureOverviewProvider.h"

void FieldFeatureOverviewProvider::update(FieldFeatureOverview& fieldFeatureOverview)
{
  const FieldFeature* fieldFeatures[FieldFeatureOverview::numOfFeatures];
  fieldFeatures[FieldFeatureOverview::midCircle] = &theMidCircle;
  fieldFeatures[FieldFeatureOverview::penaltyArea] = &thePenaltyArea;
  fieldFeatures[FieldFeatureOverview::penaltyMarkWithPenaltyAreaLine] = &thePenaltyMarkWithPenaltyAreaLine;

  fieldFeatureOverview.combinedStatus.isValid = false;
  FOREACH_ENUM(FieldFeatureOverview::Feature, i)
    if((fieldFeatureOverview.statuses[i].isValid = fieldFeatures[i]->isValid) && (fieldFeatureOverview.combinedStatus.isValid = true))
    {
      // This does not overwrite isValid.
      fieldFeatureOverview.statuses[i] = Pose2f(*fieldFeatures[i]);
      fieldFeatureOverview.combinedStatus.lastSeen = fieldFeatureOverview.statuses[i].lastSeen = theFrameInfo.time;
    }
}

MAKE_MODULE(FieldFeatureOverviewProvider, perception);
