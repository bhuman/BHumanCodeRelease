/**
 * @file OpposingKickoffProvider.cpp
 *
 * This file provides the previous opponents kickoffs.
 *
 * @author Moritz Oppermann
 */

#include "OpposingKickoffProvider.h"

MAKE_MODULE(OpposingKickoffProvider);

void OpposingKickoffProvider::update(OpposingKickoff& theOpposingKickoff)
{
  if(theOpposingKickoff.lastVariations.empty())
  {
    // load anticipated variations from cfg file
    InMapFile stream("Behavior/KickOffs/opposingKickoffVariations.cfg");
    if(stream.exists())
    {
      stream >> theOpposingKickoff;
      theOpposingKickoff.kickoffPrediction = getPrediction(theOpposingKickoff.lastVariations);
    }
  }

  if(theExtendedGameState.stateLastFrame == theGameState.opponentKickOff)
  {
    theOpposingKickoff.ballLocation = theFieldBall.recentBallEndPositionOnField();
    theOpposingKickoff.kickoffHappened = true;
  }
  else if(theOpposingKickoff.kickoffHappened)
  {
    theOpposingKickoff.ballLocations.push_back(theOpposingKickoff.ballLocation);
    theOpposingKickoff.kickoffHappened = false;

    // cluster previous ball locations and get each cluster's mean
    theOpposingKickoff.clusterMeans.clear();
    const std::vector<std::vector<Vector2f>> clusters = clustering(theOpposingKickoff.ballLocations);
    for(const std::vector<Vector2f>& cluster : clusters)
      theOpposingKickoff.clusterMeans.push_back(getMean(cluster));

    // determine the kickoff variation of each cluster
    std::vector<OpposingKickoff::KickoffVariation> kickoffVariations({});
    int numOfKicks = 0;
    float kickYSum = 0.f;
    for(const Vector2f& mean : theOpposingKickoff.clusterMeans)
    {
      OpposingKickoff::KickoffVariation kickoffVariation = OpposingKickoff::unknown;

      // classify cluster as nothing
      if(std::abs(mean.x()) < std::abs(dribbleThreshold) && std::abs(mean.y()) < std::abs(dribbleThreshold))
      {
        kickoffVariation = OpposingKickoff::nothing;
        kickoffVariations.push_back(kickoffVariation);
        continue;
      }

      // classify cluster as dribble
      if(mean.x() < dribbleThreshold)
        kickoffVariation = OpposingKickoff::dribble;

      // classify cluster as kick
      if(mean.x() < kickThreshold)
      {
        kickoffVariation = OpposingKickoff::kick;
        numOfKicks++;
        // if ball rolled too far, take intersection with imaginary line approximately at height of midfielders
        float midfielderHeight = (theFieldDimensions.xPosOwnGoalArea - theFieldDimensions.centerCircleRadius) * 0.5f;
        kickYSum += (mean.x() > midfielderHeight ? std::abs(midfielderHeight / (mean.x() / mean.y())) : std::abs(mean.y()));
      }
      kickoffVariations.push_back(kickoffVariation);
    }
    if(numOfKicks > 0)
    {
      theOpposingKickoff.absKickYMean = kickYSum / numOfKicks; // absolute y-coordinate midfielders should walk to
    }
    // midfielders should not walk too far into either direction
    if(theOpposingKickoff.absKickYMean > maxYMean)
      theOpposingKickoff.absKickYMean = maxYMean;
    else if(theOpposingKickoff.absKickYMean < minYMean)
      theOpposingKickoff.absKickYMean = minYMean;
    // get the number of times each kickoff variation was used
    std::array<unsigned, OpposingKickoff::numOfKickoffVariations> numOfVariations{};
    FOREACH_ENUM(OpposingKickoff::KickoffVariation, variation)
    {
      for(size_t i = 0; i < clusters.size(); i++)
      {
        if(kickoffVariations[i] == variation)
        {
          numOfVariations[variation] += static_cast<unsigned>(clusters[i].size());
          if(numOfVariations[variation] > theOpposingKickoff.numOfVariations[variation])
            theOpposingKickoff.lastVariations.insert(theOpposingKickoff.lastVariations.begin(), variation);
        }
      }
    }
    theOpposingKickoff.numOfVariations = numOfVariations;

    theOpposingKickoff.kickoffPrediction = getPrediction(theOpposingKickoff.lastVariations);

    theOpposingKickoff.clusterRadii.clear();
    for(const std::vector<Vector2f>& cluster : clusters)
      theOpposingKickoff.clusterRadii.push_back(getRadius(cluster));
  }
}

std::vector<std::vector<Vector2f>> OpposingKickoffProvider::clustering(const std::vector<Vector2f>& previousBallLocations) const
{
  // every ball location has it's own cluster
  std::vector<std::vector<Vector2f>> clusters;
  for(const Vector2f& previousBallLocation : previousBallLocations)
    clusters.push_back({ previousBallLocation });

  unsigned k = static_cast<unsigned>(clusters.size());
  for(; k > 0; k--)
  {
    // figuring out which two clusters have the smallest maximum distance
    float minDistance = std::numeric_limits<float>::max();
    unsigned minDistanceI = 0;
    unsigned minDistanceJ = 0;
    for(unsigned i = 0; i < clusters.size(); i++)
    {
      for(unsigned j = i + 1; j < clusters.size(); j++)
      {
        const float maxDistance = getMaxDistance(clusters[i], clusters[j]);
        if(maxDistance < minDistance && maxDistance < clusteringThreshold)
        {
          minDistance = maxDistance;
          minDistanceI = i;
          minDistanceJ = j;
        }
      }
    }
    if(minDistanceI == minDistanceJ) // no clusters found
      return clusters;

    // merging the clusters
    std::vector<Vector2f> mergedClusters = clusters[minDistanceI];
    mergedClusters.insert(mergedClusters.end(), clusters[minDistanceJ].begin(), clusters[minDistanceJ].end());
    clusters.erase(clusters.begin() + minDistanceI);
    clusters.erase(clusters.begin() + minDistanceJ - 1);
    clusters.push_back(mergedClusters);
  }
  return clusters;
}

float OpposingKickoffProvider::getMaxDistance(const std::vector<Vector2f>& cluster1, const std::vector<Vector2f>& cluster2) const
{
  float maxDistance = 0;
  for(const Vector2f& location1 : cluster1)
  {
    for(const Vector2f& location2 : cluster2)
    {
      const float distance = (location1 - location2).squaredNorm();
      if(distance > maxDistance)
        maxDistance = distance;
    }
  }
  return std::sqrt(maxDistance);
}

Vector2f OpposingKickoffProvider::getMean(const std::vector<Vector2f>& cluster) const
{
  const int numOfLocations = static_cast<int>(cluster.size());
  Vector2f sumOfVectors(0.f, 0.f);
  for(const Vector2f& location : cluster)
    sumOfVectors += location;
  return sumOfVectors / numOfLocations;
}

float OpposingKickoffProvider::getRadius(const std::vector<Vector2f>& cluster) const
{
  return getMaxDistance(cluster, cluster) / 2;
}

OpposingKickoff::KickoffVariation OpposingKickoffProvider::getPrediction(const std::vector<OpposingKickoff::KickoffVariation>& lastVariations) const
{
  // only the last x balls are considered for the predicted kickoff variation
  unsigned max = 0;
  OpposingKickoff::KickoffVariation maxVariation;
  FOREACH_ENUM(OpposingKickoff::KickoffVariation, variation)
  {
    const unsigned numOfVariation = getNumOfVariation(variation, lastVariations);
    if(numOfVariation > max)
    {
      max = numOfVariation;
      maxVariation = variation;
    }
  }
  return maxVariation;
}

OpposingKickoff::KickoffVariation OpposingKickoffProvider::getMostUsed(const std::array<unsigned, OpposingKickoff::numOfKickoffVariations>& numOfVariations) const
{
  // get the kickoff variation that was used the most by the opponent
  return static_cast<OpposingKickoff::KickoffVariation>(std::distance(numOfVariations.begin(), std::max_element(numOfVariations.begin(), numOfVariations.end(), [](const auto& a, const auto& b) {return a < b; })));
}

unsigned OpposingKickoffProvider::getNumOfVariation(const OpposingKickoff::KickoffVariation& variation, const std::vector<OpposingKickoff::KickoffVariation>& variations) const
{
  unsigned numOfVariation = 0;
  for(unsigned i = 0; i < static_cast<unsigned>(variations.size()) && i < predictionSampleSize; i++)
    if(variations[i] == variation)
      numOfVariation++;
  return numOfVariation;
}
