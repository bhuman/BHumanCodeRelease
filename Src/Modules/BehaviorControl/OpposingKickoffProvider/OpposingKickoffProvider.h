/**
 * @file OpposingKickoffProvider.h
 *
 * This file provides the previous opponents kickoffs.
 *
 * @author Moritz Oppermann
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/OpposingKickoff.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"

MODULE(OpposingKickoffProvider,
{,
  REQUIRES(FieldBall),
  REQUIRES(GameState),
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldDimensions),
  PROVIDES(OpposingKickoff),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) clusteringThreshold, /**< Threshold to determine whether two locations can be in the same cluster. */
    (float)(-40.f) dribbleThreshold, /**< Maximum x-value to be considered a dribble kickoff. */
    (float)(-800.f) kickThreshold, /**< Maximum x-value to be considered a kick kickoff. */
    (unsigned)(3) predictionSampleSize, /**< Number of latest kickoffs the next prediction should take into account. */
    (float)(400) minYMean, /**< Minimum y-value the midfielders can walk to if predicting a kick. */
    (float)(2000) maxYMean, /**< Maximum y-value the midfielders can walk to if predicting a kick. */
  }),
});

class OpposingKickoffProvider : public OpposingKickoffProviderBase
{
public:
  void update(OpposingKickoff& theOpposingKickoff) override;

private:
  /** Clusters the previous ball locations with hierarchical clustering.
   * @param previousBallLocations Locations where the ball went after previous opposing kickoffs.
   * @return A list of clustered ball locations.
   */
  std::vector<std::vector<Vector2f>> clustering(const std::vector<Vector2f>& previousBallLocations) const;

  /** Calculates the biggest distance between two given clusters.
   * @param cluster1 One of the clusters.
   * @param cluster2 The second cluster.
   * @return The maximum distance between two clusters.
   */
  float getMaxDistance(const std::vector<Vector2f>& cluster1, const std::vector<Vector2f>& cluster2) const;

  /** Calculates the mean of a cluster.
   * @param cluster The cluster of field coordinates.
   * @return The mean of the cluster in field coordinates.
   */
  Vector2f getMean(const std::vector<Vector2f>& cluster) const;

  /** Calculates the radius of a cluster.
   * @param cluster The cluster of field coordinates.
   * @return The radius of the cluster.
   */
  float getRadius(const std::vector<Vector2f>& cluster) const;

  /** Predict the next kickoff variation the opponent might use.
   * @param lastVariations List of all previous kickoff variations.
   * @return The predicted kickoff variation.
   */
  OpposingKickoff::KickoffVariation getPrediction(const std::vector<OpposingKickoff::KickoffVariation>& lastVariations) const;

  /** Get the most used kickoff variation.
   * @param numOfVariations Array with the number of times each kickoff variation was used.
   * @return The most used kickoff variation.
   */
  OpposingKickoff::KickoffVariation getMostUsed(const std::array<unsigned, OpposingKickoff::numOfKickoffVariations>& numOfVariations) const;

  /** Get the number of times a specific kickoff variation was used.
   * @param variation The specific variation to get the number of.
   * @param lastVariations List of all previous kickoff variations.
   * @return The number of times the variation was used.
   */
  unsigned getNumOfVariation(const OpposingKickoff::KickoffVariation& variation, const std::vector<OpposingKickoff::KickoffVariation>& lastVariations) const;
};
