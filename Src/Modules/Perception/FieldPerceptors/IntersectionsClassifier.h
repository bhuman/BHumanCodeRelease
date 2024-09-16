/**
 * @file IntersectionsClassifier.h
 *
 * This file declares a module that ...
 *
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 */

#pragma once

#include "Framework/Module.h"
#include "ImageProcessing/PatchUtilities.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/FieldPercepts/IntersectionCandidates.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

MODULE(IntersectionsClassifier,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(IntersectionCandidates),
  REQUIRES(RobotPose),
  PROVIDES(IntersectionsPercept),
  LOADS_PARAMETERS(
  {,
    (float) threshold,  /**< threshold value for the confidence value of the neural net. If 0, neural net is not used. */
  }),
});

class IntersectionsClassifier : public IntersectionsClassifierBase
{
public:
  /** Constructor. */
  IntersectionsClassifier();

private:
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theIntersectionsPercept The representation to be updated.
   */
  void update(IntersectionsPercept& theIntersectionsPercept) override;

  /**
   * Validates the intersection type and emplaces the newly found intersection into the IntersectionsPercept.
   * @param intersectionsPercept the IntersectionsPercept the intersection is added to.
   * @param IntersectionCandidates::IntersectionCandidate the classified intersection to add.
   */
  void addIntersection(IntersectionsPercept& intersectionsPercept, IntersectionCandidates::IntersectionCandidate& intersection);

  /** Classifies the intersection candidate with a neural net and returns the predicted type.
   * @param intersection the intersection to be classified.
   * @return False if the neural net predicted the given candidate not to be an intersection. True otherwise.
   */
  bool classifyIntersection(IntersectionCandidates::IntersectionCandidate& intersection);

  /** enforces that horizontal is +90° of vertical */
  void enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const;

  NeuralNetwork::CompiledNN network;
  std::unique_ptr<NeuralNetwork::Model> model;
};
