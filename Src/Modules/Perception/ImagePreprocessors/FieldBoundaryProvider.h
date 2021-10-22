/**
 * @file FieldBoundaryProvider.h
 *
 * This file declares a module that calculates the field boundary
 * using a deep neural network (and subsequent line fitting).
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/LeastSquares.h"
#include "Tools/Module/Module.h"
#include <CompiledNN/CompiledNN.h>
#include <memory>

ENUM(FittingMethod,
{,
  Ransac,
  NotRansac,
  NoFitting,
});

MODULE(FieldBoundaryProvider,
{,
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(Odometer),
  REQUIRES(OtherFieldBoundary),
  PROVIDES(FieldBoundary),
  DEFINES_PARAMETERS(
  {,
    (float)(100.f) minDistance, /**< Boundary spots closer than this distance will be ignored (in mm). */
    (FittingMethod)(NoFitting) fittingMethod, /**< Which line fitting should be used? */
    (unsigned)(10) minNumberOfSpots, /**< The minimum number of valid spots to calculate a field boundary. */
    (int)(50) maxNumberOfIterations, /**< Up to how often does RANSAC iterate? */
    (int)(100) maxSquaredError, /**< Limit at which deviations of spots from the boundary saturate (in pixel^2).  */
    (int)(1) spotAbovePenaltyFactor, /**< A spot being above this boundary is this factor worse than being below. */
    (float)(0.1f) acceptanceRatio, /**< Which overall ratio of maxSquaredError is good enough to end the RANSAC? */
    (Angle)(15_deg) randomlyChosenAngleDevThreshold,
    (float)(2.f) threshold, /**< threshold to determine weather the boundary is smooth enough */
    (float)(0.1) nonTopPoints, /**<how many points must not be at the top of the image in relation to the total number of points */
    (int)(4) top, /**< to which pixel points are considered as at the top*/
    (float)(6) uncertaintyLimit, /**< maximum average uncertainty of the non top spots to be not considered as odd */
    (int)(2) maxPointsUnderBorder, /**< how much the points are allowed to be below the lower end on average*/
  }),
});

class FieldBoundaryProvider : public FieldBoundaryProviderBase
{
public:
  /** Constructor. */
  FieldBoundaryProvider();

private:
  /** A boundary spot both in image and field coordinates. */
  struct Spot
  {
    Vector2i inImage; /**< The spot in image coordinates. */
    Vector2f onField; /**< The spot in robot-relative field coordinates. */
    float uncertanty; /**<uncertainty of the network for this spot 0 if the network do not provide it */

    Spot() = default;
    Spot(const Vector2i& inImage, const Vector2f& onField, const float u) : inImage(inImage), onField(onField), uncertanty(u) {}
  };

  struct LineCandidate
  {
    Geometry::Line line;
    std::vector<const Spot*> spots;

    LineCandidate() {}
    LineCandidate(const std::vector<Spot>& s, int start, int end, bool fitOnField)
    {
      for(int i = start; i < end; ++i)  spots.emplace_back(&s[i]);
      fitLine(fitOnField);
    }

    inline void fitLine(bool fitOnField)
    {
      Vector2f n0;
      float d;
      LeastSquares::LineFitter fitter;
      for(const Spot* spot : spots)
        fitOnField ? fitter.add(spot->onField) : fitter.add(spot->inImage.cast<float>());

      VERIFY(fitter.fit(n0, d));
      line.base = n0 * d;
      line.direction = n0;
      line.direction.rotateRight();
    }
  };

  /**
   * This method is called when the representation provided needs to be updated.
   * @param fieldBoundary The representation updated.
   */
  void update(FieldBoundary& fieldBoundary) override;

  void validatePrediction(FieldBoundary& fieldBoundary, std::vector<Spot>& spots);

  /**
   * Predict where the previous field boundary will be in the current image.
   * @param fieldBoundary The field boundary that is updated.
   */
  void projectPrevious(FieldBoundary& fieldBoundary);

  void predictSpots(std::vector<Spot>& spots);

  /**
   * Checks if the calculated boundary spots is odd/not good.
   * @param spots The spots that are validated.
   * @return True if the boundary spots seem odd.
   */
  bool boundaryIsOdd(const std::vector<Spot>& spots) const;

  /**
   * Return a weighted, squared, and saturated error between boundary spots and a
   * boundary line.
   * @param error The vertical pixel offset between spot and line. Positive if the
   *              spot is above the line.
   */
  int effectiveError(int error) const
  {
    return std::min(sqr(error), maxSquaredError) * (error < 0 ? 1 : spotAbovePenaltyFactor);
  }

  /**
   * Calculate the field boundary using the RANSAC approach. The method always constructs a
   * model from three sample points and considers a straight line between the first two points
   * or also a perpendicular line (in field coordinates) to the third point.
   * @param spots The boundary spots that are sampled.
   * @param fieldBoundary The field boundary that is filled.
   */
  void fitBoundaryRansac(const std::vector<Spot>& spots, FieldBoundary& fieldBoundary);

  void fitBoundaryNotRansac(const std::vector<Spot>& spots, FieldBoundary& fieldBoundary);

  std::unique_ptr<NeuralNetwork::Model> model; /** The model of the neural network. */
  NeuralNetwork::CompiledNN network; /**< The compiled neural network. */
  Vector2i patchSize;  /**< The width and height of the neural network input image. */
};
