/**
 * @file PerceptRegistration.cpp
 *
 * This file implements some functions declared by
 * the PerceptRegistration representation's subrepresentations.
 *
 * @author Tim Laue
 */

#include "PerceptRegistration.h"
#include "Math/Approx.h"
#include "Math/Geometry.h"

RegisteredLine::RegisteredLine(const Vector2f& perceptStart, const Vector2f& perceptEnd,
                               const Vector2f& modelStart, const Vector2f& modelEnd,
                               const Matrix2f& covariance, bool partOfCenterCircle):
  perceptStart(perceptStart), perceptEnd(perceptEnd),
  covPerceptCenter(covariance),
  modelStart(modelStart), modelEnd(modelEnd), partOfCenterCircle(partOfCenterCircle)
{
  perceptDirection = perceptEnd - perceptStart;
  perceptDirection.normalize();
  perceptCenter = (perceptStart + perceptEnd) * 0.5f;
  parallelToWorldModelXAxis = Approx::isZero(modelStart.y() - modelEnd.y());
  orthogonalProjection = Geometry::getOrthogonalProjectionOfPointOnLine(perceptStart, perceptDirection, Vector2f::Zero());
  measuredAngle = -std::atan2(orthogonalProjection.y(), orthogonalProjection.x());
  measuredAngle = Angle::normalize(measuredAngle + (parallelToWorldModelXAxis ? pi_2 : 0));
  measuredAngleAlternative = Angle::normalize(measuredAngle - pi);
}
