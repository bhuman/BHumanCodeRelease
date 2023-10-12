/**
 * @file LabelImage.h
 *
 * Implementation for a module that describes a set of labeling annotations for an image
 *
 * @author Bernd Poppinga
 */
#pragma once

#include "Math/Eigen.h"
#include <vector>

struct LabelImage
{
  struct Annotation
  {
    float getIou(const Annotation& annotation) const;
    bool operator<(const Annotation& annotation) const;
    bool isInside(const Annotation& annotation) const;
    float getOverlap(const Annotation& annotation) const;
    float getArea() const;
    Vector2f upperLeft;
    Vector2f lowerRight;
    float probability;
    float distance; // mm
  };

  void nonMaximumSuppression(float threshold = 0.7f);
  void bigBoxSuppression();

  std::vector<Annotation> annotations;
};
