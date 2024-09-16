#include "LabelImage.h"
#include <algorithm>

bool LabelImage::Annotation::isInside(const Annotation& annotation) const
{
  Vector2f lowerDiff = lowerRight - annotation.lowerRight;
  Vector2f upperDiff = upperLeft - annotation.upperLeft;
  return lowerDiff.x() > 0 && lowerDiff.y() > 0 && upperDiff.x() < 0 && upperDiff.y() < 0;
}

bool LabelImage::Annotation::operator<(const Annotation& annotation) const
{
  return this->confidence < annotation.confidence;
}

float LabelImage::Annotation::getArea() const
{
  //TODO times 2 for smaller images
  return (lowerRight.x() - upperLeft.x()) * (lowerRight.y() - upperLeft.y());
}

float LabelImage::Annotation::getOverlap(const Annotation& annotation) const
{
  float xOverlap = std::max(0.f, std::min(lowerRight.x(), annotation.lowerRight.x()) - std::max(upperLeft.x(), annotation.upperLeft.x()));
  float yOverlap = std::max(0.f, std::min(lowerRight.y(), annotation.lowerRight.y()) - std::max(upperLeft.y(), annotation.upperLeft.y()));
  return xOverlap * yOverlap;
}

float LabelImage::Annotation::getIou(const Annotation& annotation) const
{
  float intersection = getOverlap(annotation);
  return intersection / (getArea() + annotation.getArea() - intersection + 1e-6f);
}

void LabelImage::nonMaximumSuppression(float threshold)
{
  std::vector<Annotation> maximumAnnotations;
  for(const Annotation& annotation : annotations)
  {
    bool best = true;
    for(const Annotation& cmp : annotations)
    {
      if((annotation.getIou(cmp) > threshold && annotation < cmp))
      {
        best = false;
        break;
      }
    }
    if(best)
      maximumAnnotations.push_back(annotation);
  }
  annotations = maximumAnnotations;
}

void LabelImage::bigBoxSuppression()
{
  std::vector<Annotation> smallerAnnotations;
  for(const Annotation& annotation : annotations)
  {
    bool best = true;
    for(const Annotation& cmp : annotations)
    {
      if(annotation.isInside(cmp))
      {
        best = false;
        break;
      }
    }
    if(best)
      smallerAnnotations.push_back(annotation);
  }
  annotations = smallerAnnotations;
}
