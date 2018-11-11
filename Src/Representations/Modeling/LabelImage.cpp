#include "LabelImage.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <numeric>

bool Annotation::operator<(const Annotation& annotation) const
{
  return this->probability < annotation.probability;
}

float Annotation::getArea() const
{
  //TODO times 2 for smaller images
  return (lowerRight.x() - upperLeft.x()) * (lowerRight.y() - upperLeft.y());
}

float Annotation::getOverlap(const Annotation& annotation) const
{
  float xOverlap = std::max(0.f, std::min(lowerRight.x(), annotation.lowerRight.x()) - std::max(upperLeft.x(), annotation.upperLeft.x()));
  float yOverlap = std::max(0.f, std::min(lowerRight.y(), annotation.lowerRight.y()) - std::max(upperLeft.y(), annotation.upperLeft.y()));
  return xOverlap * yOverlap;
}

float Annotation::getIou(const Annotation& annotation) const
{
  float intersection = getOverlap(annotation);
  return intersection / (getArea() + annotation.getArea() - intersection + 1e-6f);
}

float Annotation::getMaxIou(const LabelImage& groundTruth)const
{
  float max_iou = 0;
  for(const Annotation& annotation : groundTruth[labelType])
    max_iou = std::max(max_iou, getIou(annotation));
  return max_iou;
}

void LabelImage::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:LabelData:image", "drawingOnImage");

  if(valid)
  {
    for(unsigned int i = 0; i < annotations.size() ; i++)
    {
      ColorRGBA color;
      switch(annotations.at(i).labelType)
      {
        case Annotation::PenaltyMark:
          color = ColorRGBA::blue;
          break;
        case Annotation::FootFront:
          color = ColorRGBA::orange;
          break;
        case Annotation::FootBack:
          color = ColorRGBA::violet;
          break;
        case Annotation::Ball:
          color = ColorRGBA::yellow;
          break;
        case Annotation::Robot:
          color = ColorRGBA::red;
          break;
        default:
          color = ColorRGBA::white;
          break;
      }

      RECTANGLE("representation:LabelData:image", annotations.at(i).upperLeft.x(), annotations.at(i).upperLeft.y(), annotations.at(i).lowerRight.x(), annotations.at(i).lowerRight.y(),
                2, Drawings::solidPen, color); // TODO fix half size for pictures from bait
    }
  }
}

std::vector<Annotation> LabelImage::getLabels(const Vector2i& center, const Vector2i& size) const
{
  Annotation reference;
  reference.upperLeft = (center.array() - size.array() / 2).matrix().cast<float>();
  reference.lowerRight = (center.array() + size.array() / 2).matrix().cast<float>();
  std::vector<Annotation> ret;
  for(const Annotation& annotation : annotations)
  {
    if(annotation.ignore)
      continue;
    float overlapArea = reference.getOverlap(annotation);
    float overlap = overlapArea / reference.getArea();
    float overlap_ = overlapArea / annotation.getArea();

    if(overlap > 0.25f || overlap_ > 0.8)
      ret.push_back(annotation);
  }
  return ret;
}

std::vector<Annotation> LabelImage::operator[](const Annotation::LabelType& type) const
{
  std::vector<Annotation> ret;
  for(const Annotation& annotation : annotations)
    if(annotation.ignore || annotation.labelType != type)
      continue;
    else
      ret.push_back(annotation);
  return ret;
}

float LabelImage::getRecall(const LabelImage& groundTruth, const Annotation::LabelType& type) const
{
  std::vector<Annotation> y_true = groundTruth[type];
  if(y_true.size() == 0)
    return -1.f;
  float true_positives_count = 0;
  for(const Annotation& annotation : y_true)
    for(const Annotation& prediction : (*this)[type])
      if(annotation.getIou(prediction) > 0.5)
      {
        true_positives_count++;
        break;
      }

  return true_positives_count / static_cast<float>(y_true.size());
}

float LabelImage::getPrecision(const LabelImage& groundTruth, const Annotation::LabelType& type) const
{
  std::vector<Annotation> y_pred = (*this)[type];
  if(y_pred.size() == 0)
    return -1.f;
  float true_positives_count = 0;
  for(const Annotation& prediction : y_pred)
    for(const Annotation& annotation : groundTruth[type])
      if(annotation.getIou(prediction) > 0.5)
      {
        true_positives_count++;
        break;
      }

  return true_positives_count / static_cast<float>(y_pred.size());
}

float LabelImage::getIou(const LabelImage& groundTruth, const Annotation::LabelType& type) const
{
  std::vector<Annotation> y_pred = (*this)[type];
  if(y_pred.size() == 0)
    return -1.f;
  float iou_sum = 0;
  for(const Annotation& prediction : y_pred)
    iou_sum += prediction.getMaxIou(groundTruth);
  return iou_sum / static_cast<float>(y_pred.size());
}

float ImageSet::getMAP(const Annotation::LabelType& type) const
{
  std::vector<Annotation> sortedAnnotations = labelImages[0][type];
  for(const LabelImage& image : labelImages)
  {
    std::vector<Annotation> annotations = image[type];
    sortedAnnotations.insert(sortedAnnotations.end(), annotations.begin(), annotations.end());
  }

  if(sortedAnnotations.size() == 0)
    return -1.f;

  std::sort(sortedAnnotations.begin(), sortedAnnotations.end());

  float positives_count = 0.f;
  for(const Annotation& prediction : sortedAnnotations)
    if(prediction.correct)
      positives_count++;

  if(positives_count == 0)
    return -1.f;

  std::vector<float> precisions;
  float true_positives_count = 0.f;
  float true_positives_count_tmp = 0.f;
  for(unsigned i = 1; i <= sortedAnnotations.size(); i++)
  {
    if(sortedAnnotations[i - 1].correct)
    {
      true_positives_count++;
      true_positives_count_tmp++;
    }

    if(true_positives_count_tmp / positives_count > 0.1f)
    {
      precisions.emplace_back(true_positives_count / static_cast<float>(i));
      true_positives_count_tmp = 0;
    }
  }
  return std::accumulate(precisions.begin(), precisions.end(), 0.f) / precisions.size();
}

bool LabelImage::hasLabel(const Vector2i& center, const Vector2i& size, const Annotation::LabelType type) const
{
  for(const Annotation& annotation : getLabels(center, size))
    if(type == annotation.labelType)
      return true;
  return false;
}

bool LabelImage::hasLabel(const Annotation::LabelType type) const
{
  for(const Annotation& annotation : annotations)
    if(type == annotation.labelType)
      return true;
  return false;
}
