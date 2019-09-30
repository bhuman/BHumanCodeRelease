#include "LabelImage.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Angle.h"
#include <numeric>

bool LabelImage::Annotation::isInside(const Annotation& annotation)const
{
  Vector2f lower_diff = lowerRight - annotation.lowerRight;
  Vector2f upper_diff = upperLeft - annotation.upperLeft;
  return lower_diff.x() > 0 && lower_diff.y() > 0 && upper_diff.x() < 0 && upper_diff.y() < 0;
}

bool LabelImage::Annotation::operator<(const Annotation& annotation) const
{
  return this->probability < annotation.probability;
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

float LabelImage::Annotation::getMaxIou(const LabelImage& groundTruth)const
{
  float max_iou = 0;
  for(const Annotation& annotation : groundTruth[labelType])
    max_iou = std::max(max_iou, getIou(annotation));
  return max_iou;
}

void LabelImage::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:LabelImage:annotations", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:LabelImage:info", "drawingOnImage");

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
        case Annotation::Goalpost:
          color = ColorRGBA::brown;
          break;
        default:
          color = ColorRGBA::white;
          break;
      }

      RECTANGLE("representation:LabelImage:annotations", annotations.at(i).upperLeft.x(), annotations.at(i).upperLeft.y(), annotations.at(i).lowerRight.x(), annotations.at(i).lowerRight.y(),
                2, Drawings::solidPen, color);
      DRAWTEXT("representation:LabelImage:info", annotations.at(i).upperLeft.x(), annotations.at(i).upperLeft.y() - 2, 10, color, "dist: " << 0.01 * static_cast<float>(static_cast<int>(0.1 * annotations.at(i).distance)) << "m");
      DRAWTEXT("representation:LabelImage:info", annotations.at(i).upperLeft.x(), annotations.at(i).upperLeft.y() - 12, 10, color, "rot: " << toDegrees<float>(annotations.at(i).rotation) << "°");
    }
  }
}

std::vector<LabelImage::Annotation> LabelImage::getLabels(const Vector2i& center, const Vector2i& size) const
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

std::vector<LabelImage::Annotation> LabelImage::operator[](const Annotation::LabelType& type) const
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
  std::vector<Annotation> y_pred = (*this)[type];
  if(numPositves <= 0)
    return -1;
  float true_positives_count = 0;
  for(const Annotation& prediction : y_pred)
    if(prediction.correct)
      true_positives_count++;

  return true_positives_count / static_cast<float>(numPositves);
}

float LabelImage::getPrecision(const LabelImage& groundTruth, const Annotation::LabelType& type) const
{
  std::vector<Annotation> y_pred = (*this)[type];
  if(y_pred.size() == 0)
    return -1;
  float true_positives_count = 0;
  for(const Annotation& prediction : y_pred)
    if(prediction.correct)
      true_positives_count++;

  return true_positives_count / static_cast<float>(y_pred.size());
}

float LabelImage::getIou(const LabelImage& groundTruth, const Annotation::LabelType& type) const
{
  std::vector<Annotation> y_pred = (*this)[type];

  float iou_sum = 0;
  float counter = 0.f;
  for(const Annotation& annotation : y_pred)
  {
    if(!annotation.correct) continue;

    iou_sum += annotation.iou;
    counter++;
  }

  if(counter == 0)
    return -1;
  return iou_sum / counter;
}

float ImageSet::getMAP(const LabelImage::Annotation::LabelType& type) const
{
  std::vector<LabelImage::Annotation> sortedAnnotations;
  int numPositives = 0;
  for(const LabelImage& image : labelImages)
  {
    numPositives += image.numPositves;
    std::vector<LabelImage::Annotation> annotations = image[type];
    sortedAnnotations.insert(sortedAnnotations.end(), annotations.begin(), annotations.end());
  }

  if(numPositives == 0)
    return -1.f;

  if(sortedAnnotations.empty())
    return -1.f;

  std::sort(sortedAnnotations.begin(), sortedAnnotations.end());

  std::vector<float> recalls;
  std::vector<float> precisions;
  float true_positives_count = 0.f;
  for(unsigned i = 1; i <= sortedAnnotations.size(); i++)
  {
    if(sortedAnnotations[sortedAnnotations.size() - i].correct)
      true_positives_count++;

    recalls.push_back(true_positives_count / numPositives);
    precisions.push_back(true_positives_count / i);
  }

  std::vector<float> precision_steps;
  for(float step = 0.f; step <= 1.01f; step += 0.1f)
  {
    unsigned int i = 0;
    for(; recalls[i] < step && i < recalls.size(); i++);
    if(recalls[i] < step)
    {
      precision_steps.push_back(0.f);
      continue;
    }
    float best = 0;
    for(; i < recalls.size(); i++)
      best = std::max(best, precisions[i]);
    precision_steps.push_back(best);
  }

  return std::accumulate(precision_steps.begin(), precision_steps.end(), 0.f) / precision_steps.size();
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

void LabelImage::evaluate(const LabelImage& groundTruth, const Annotation::LabelType& type, float threshold)
{
  for(const Annotation& y_true : groundTruth[type])
  {
    Annotation* best = nullptr;
    for(Annotation& y_pred : annotations)
    {
      if((y_pred.labelType != type && y_pred.labelType != Annotation::LabelType::Robot) || y_pred.getIou(y_true) < threshold) continue;

      if(!best || (best->probability < y_pred.probability))
        best = &y_pred;
    }
    if(best)
    {
      best->correct = true;
      best->iou = best->getIou(y_true);
    }
  }
  numPositves = static_cast<int>(groundTruth[type].size());
  evaluated = true;
}

/*void LabelImage::evaluate_area(const float distance, const bool lower, const CameraInfo cameraInfo, const CameraMatrix cameraMatrix, const LabelImage& groundTruth, const Annotation::LabelType& type, float threshold)
{
  LabelImage groundTruth_area = groundTruth;
  groundTruth_area.annotations.clear();
  for(const Annotation& y_true : groundTruth[type])
  {
    Vector2f point = Vector2f((y_true.lowerRight.x() + y_true.upperLeft.x()) / 2, y_true.lowerRight.y());
    Transformation::imageToRobot((Vector2i)point.cast<int>(), cameraMatrix, cameraInfo, point);
    float true_distance = point.norm() / 1000;

    bool inRange = false;
    if(lower && y_true.lowerRight.y() >= cameraInfo.height - 50)
      inRange = true;
    if(!lower && y_true.lowerRight.y() < cameraInfo.height - 50)
      inRange = true;
    if(lower && distance > true_distance)
      inRange = true;
    if(!lower && distance <= true_distance)
      inRange = true;
    if(inRange)
      groundTruth_area.annotations.push_back(y_true);
  }
  evaluate(groundTruth_area, type, threshold);
}*/
