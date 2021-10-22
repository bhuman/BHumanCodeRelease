#include "LabelImage.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Angle.h"
#include <numeric>

bool LabelImage::Annotation::isInside(const Annotation& annotation)const
{
  Vector2f lowerDiff = lowerRight - annotation.lowerRight;
  Vector2f upperDiff = upperLeft - annotation.upperLeft;
  return lowerDiff.x() > 0 && lowerDiff.y() > 0 && upperDiff.x() < 0 && upperDiff.y() < 0;
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
  float maxIou = 0;
  for(const Annotation& annotation : groundTruth[labelType])
    maxIou = std::max(maxIou, getIou(annotation));
  return maxIou;
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
      DRAW_TEXT("representation:LabelImage:info", annotations.at(i).upperLeft.x(), annotations.at(i).upperLeft.y() - 2, 10, color, "dist: " << 0.01 * static_cast<float>(static_cast<int>(0.1 * annotations.at(i).distance)) << "m");
      DRAW_TEXT("representation:LabelImage:info", annotations.at(i).upperLeft.x(), annotations.at(i).upperLeft.y() - 12, 10, color, "rot: " << toDegrees<float>(annotations.at(i).rotation) << "°");
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
    float overlapRef = overlapArea / reference.getArea();
    float overlapAno = overlapArea / annotation.getArea();

    if(overlapRef > 0.25f || overlapAno > 0.8)
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

float LabelImage::getRecall(const Annotation::LabelType& type) const
{
  std::vector<Annotation> yPred = (*this)[type];
  if(numPositives <= 0)
    return -1;
  float truePositivesCount = 0;
  for(const Annotation& prediction : yPred)
    if(prediction.correct)
      truePositivesCount++;

  return truePositivesCount / static_cast<float>(numPositives);
}

float LabelImage::getPrecision(const Annotation::LabelType& type) const
{
  std::vector<Annotation> yPred = (*this)[type];
  if(yPred.size() == 0)
    return -1;
  float truePositivesCount = 0;
  for(const Annotation& prediction : yPred)
    if(prediction.correct)
      truePositivesCount++;

  return truePositivesCount / static_cast<float>(yPred.size());
}

float LabelImage::getIou(const Annotation::LabelType& type) const
{
  std::vector<Annotation> yPred = (*this)[type];

  float iou_sum = 0;
  float counter = 0.f;
  for(const Annotation& annotation : yPred)
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
    numPositives += image.numPositives;
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
  float truePositivesCount = 0.f;
  for(unsigned i = 1; i <= sortedAnnotations.size(); i++)
  {
    if(sortedAnnotations[sortedAnnotations.size() - i].correct)
      truePositivesCount++;

    recalls.push_back(truePositivesCount / numPositives);
    precisions.push_back(truePositivesCount / i);
  }

  std::vector<float> precisionSteps;
  for(float step = 0.f; step <= 1.01f; step += 0.1f)
  {
    unsigned int i = 0;
    for(; recalls[i] < step && i < recalls.size(); i++);
    if(recalls[i] < step)
    {
      precisionSteps.push_back(0.f);
      continue;
    }
    float best = 0;
    for(; i < recalls.size(); i++)
      best = std::max(best, precisions[i]);
    precisionSteps.push_back(best);
  }

  return std::accumulate(precisionSteps.begin(), precisionSteps.end(), 0.f) / precisionSteps.size();
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
  for(const Annotation& yTrue : groundTruth[type])
  {
    Annotation* best = nullptr;
    for(Annotation& yPred : annotations)
    {
      if((yPred.labelType != type && yPred.labelType != Annotation::LabelType::Robot) || yPred.getIou(yTrue) < threshold) continue;

      if(!best || (best->probability < yPred.probability))
        best = &yPred;
    }
    if(best)
    {
      best->correct = true;
      best->iou = best->getIou(yTrue);
    }
  }
  numPositives = static_cast<int>(groundTruth[type].size());
  evaluated = true;
}

/*void LabelImage::evaluateArea(const float distance, const bool lower, const CameraInfo cameraInfo, const CameraMatrix cameraMatrix, const LabelImage& groundTruth, const Annotation::LabelType& type, float threshold)
{
  LabelImage groundTruthArea = groundTruth;
  groundTruthArea.annotations.clear();
  for(const Annotation& yTrue : groundTruth[type])
  {
    Vector2f point = Vector2f((yTrue.lowerRight.x() + yTrue.upperLeft.x()) / 2, yTrue.lowerRight.y());
    Transformation::imageToRobot((Vector2i)point.cast<int>(), cameraMatrix, cameraInfo, point);
    float trueDistance = point.norm() / 1000;

    bool inRange = false;
    if(lower && yTrue.lowerRight.y() >= cameraInfo.height - 50)
      inRange = true;
    if(!lower && yTrue.lowerRight.y() < cameraInfo.height - 50)
      inRange = true;
    if(lower && distance > trueDistance)
      inRange = true;
    if(!lower && distance <= trueDistance)
      inRange = true;
    if(inRange)
      groundTruthArea.annotations.push_back(yTrue);
  }
  evaluate(groundTruthArea, type, threshold);
}*/
