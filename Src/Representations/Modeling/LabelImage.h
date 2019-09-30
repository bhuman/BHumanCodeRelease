/**
 * @file LabelImage.h
 *
 * Implementation for a module that describes a set of labeling annotations for an image
 *
 * @author Bernd Poppinga
 */
#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <vector>
#include <map>

STREAMABLE(LabelImage,
{
  STREAMABLE(Annotation,
  {
    bool operator<(const Annotation& annotation) const;
    float getIou(const Annotation& annotation) const;
    bool isInside(const Annotation& annotation) const;
    float getMaxIou(const LabelImage& groundTruth) const;
    float getOverlap(const Annotation& annotation) const;
    float getArea()const;
    float probability;
    float mesuredDistance;
    bool correct;
    float iou;
    ENUM(LabelType,
    {,
       PenaltyMark,
       FootFront,
       FootBack,
       Ball,
       Robot,
       Goalpost,
    }),
    (LabelType) labelType,
    (Vector2f) upperLeft,
    (Vector2f) lowerRight,
    (Vector2f) center,
    (float) distance, // mm
    (float) rotation, // radians
    (bool)(false) hidden,
    (bool)(false) blurred,
    (bool)(false) ignore,
  });

  void draw() const;
  float getRecall(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  float getPrecision(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  float getIou(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  float getMAP(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  void nonMaximumSuppression(float threshold = 0.7f);
  void bigBoxSuppression();
  void evaluate(const LabelImage& groundTruth, const Annotation::LabelType& type, float threshold = 0.7f);
  //void evaluate_area(const float distance, const bool lower, const CameraInfo cameraInfo, const CameraMatrix cameraMatrix, const LabelImage& groundTruth, const Annotation::LabelType& type, float threshold = 0.7f);

  std::vector<LabelImage::Annotation> operator[](const Annotation::LabelType& type) const;
  std::vector<LabelImage::Annotation> getLabels(const Vector2i& center, const Vector2i& size) const;
  bool hasLabel(const Annotation::LabelType type) const;
  std::string name;
  bool hasLabel(const Vector2i& center, const Vector2i& size, const Annotation::LabelType type) const,

  (bool)(false) valid,
  (bool)(false) evaluated,
  (int)(-1) numPositves,
  (unsigned)(0) frameTime,
  (std::vector<Annotation>) annotations,
});

STREAMABLE(ImageSet,
{
  float getMAP(const LabelImage::Annotation::LabelType& type) const,
        (std::string) imageSetName,
        (std::vector<LabelImage>) labelImages,
});
