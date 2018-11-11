#pragma once

#include <vector>
#include <map>
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

struct LabelImage;

STREAMABLE(Annotation,
{
  bool operator<(const Annotation& annotation) const;
  float getIou(const Annotation& annotation)const;
  float getMaxIou(const LabelImage& groundTruth)const;
  float getOverlap(const Annotation& annotation)const;
  float getArea()const;
  float probability;
  bool correct;
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
  (bool)(false) hidden,
  (bool)(false) blurred,
  (bool)(false) ignore,
});

using stdVectorAnnotations = std::vector<Annotation>;

STREAMABLE(LabelImage,
{
  void draw() const;
  float getRecall(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  float getPrecision(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  float getIou(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  float getMAP(const LabelImage& groundTruth, const Annotation::LabelType& type) const;
  std::vector<Annotation> operator[](const Annotation::LabelType& type) const;
  std::vector<Annotation> getLabels(const Vector2i& center, const Vector2i& size) const;
  bool hasLabel(const Annotation::LabelType type) const;
  bool hasLabel(const Vector2i& center, const Vector2i& size, const Annotation::LabelType type) const,

  (bool)(false) valid,
  (unsigned)(0) frameTime,
  (stdVectorAnnotations) annotations,
});

using stdVectorImages = std::vector<LabelImage>;

STREAMABLE(ImageSet,
{
  float getMAP(const Annotation::LabelType& type) const,
        (std::string) imageSetName,
        (stdVectorImages) labelImages,
});