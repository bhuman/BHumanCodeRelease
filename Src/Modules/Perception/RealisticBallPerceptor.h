/**
 * @file RealisticBallPerceptor.h
 * @author Felix Thielke
 */

#pragma once

#include <cmath>
#include <set>
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/RealisticBallPercepts.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/ImageProcessing/Sobel.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Approx.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"


#define MAX_WINDOW_RADIUS 64

MODULE(RealisticBallPerceptor,
{,
  REQUIRES(BodyContour),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(LinePercept),
  REQUIRES(ColorTable),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  USES(RobotPose),
  PROVIDES(RealisticBallPercepts),
  DEFINES_PARAMETERS(
  {
    ENUM(EdgeDetectMode,
    {,
      sobel,
      sobelNMS,
      edgeDrawing,
    }),
    (EdgeDetectMode)(sobelNMS) edgeDetectMode,
    (bool)(true) cbChannel,
    (bool)(false) crChannel,
    (unsigned char)(24) gradientThreshold,
    (unsigned char)(12) edAnchorThreshold,
    (unsigned char)(2) edAnchorGridSize,
    (bool)(false) randomness,
    (char)(16) windowRadius,
    (unsigned char)(5) windowPadding,
    (unsigned short)(30) expectedPixelsPerCircleLower,
    (unsigned short)(10) expectedPixelsPerCircleUpper,
    (float)(7.5f) minRadiusLower,
    (float)(1) minRadiusUpper,
    (float)(75) maxRadiusLower,
    (float)(20) maxRadiusUpper,
    (unsigned int)(5) circleThreshold,
    (float)(10) mergeRadius,
    (float)(10) mergeDistance,
    (bool)(true) validatePre,
    (bool)(true) validateFieldRadius,
    (bool)(true) validatePost,
    (float)(0.1f) maxGreenThreshold,
    (float)(1) minGreenThreshold,
    (float)(35) expectedRadius,
    (float)(120) minDistance,
    (float)(32) minRadiusOnField,
    (float)(220) maxRadiusOnField,
    (float)(50) minDiffFromLineWidth,
    (float)(200) minDistanceFromLine,
    (float)(200) minDistanceFromPenaltyMark,
  }),
});

class RealisticBallPerceptor : public RealisticBallPerceptorBase
{
private:
  DECLARE_DEBUG_IMAGE(sobelImage);

  void update(RealisticBallPercepts& ballPercepts);

  struct Point
  {
    unsigned short x;
    unsigned short y;

    Point(unsigned short x, unsigned short y) : x(x), y(y) {}

    bool operator<(const Point& rhs) const
    {
      return y < rhs.y ? true : y == rhs.y && x < rhs.x;
    }
  };

  class GradientDirMap
  {
  public:
    struct Pixel
    {
    private:
      const Angle angle225 = 22.5_deg;
      const Angle angle45 = 45_deg;

    public:
      enum class Dir { HORIZONTAL = 0, DIAGONALUP = 1, VERTICAL = 2, DIAGONALDOWN = 3 };

      unsigned char gradient;
      Dir dir;

      inline void fromSobelPx(const Sobel::SobelPixel& sobelPx, unsigned char threshold)
      {
        const unsigned char grad = static_cast<unsigned char>(static_cast<char>(std::abs(sobelPx.x)) + static_cast<char>(std::abs(sobelPx.y)));
        if(grad >= threshold)
        {
          gradient = grad;
          dir = static_cast<Dir>(static_cast<int>((Approx::atan2(sobelPx.y, sobelPx.x) + angle225) / angle45) & 3);
        }
        else
        {
          gradient = 0;
        }
      }

      inline void fromSobelPxSimple(const Sobel::SobelPixel& sobelPx, unsigned char threshold)
      {
        const char xAbs = static_cast<char>(std::abs(sobelPx.x));
        const char yAbs = static_cast<char>(std::abs(sobelPx.y));
        const unsigned char grad = xAbs + yAbs;
        if(grad >= threshold)
        {
          gradient = grad;
          dir = sobelPx.x < sobelPx.y ? Dir::HORIZONTAL : Dir::VERTICAL;
        }
        else
        {
          gradient = 0;
        }
      }

      inline void addSobelPx(const Sobel::SobelPixel& sobelPx, unsigned char threshold)
      {
        const unsigned char grad = static_cast<unsigned char>(static_cast<char>(std::abs(sobelPx.x)) + static_cast<char>(std::abs(sobelPx.y)));
        if(grad >= threshold && grad > gradient)
        {
          gradient = grad;
          dir = static_cast<Dir>(static_cast<int>((Approx::atan2(sobelPx.y, sobelPx.x) + angle225) / angle45) & 3);
        }
      }

      inline void addSobelPxSimple(const Sobel::SobelPixel& sobelPx, unsigned char threshold)
      {
        const char xAbs = static_cast<char>(std::abs(sobelPx.x));
        const char yAbs = static_cast<char>(std::abs(sobelPx.y));
        const unsigned char grad = xAbs + yAbs;
        if(grad >= threshold && grad > gradient)
        {
          gradient = grad;
          dir = sobelPx.x < sobelPx.y ? Dir::HORIZONTAL : Dir::VERTICAL;
        }
      }
    };

  private:
    int width;
    int height;
    Pixel* map;

  public:
    GradientDirMap(unsigned short width, unsigned short height) : width(width), map(new Pixel[width * height]) {}

    ~GradientDirMap()
    {
      delete[] map;
    }

    GradientDirMap(const GradientDirMap& other) = delete;

    GradientDirMap& operator=(const GradientDirMap& other) = delete;

    inline void resize(unsigned short width, unsigned short height)
    {
      if(width * height > this->width * this->height)
      {
        delete[] map;
        map = new Pixel[width * height];
      }
      this->width = width;
      this->height = height;
    }

    inline Pixel& operator()(const Point& p)
    {
      return map[p.y * width + p.x];
    }

    inline Pixel& operator()(int x, int y)
    {
      return map[y * width + x];
    }

    inline Pixel* operator[](int y)
    {
      return &map[y * width];
    }

    inline Pixel* operator&()
    {
      return map;
    }

    inline const Pixel& operator()(const Point& p) const
    {
      return map[p.y * width + p.x];
    }

    inline const Pixel& operator()(int x, int y) const
    {
      return map[y * width + x];
    }

    inline const Pixel* operator[](int y) const
    {
      return &map[y * width];
    }

    inline const Pixel* operator&() const
    {
      return map;
    }
  };

  unsigned short yStart = 0;
  unsigned short rows = 0;
  unsigned int count = 0;
  unsigned short width = 0;
  unsigned short height = 0;

  unsigned short minY[Image::maxResolutionWidth];
  unsigned short maxY[Image::maxResolutionWidth];

  Sobel::Image1D image1D;
  Sobel::SobelImage sobelImage;
  GradientDirMap gdMap;
  std::set<Point> edgels;
  std::vector<unsigned short> points[Image::maxResolutionHeight];
  std::vector<Point> anchors;

  void findEdges();

  void cleanup();

  void buildGradientDirMapFirstChannel();
  void buildGradientDirMapAddChannel();
  void buildSimpleGradientDirMapFirstChannel();
  void buildSimpleGradientDirMapAddChannel();
  void sobel();
  void sobelNMS();
  void edgeDrawing();
  void edGoLeft(const Point& p, unsigned int& length);
  void edGoRight(const Point& p, unsigned int& length);
  void edGoUp(const Point& p, unsigned int& length);
  void edGoDown(const Point& p, unsigned int& length);

  struct Circle
  {
  public:
    Vector2f center;
    float radius;
    unsigned int accum;

    Circle(short x1, short y1, short x2, short y2, short x3, short y3);
  };

  std::array<Vector2s*, MAX_WINDOW_RADIUS * 3> cells;

  /**
  * Fast randomized hough transform algorithm. The algorithm starts at points.yStart.
  *
  * @param srcImage The source image.
  * @param destImage The destination image.
  */
  void fastRHT(std::vector<Circle>& circles, unsigned char windowPadding, unsigned short expectedPixelsPerCircle,
               float minRadius, float maxRadius, unsigned int circleThreshold, float mergeRadius, float mergeDistanceSquared);

  /**
  * Fast hough transform algorithm. The algorithm starts at points.yStart.
  *
  * @param srcImage The source image.
  * @param destImage The destination image.
  */
  void fastHT(std::vector<Circle>& circles, unsigned char windowPadding, float minRadius,
              float maxRadius, unsigned int circleThreshold, float mergeRadius, float mergeDistanceSquared);

  void fastHTPoint(short x, short y, std::vector<Circle>& circles, unsigned char windowPadding, float minRadius,
                   float maxRadius, unsigned int circleThreshold, float mergeRadius, float mergeDistanceSquared);

  bool validatePerceptPreField(const RealisticBallPercept& ballPercept) const;
  bool calculateBallOnField(RealisticBallPercept& ballPercept) const;
  bool validatePerceptPostField(const RealisticBallPercept& ballPercept, float minDistanceFromLineSquared, float centerCircleRadiusSquared) const;

public:
  RealisticBallPerceptor();
};
