/**
* @file WhiteGoalPerceptor2.h
* @author Jonas Stiensmeier
*/

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ScanlineRegions.h"

#include "Representations/Perception/GoalPercept.h"

#include <vector>
#include <list>

MODULE(WhiteGoalPerceptor2,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorTable),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(Image),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(ScanlineRegionsClipped),
  PROVIDES(GoalPercept),
  LOADS_PARAMETERS(
  {,
    (int) angleOffsetInDeg, //Offset of the angle in Degree
    (float) angleValidationOffset, //Offset for the validation of the angle
    (float) brightenEdgeOffset, // Shall the Edges be brightened (for better visability)
    (float) checkBetweenLinesOffset, //Offset how many edge pixels are allowed in the middle of the post
    (float) distanceValidationOffset, //Offset for the validation of two goalposts, should always be greater or equal 1
    (float) heightValidationOffset, //Offset for the Validation of the height of a goalpost
    (float) houghOffset, // Offset for the houghthreshold between 0 and 1
    (float) houghIgnorePostOffset, //Number of Pixels which should be ignored for the post hough (i.e. sobel can't detect edges since a bord is behind the post)
    (float) houghIgnoreBarOffset, //Number of Pixels which should be ignore for the crossbar hough
    (float) maxRegionLengthOffset, // Offset for the maximal Height a Region should have
    (float) maxWidthOffset, // Offset for the maximal allowed Width of a Region
    (float) minimalHeightThreshold, // Offset for number of Pixel of a goalpost should be in the image in percent
    (int) minRadiusOffset, //The least amount of Pixel the radius of a goalpost should include
    (int) minRegionLength, // if the Region is below  this Offset, assume that it isn't part of the Goalpost
    (float) minRegionLengthOffset,  // Offset for the minimal Height a Region should have
    (float) minWidthOffset, // Offset for the minimal allowed Width of a Region
    (int) nearTopThreshold, // How far away from the top of the Image the fieldboundary needs to be to be ignored for the scanheight
    (float) parameterSpaceOffset, // Increase of Intensity for the display of the parameter space
    (float) radiusNMSOffset, // in which area should only one line be found
    (int) regionThreshold, // The maximum number of Pixel a region should have
    (float) scanAreaSizeOffset, // The Offset for the amout of Pixel left and right of the possible Goalpost which should also be scanned
    (float) scanHeightOffset, // Offset for the scanheight between 1 and 0 - 1 is the fieldline and 0 the fieldboundary
    (float) sigmaThreshold, //Threshold for the Noise level
    (int) sobelThreshold, // Threshold for the square of the sobelvalue
    (int) sobelCrossBarThreshold, // Threshold for the square of the sobelvalue at the crossbar
    (bool) useAlternativeLineSearch, //Use another method to filter houghline better used if a lot of houghlines are next to each other
    (bool) useCheckBetweenLines, //Shall the validation check for edges inside the post
    (bool) useHough, //set to true to use hough to detect lines
    (bool) useLinesAsBase, //if true use (line1.x + line2.x) /2 as Basepoint instead of the region
    (bool) useNMS, // set to true to use non maximum suppression on the edgemap
    (bool) useNoise, //if true use a noisecalculator to determine the noise in a goalpost
    (bool) useOtsu, //if true use otsu to calculate edges
    (float) validationGoalPostThreshold, //Threshold for the validation of a single goalpost
    (float) validationOffset, //Offset for the validation if only one side of the goalpost has been seen
    (float) validationThreshold, // Threshold from which value onwards Goals are accepted
    (float) widthValidationOffset, //Offset for the validation of the width
  }),
});

class WhiteGoalPerceptor2 : public WhiteGoalPerceptor2Base
{

  //Creates a Model of the Goal in the Image
  struct GoalModel
  {
    //Height of the inner rectangle of the Goal in Pixel
    int innerHeight = 0;
    //Height of the outer rectangle of the Goal in Pixel
    int outerHeight = 0;
    //Width of the Inner rectangle of the Goal in Pixel
    int innerWidth = 0;
    //Width of the Outer rectangle of the Goal in Pixel
    int outerWidth = 0;
    //Radius in Pixel of a Goalpost
    int radius = 0;
    //angle in degree of a goalpost in the image (might not be vertical since rolling shutter exists)
    float angleInDeg = 0;
  };

  //A Line generate bei hough transformation
  struct Line
  {
    //Start and Endpoint of the line
    int x0 = 0;
    int x1 = 0;
    int y0 = 0;
    int y1 = 0;
    //angle and r of the line in hesse normal form
    int angleInDeg = 0;
    int r = 0;

    //value is the number of intersections in the parameter space
    int value = 0;

    bool operator<(const Line& other)const
    {
      return value < other.value;
    }
  };

  //Possibles GoalPosts after scanning
  struct PossiblePost
  {
    //Constructor
    PossiblePost(Vector2i basePoint) : isCrossBar(false), base(basePoint), modelCreated(false), edgeMapSuccess(false), expectedHoughValue(0), scanAreaSize(0), crossBarPosition(0), threshold(0), angleInDeg(0), value(0.f) {
    }

    bool isCrossBar;

    //base is the horizontal middle and lowest Point of a post
    Vector2i base;

    //Model of the Goal in the Image, Depending on the Position of the Post
    GoalModel model;

    bool modelCreated;

    //data of the edgeMap
    int imageHeight;// post.model.outerHeight;
    int imageWidth;// post.model.radius + scanAreaSize * 2;
    Vector2i imageOrigin;

    //edgemap created by sobel
    std::vector<std::vector<int>> edgeMap;
    bool edgeMapSuccess;

    //Accumulator Array for Hough
    std::vector<std::vector<int>> houghImg;
    int minAngle;
    int rMax;

    //the number of expected intersections of a line of this post in the parameter space
    int expectedHoughValue;
    int scanAreaSize;
    int crossBarPosition;

    std::vector<Line> lines;

    int threshold;

    bool operator<(const PossiblePost& other)const
    {
      return value < other.value;
    }

    int angleInDeg;
    //value if it is a goalpost: typically between 0 and 1
    float value;
  };

  struct PossibleGoal
  {
    //If two Goalposts are seen first is the left and second the right one
    Vector2i baseInImageFirstPost;
    Vector2i baseInImageSecondPost;
    Vector2f baseOnFieldFirstPost;
    Vector2f baseOnFieldSecondPost;
    std::vector<PossiblePost*> posts;
    float value;
  };

  //All Possible Goalposts
  std::list<PossiblePost> possiblePosts;
  std::vector<PossibleGoal> possibleGoals;

  //sin and cos look up table
  std::vector<float> sinLUT;
  std::vector<float> cosLUT;
  //sin and cos look up table with an added square angle
  std::vector<float> sinLUTsA;
  std::vector<float> cosLUTsA;

  //angle of a vertical line in the image
  float angleInRad;

  //Offset for the angle in rad angle+-angleoffset is the area which is scanned
  float angleOffsetInRad;

  void update(GoalPercept& percept);

  //scan all possible Goalpost for a corroborate of the hypothesis
  void scanGoalPosts();

  //find possible Goal Posts by scanning between the fieldboundary and the fieldline
  void findPossibleGoalPosts();

  bool checkRegion(const ScanlineRegions::Region& region, const int& x, const float& minDistanceSquared);

  float calculateCompleteGoalValue(const PossiblePost& post1, const PossiblePost& post2);

  //create the Model of the Goal given  a distance
  void createGoalModel(const float& distance, PossiblePost& post);

  //calculate the EdgeMap to search for contours
  void calculateEdgeMap(PossiblePost& post);

  //create accumulatorarray
  void createAccumulatorArray(PossiblePost& post);

  //fill the accumulator array for the hough
  void fillAccumulatorArray(PossiblePost& post);

  //search for lines in the accumulator array
  void evaluateAccumulatorArray(PossiblePost& post);

  //check if a goal was found in the Image
  void validateGoal(PossibleGoal& goal);

  //Post it to the goalpercept
  void posting(GoalPercept& goalPercept, PossibleGoal& goal);

  //scans the crossbar connecting two Posts
  void scanCrossBar(const PossiblePost& leftPost, const PossiblePost& rightPost, PossiblePost& bar);

  //set the Posts for a Goal
  bool setPosts(PossibleGoal& goal);

  //transform a position in the image to a position on the field with correction
  bool transformImageToField(Vector2i& positionInImage, Vector2f& positionOnField);

  //validates a Goalpost
  void validateGoalPost(PossiblePost& post);

  bool checkBetweenLines(const Line& firstLine, const Line& secondLine, const int& width, const PossiblePost& post);
  bool calculateNoise(const Line& firstLine, const Line& secondLine, const int& width, const PossiblePost& post);

  //creates the LUT for sin and cos
  void createLUT(const float& angleInRad);

  //adds pixel to the hough accumulator array
  void addPixelForHough(PossiblePost& post, const int& x, const int& y);

  //calculates a line in x and y values given certain critera in hesse normal form
  int calculateHoughLineForPost(const int& y, const int& alpha, const int& r, const int& offset);
  //calculates a line in x and y values given certain critera in hesse normal form
  int calculateHoughLineForBar(const int& x, const int& alpha, const int& r, const int& offset);

  //calculates a 90 degree angle with the effect of rolling shutter
  float calculateAngle();

  //faster sobel impelementation
  int sobelFast(const Image::Pixel* pSrc, int sys)
  {
    int sumH = -pSrc[-sys - 1].y - (pSrc[-sys].y << 1) - pSrc[-sys + 1].y
               + pSrc[sys - 1].y + (pSrc[sys].y << 1) + pSrc[sys + 1].y;

    int sumV = -pSrc[-sys - 1].y + pSrc[-sys + 1].y
               - (pSrc[-1].y << 1) + (pSrc[1].y << 1)
               - pSrc[sys - 1].y + pSrc[sys + 1].y;
    sumH = (sumH + 3) >> 3;
    sumV = (sumV + 3) >> 3;

    return sumV * sumV + sumH * sumH;
  }
  
  bool sobel(PossiblePost& post);

  //nonMaximumSuppression for Sobel, this could be improved
  void nonMaximumSuppression(PossiblePost& post);

  void otsuThreshold(PossiblePost& post);

  bool isGreenInSurrounding(const int& x, const int& y);
};