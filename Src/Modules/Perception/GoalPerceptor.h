/**
* @file GoalPerceptor.h
* @ author Michel Bartsch
* @ author Thomas Münder
*/

#pragma once

#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <list>

MODULE(GoalPerceptor,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  REQUIRES(Image),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(ColorTable),
  REQUIRES(FieldBoundary),
  REQUIRES(Odometer),
  PROVIDES_WITH_MODIFY_AND_DRAW(GoalPercept),
  LOADS_PARAMETERS(
  {,
    (int) horizontalSkippingHorizon, /**< How many non-yellow pixels can be skipped while initially scanning along the horizon */
    (int) minWidthHorizon, /**< Min pixel width a goal-post needs on the horizon */
    (float) connectionHorizon, /**< How many of the goal-post-pixels need to be yellow */
    (int) verticalSkippingUnderHorizon, /**< How many non-yellow pixels can be skipped while scanning down from the horizon */
    (int) horizontalSkippingUnderHorizon, /**< How many non-yellow pixels can be skipped while horizontal scanning under the horizon */
    (int) verticalSkippingOverHorizon, /**< How many non-yellow pixels can be skipped while scanning up from the horizon */
    (int) horizontalSkippingOverHorizon, /**< How many non-yellow pixels can be skipped while horizontal scanning over the horizon */
    (float) lowerCameraMinHeight, /**< Min height, relative to the image´s height, a goal-post needs to have in the lower camera */
    (int) lowerCameraMinWidth, /**< Min pixel width a goal-post needs to be in the lower camera */
    (int) underFieldBoundaryOffset, /**< How many pixels above the FieldBoundary a goal-post can be */
    /** All tolerance values are relative to the original real-world size.
        A tolerance of 2 for example means, that a failure of twice the real-world size is 0%,
        no failure is 100% and everything between is linear. */
    (float) constantWidthToleranceFactor, /**< For checking the shape, a goal-post should have nearly the same width at some different y-coordinates */
    (float) relationWidthToHeightToleranceFactor, /**< The relation between a goal-posts width and it´s height */
    (float) expectedWidthToleranceFactor, /**< How well the width fits */
    (float) expectedHeightToleranceFactor, /**< How well the height fits */
    /** When comparing goal-posts to each other, their quality can only get better by one another.
        A goal-post can´t lower another posts quality. */
    (float) distanceToEachOtherToleranceFactor, /**< How well distances between two goal-posts needs to fit for getting some of the Bonus */
    (float) distanceToEachOtherBonusFactor, /**< Bonus for perfect distance between two goal-posts */
    (float) matchingCrossbarsBonusFactor, /**< Bonus for two goal-posts with crossbars matching to each other */
    (float) quality, /**< Min quality value needed for a goal-post to be accepted as real. Will be between 0% and 100% or higher, wenn two posts fit to each other. */
  }),
});

/**
 * @class GoalPerceptor
 */
class GoalPerceptor: public GoalPerceptorBase
{
private:
  struct Spot
  {
  public:
    Spot(int xStart, int xEnd, int y) : start(xStart), end(xEnd), validity(1.0f)
    {
      width = end - start;
      mid = Vector2<int>(start + (width / 2), y);
    }

    bool operator<(const Spot& other)const
    {
      return validity < other.validity;
    }

    int start;
    int end;
    int width;
    std::vector<int> widths;
    Vector2<int> mid;
    Vector2<int> base;
    Vector2<int> top;
    GoalPost::Position leftRight;
    Vector2<> position;
    float validity;
  };

  void update(GoalPercept& percept);

  void findSpots(const int& height);

  void verticalColorScanDown();

  void verticalColorScanUp();

  bool isYellow(const int& x, const int& y);

  void calculatePosition(const int& height);

  void validate();

  void posting(GoalPercept& percept);

  std::list<Spot> spots;

  std::vector<Spot> lastPosts;
};
