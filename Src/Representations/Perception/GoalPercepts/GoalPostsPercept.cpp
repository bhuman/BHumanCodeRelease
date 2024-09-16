/**
 * @file GoalPostsPercept.cpp
 *
 * Very simple representation of a seen goal post
 *
 * @author Alpay Yildiray
 * @author Laurens Schiefelbein
 */

#include "GoalPostsPercept.h"
#include "Debugging/DebugDrawings.h"

void GoalPostsPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GoalPostsPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPostsPercept:goalPostBase", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPostsPercept:relativeGoalPost", "drawingOnField");

  for(const GoalPost& gp : goalPosts)
  {
    LINE("representation:GoalPostsPercept:image",
          gp.positionInImage.x(),
          gp.positionInImage.y(),
          gp.positionInImage.x(),
          gp.positionInImage.y() - 30,
          10, Drawings::solidPen, ColorRGBA::cyan);

    if(gp.baseInImage)
    {
      CROSS("representation:GoalPostsPercept:relativeGoalPost",
            gp.positionOnField.x(),
            gp.positionOnField.y(),
            50, 20, Drawings::solidPen, ColorRGBA::black);

      CROSS("representation:GoalPostsPercept:goalPostBase",
            gp.positionInImage.x(),
            gp.positionInImage.y(),
            6, 3, Drawings::solidPen, ColorRGBA::red);

      std::string distance = "Distance: " + std::to_string(gp.positionOnField.norm() / 1000) + " m";
      DRAW_TEXT("representation:GoalPostsPercept:goalPostBase",
                gp.positionInImage.x(),
                static_cast<unsigned>(gp.positionInImage.y()) - 20,
                10, ColorRGBA::red, distance);
    }
  }
}

void GoalPostsPercept::verify() const
{
#ifndef NDEBUG
  for(const GoalPost& gp : goalPosts)
  {
    ASSERT(std::isfinite(gp.positionInImage.x()));
    ASSERT(std::isfinite(gp.positionInImage.y()));
    ASSERT(std::isfinite(gp.positionOnField.x()));
    ASSERT(std::isfinite(gp.positionOnField.y()));
  }
#endif
}
