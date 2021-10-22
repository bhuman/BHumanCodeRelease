/**
 * @file GoalPostsPercept.cpp
 *
 * Very simple representation of a seen goal post
 *
 * @author Alpay Yildiray
 */

#include "GoalPostsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <cmath>

void GoalPostsPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GoalPostsPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPostsPercept:postFeet", "drawingOnImage");

  DECLARE_DEBUG_DRAWING("representation:GoalPostsPercept:relativeGoalPost", "drawingOnField");

  for(const GoalPost& gp : goalPosts)
  {
    RECTANGLE("representation:GoalPostsPercept:image",
              gp.positionInImage.x() - gp.thicknessInImage / 2,
              gp.positionInImage.y() - gp.heightInImage,
              gp.positionInImage.x() + gp.thicknessInImage / 2,
              gp.positionInImage.y(),
              4, Drawings::solidPen, ColorRGBA::green);
    CROSS("representation:GoalPostsPercept:image",
          gp.positionInImage.x(),
          gp.positionInImage.y(),
          6, 2, Drawings::solidPen, ColorRGBA::cyan);

    if(gp.baseInImage)
    {
      CROSS("representation:GoalPostsPercept:relativeGoalPost",
            gp.relativePosition.x(),
            gp.relativePosition.y(),
            50, 20, Drawings::solidPen, ColorRGBA::black);

      CROSS("representation:GoalPostsPercept:postFeet",
            gp.positionInImage.x(),
            gp.positionInImage.y(),
            6, 2, Drawings::solidPen, ColorRGBA::red);

      std::string distance = "Distance: " + std::to_string(gp.relativePosition.norm() / 1000) + " m";
      DRAW_TEXT("representation:GoalPostsPercept:postFeet",
        gp.positionInImage.x(),
        (unsigned)gp.positionInImage.y() - 20,
        8, ColorRGBA::red, distance);
    }
  }
}

void GoalPostsPercept::verify() const
{
  for(const GoalPost& gp : goalPosts)
  {
    ASSERT(gp.positionInImage.x() > 0);
    ASSERT(gp.positionInImage.y() > 0);
    ASSERT(gp.heightInImage > 0);
    ASSERT(gp.thicknessInImage > 0);
  }
}
