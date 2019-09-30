/**
 * @file BallSpots.cpp
 * Declaration of a struct that represents spots that might be an indication of a ball.
 * @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
 * @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
 *
 */

#include "BallSpots.h"
#include "Platform/SystemCall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Module/Blackboard.h"
#include <sstream>

void BallSpots::draw() const
{
  DEBUG_DRAWING("representation:BallSpots:image", "drawingOnImage") // Draws the ballspots to the image
  {
    for(const Vector2i& ballSpot : ballSpots)
    {
      CROSS("representation:BallSpots:image", ballSpot.x(), ballSpot.y(), 2, 3, Drawings::solidPen, ColorRGBA::orange);
      CROSS("representation:BallSpots:image", ballSpot.x(), ballSpot.y(), 2, 0, Drawings::solidPen, ColorRGBA::black);
    }
  }

  DEBUG_DRAWING("labeling:BallSpots", "drawingOnImage") // Draws the ballspots to the image
  {
    if(Blackboard::getInstance().exists("CameraInfo") && Blackboard::getInstance().exists("CameraMatrix")
       && Blackboard::getInstance().exists("BallSpecification"))
    {
      const CameraInfo& theCameraInfo = static_cast<CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      const CameraMatrix& theCameraMatrix = static_cast<CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      const BallSpecification& theBallSpecification = static_cast<BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);

      // Collect patch dimensions of all spots
      std::vector<Boundaryi> spots;
      for(const Vector2i& ballSpot : ballSpots)
      {
        const float radius = IISC::getImageBallRadiusByCenter(ballSpot.cast<float>(), theCameraInfo, theCameraMatrix, theBallSpecification);
        const int ballArea = (static_cast<int>(radius * 3.5f) + 3) / 4 * 4;
        spots.emplace_back(Rangei(ballSpot.x() - ballArea / 2, ballSpot.x() + ballArea / 2),
                           Rangei(ballSpot.y() - ballArea / 2, ballSpot.y() + ballArea / 2));
      }

      // Create commands for each spot
      for(const Boundaryi& spot : spots)
      {
        std::stringstream action;

        // The clicked one is a ball
        action << "si " << (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower") << " number grayscale region "
               << spot.x.min << " " << spot.y.min << " " << spot.x.max << " " << spot.y.max << " Images/Balls/"
               << (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower");

        // All others are no balls. Ignore spots with centers inside this spot's region
        for(const Boundaryi& other : spots)
          if(&other != &spot &&
             (spot.x.max <= other.x.min || spot.x.min >= other.x.max || spot.y.max <= other.y.min || spot.y.min >= other.y.max))
            action << "\nsi " << (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower") << " number grayscale region "
                   << other.x.min << " " << other.y.min << " " << other.x.max << " " << other.y.max << " Images/NoBalls/"
                   << (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower");

        // The actual drawing commands
        RECTANGLE("labeling:BallSpots", spot.x.min, spot.y.min, spot.x.max - 1, spot.y.max - 1, 1, Drawings::solidPen, ColorRGBA::green);
        SPOT("labeling:BallSpots", spot.x.min, spot.y.min, spot.x.max, spot.y.max, action.str()
             << (SystemCall::getMode() == SystemCall::logFileReplay ? "\nlog forward image" : ""));
      }

      // A separate drawing outside the upper right corner if no spot is the ball
      std::stringstream action;
      for(const Boundaryi& other : spots)
        action << "\nsi " << (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower") << " number grayscale region "
               << other.x.min << " " << other.y.min << " " << other.x.max << " " << other.y.max << " Images/NoBalls/"
               << (theCameraInfo.camera == CameraInfo::upper ? "upper" : "lower");
      RECTANGLE("labeling:BallSpots", theCameraInfo.width, 0, theCameraInfo.width * 11 / 10 - 1, theCameraInfo.width / 10 - 1, 1, Drawings::solidPen, ColorRGBA::red);
      SPOT("labeling:BallSpots", theCameraInfo.width, 0, theCameraInfo.width * 11 / 10, theCameraInfo.width / 10, action.str().c_str() + 1
           << (SystemCall::getMode() == SystemCall::logFileReplay ? "\nlog forward image" : ""));
    }
  }
}
