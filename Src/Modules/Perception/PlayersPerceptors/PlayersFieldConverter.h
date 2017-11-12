#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/PlayersPercepts/PlayersImagePercept.h"
#include "Representations/Perception/PlayersPercepts/PlayersFieldPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"

MODULE(PlayersFieldConverter,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(Image),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PlayersImagePercept),

  PROVIDES(PlayersFieldPercept),
  DEFINES_PARAMETERS(
  {,
    // Orientation Determination parameters:
    (bool)(false) orientationDetermination,
    (bool)(false) cnsForSilhouette,

    // Jersey-Parameter:
    (bool)(true) jerseyDetection,   // If true, the value PlayersPerceptors ownTeam and detectedJersey are ignored
                                    // and calculated by this module.
    (int)(350) jerseyHeight,        // Height of the jersey in mm.

    (int)(4) jerseyScanGridX,       // Sample rate for scanning the jersey in x direction.
    (int)(4) jerseyScanGridY,       // Sample rate for scanning the jersey in y direction.
    (int)(28) jerseyScanRadius,     // Scan radius (size of the square box).

    (float)(2) jerseyDifFactor,     // Minimum Distance between both team.
    (float)(0.5f) minCorrectPixels, // relative minimum count of pixels which has to be in the correct color.

    (int)(70) minSaturationToBeAColor, // minimum saturation value to be classified as colored (non gray).
    (bool)(true) ignoreUncoloredJerseys, // if both teams have colored jerseys and this is true, fieldColors != none are ignored.

    (float)(0.16f) approxNoFeetTeamCalcRobotWidth, // Relative Width of a seen robot exactly at the point where feet are not recognized completely. (sure just approx.)
    (float)(0.75) approxNoFeetTeamCalcInfluence,
  }),
});

/**
 * @class PlayersPerceptor
 * This class finds indicators for players in the image.
 */
class PlayersFieldConverter : public PlayersFieldConverterBase
{
private:

  void update(PlayersFieldPercept& PlayersFieldPercept);
  void detectJersey(const PlayersImagePercept::PlayerInImage& playerInImage, PlayersFieldPercept::PlayerOnField& playerOnField);

  int teamColorsHue[10] =
  {
    (int)(10.f / 12 * 255), // blue
    (int)(3.f / 12 * 255), // red
    (int)(5.5f / 12 * 255), // yellow
    0, // black
    0, // white
    (int)(7.5f / 12 * 255), // green
    (int)(4.5f / 12 * 255), // orange
    (int)(1.5f / 12 * 255), // purple
    (int)(6.f / 12 * 255), // brown
    0, // gray
  };
};
