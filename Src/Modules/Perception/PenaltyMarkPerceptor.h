#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/PenaltyMarkPercept.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/PotentialLineSpots.h"
#include "Representations/Perception/LineSpots.h"
#include <vector>

MODULE(PenaltyMarkPerceptor,
{,
  REQUIRES(Image),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(ScanlineRegionsClipped),
  REQUIRES(ColorTable),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBoundary),
  REQUIRES(LineSpots),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(PotentialLineSpots),
  REQUIRES(BallPercept),
  PROVIDES(PenaltyMarkPercept),
  DEFINES_PARAMETERS(
  {,
    //note: all pixel parameters are for the resolution 320x240.
    //      For other resolutions they need to be scaled.
    (int) (100) expectedSize, /**<The width and height in mm of the penalty mark */
    (int) (3) toleratedSizeDeviation, /**<The percentual devitaion of the found size from the expected, that is tolerated*/
      (int) (8) minNumPixelVertical, /**< The minimum size in pixel that a recognizeable penalty mark can have*/
      (int) (4) minNumPixelHorizontal,
    (int) (7) whiteSpacing, /**<the additional spacing outside the found white area to search for green surrounding*/
    (int) (2) noiseAreaSize,
    (unsigned) (4) numNotGreenMax,
    (unsigned) (1700) minDistToFieldBoundary,
    (unsigned) (2500) maxDetectionDistance,
    (float) (40.0f) maxVariance,/** Maximum y variance in noise area. */
  }),
});

/**
* @class PenaltyMarkPerceptor
* A module that provides a percept for a seen penaltyMark
*/
class PenaltyMarkPerceptor : public PenaltyMarkPerceptorBase
{
private:
  void update(PenaltyMarkPercept& PenaltyMarkPercept);
  /*
   * searches the scan lines for a possible penaltyMark
   */
  void searchPotentialLineSpots(PenaltyMarkPercept& penaltyMark);

  /*
   * checks whether the candidate is a real penalty mark
   */
  bool isPenaltyMark(PenaltyMarkPercept& pp);
  
  bool isFarAwayFromFieldBoundary(PenaltyMarkPercept& pp) const;
  
  bool isTooFarAway(PenaltyMarkPercept& pp) const;
  
  float calcVariance(const std::vector<unsigned char>& data) const;
  
  /**
   * @param[out] yValues The y value of each pixel that was checked
   * @return 
   */
  bool checkIsNoise(unsigned int x, unsigned int y, std::vector<unsigned char>& yValues);

  /**checks if spot is inside recognized ball percept*/
  bool isInsideBall(PenaltyMarkPercept& pp);

  using Scanline = ScanlineRegionsClipped::Scanline;
  using Region = ScanlineRegionsClipped::Region;

  int scaledToleratedSizeDeviation;
  int scaledWhiteSpacing;
};
