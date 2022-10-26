/**
 * @file PenaltyMarkClassifier.cpp
 *
 * This file implements a module that detects penalty marks in images with a neural network.
 *
 * @author Simon Werner
 * @author Finn Ewers
 * @author Thomas Röfer
 */

#include "PenaltyMarkClassifier.h"
#include "Platform/File.h"
#include "Tools/Math/InImageSizeCalculations.h"
#include "Debugging/Stopwatch.h"

MAKE_MODULE(PenaltyMarkClassifier, perception);

PenaltyMarkClassifier::PenaltyMarkClassifier() : network(&Global::getAsmjitRuntime())
{
  model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir())
    + "/Config/NeuralNets/PenaltyMarkClassifier/penalty_model.h5");
  network.compile(*model);
}

void PenaltyMarkClassifier::update(PenaltyMarkPercept& thePenaltyMarkPercept)
{
  thePenaltyMarkPercept.wasSeen = false;

  // Special handling for penalty shootout: the penalty mark is where the ball is.
  if (theGameState.isPenaltyShootout())
  {
    if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 2000
      && theBallModel.estimate.position.squaredNorm() < sqr(700))
    {
      thePenaltyMarkPercept.positionOnField = theBallModel.estimate.position;

      Vector2f inImage;
      if (Transformation::robotToImage(thePenaltyMarkPercept.positionOnField, theCameraMatrix, theCameraInfo, inImage))
        thePenaltyMarkPercept.positionInImage = inImage.cast<int>();

      thePenaltyMarkPercept.wasSeen = true;
      return;
    }
  }

  if(!network.valid() || thePenaltyMarkRegions.regions.empty())
    return;

  for(const Boundaryi& region : thePenaltyMarkRegions.regions)
  {
    STOPWATCH("module:PenaltyMarkClassifier:network") {
      //extract patch
      float width, height;
      const Vector2f center(region.x.getCenter(), region.y.getCenter());
      if (!IISC::calculateImagePenaltyMeasurementsByCenter(center, width, height, theCameraInfo, theCameraMatrix, theFieldDimensions))
        continue;

      const int inputSize = static_cast<int>(std::max(width, height) * 1.7f);
      PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(32, 32), theECImage.grayscaled, network.input(0).data());

      network.apply();

      //do something with prediction
      float first = network.output(0)[0];
      float second = network.output(0)[1];
      float confidence = second - first;

      if (first < second && confidence > threshold)
      {
        Vector2i positionInImage{ center.x(), center.y() }; // todo: estimate actual center
        if (Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(positionInImage),
          theCameraMatrix, theCameraInfo, thePenaltyMarkPercept.positionOnField)) {
          thePenaltyMarkPercept.positionInImage = positionInImage;
          thePenaltyMarkPercept.wasSeen = true;
        }
        break;
      }
    }
  }
}
