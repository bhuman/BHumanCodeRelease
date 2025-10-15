/**
 * @file RefereeGestureClassifier.cpp
 *
 * This file implements a module that classifies referee gestures and provides the result to the RefereeGesture representation.
 *
 * @author Roman Sablotny, Thomas Röfer
 */

#include "RefereeGestureClassifier.h"
#include "Debugging/DebugImages.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "Tools/Math/Projection.h"

MAKE_MODULE(RefereeGestureClassifier);

RefereeGestureClassifier::RefereeGestureClassifier() : networkKickIn(&Global::getAsmjitRuntime()), networkStandby(&Global::getAsmjitRuntime())
{
  // Compile neural networks for kick-in and standby gestures
  networkKickIn.compile(NeuralNetwork::Model(std::string(File::getBHDir()) + "/Config/NeuralNets/RefereeGestureClassifier/kick_in_without_softmax.h5"));
  networkStandby.compile(NeuralNetwork::Model(std::string(File::getBHDir()) + "/Config/NeuralNets/RefereeGestureClassifier/standby_without_softmax.h5"));

  ASSERT(networkKickIn.numOfInputs() == 2);
  ASSERT(networkKickIn.input(0).dims(0) == patchSize);
  ASSERT(networkKickIn.input(0).dims(0) == networkKickIn.input(0).dims(1));
  ASSERT(networkStandby.numOfInputs() == 2);
  ASSERT(networkStandby.input(0).dims(0) == patchSize);
  ASSERT(networkStandby.input(0).dims(1) == patchWidthStandby);
}

void RefereeGestureClassifier::update(RefereeGesture& theRefereeGesture)
{
  DECLARE_DEBUG_RESPONSE("debug images:module:RefereeGestureClassifier:patch");
  if(!theOptionalCameraImage.image.has_value())
  {
    theRefereeGesture.gesture = RefereeGesture::none;
    return;
  }

  // Calculate referee position on field and its offset from the robot
  const Vector2f refereeOnField(theFieldDimensions.xPosHalfwayLine,
                                (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                * (theGameState.leftHandTeam ? 1.f : -1.f));
  const Vector2f refereeOffset = refereeOnField - theRobotPose.translation;
  const float refereeDistance = refereeOffset.norm();
  // Normalize the referee distance for neural network input
  const float normalizedDistance = normalizeDistance(refereeDistance, normMinDist, normMaxDist);

  const CameraImage& theCameraImage = theOptionalCameraImage.image.value();
  // Determine patch size and scaling based on game state and configuration
  // Patch height is always constant, patch width depends on game state
  const int height = static_cast<int>(patchSize);
  const int width = static_cast<int>(theGameState.isKickIn() ? patchSize : patchWidthStandby);
  // Use dynamic scaling for kick-in patches if enabled
  const int inHeight = useDynamicScaling && theGameState.isKickIn() ? std::min(static_cast<unsigned>(Projection::getSizeByDistance(theUpperCameraInfo, refereeHeight, refereeDistance)), theCameraImage.height) : height;
  const int inWidth = useDynamicScaling && theGameState.isKickIn() ? std::min(static_cast<unsigned>(Projection::getSizeByDistance(theUpperCameraInfo, refereeHeight, refereeDistance)), theCameraImage.height) : width;
  // Calculate the center of the patch in the image
  const unsigned centerX = theCameraImage.width;
  const unsigned centerY = (inHeight & 1 ? inHeight + 1 : inHeight) / 2;

  YUVImage img;
  STOPWATCH("module:RefereeGestureClassifier:extractPatch") PatchUtilities::extractPatch(Vector2i(centerX, centerY), Vector2i(inWidth, inHeight), Vector2i(width, height), theCameraImage, img);
  SEND_DEBUG_IMAGE("module:RefereeGestureClassifier:patch", img);

  NeuralNetwork::CompiledNN& network = theGameState.isKickIn() ? networkKickIn : networkStandby;
  STOPWATCH("module:RefereeGestureClassifier:copyPatchToinput") copyPatchToInput(img, network.input(0).data());

  *network.input(1).data() = normalizedDistance;

  STOPWATCH("module:RefereeGestureClassifier:applyNetwork") network.apply();

  // Prepare raw output for softmax, use third output only for kick-in
  Vector3f rawNetworkOutput {network.output(0)[0], network.output(0)[1], theGameState.isKickIn() ? network.output(0)[2] : 0.f};
  // Apply softmax to network output and store results in RefereeGesture
  // Softmax is applied manually, because CompiledNN's implementation is incomplete
  softmaxNetworkOutput(rawNetworkOutput, theGameState.isKickIn(), theRefereeGesture);
  theRefereeGesture.normDist = normalizedDistance;

  theRefereeGesture.gesture = RefereeGesture::none;
  if(theGameState.isKickIn())
  {
    if(theRefereeGesture.netout1 >= netThreshold)
      theRefereeGesture.gesture = RefereeGesture::kickInLeft;
    else if(theRefereeGesture.netout2 >= netThreshold)
      theRefereeGesture.gesture = RefereeGesture::kickInRight;
  }
  else
  {
    if(theRefereeGesture.netout1 >= netThreshold)
      theRefereeGesture.gesture = RefereeGesture::ready;
  }
}

void RefereeGestureClassifier::copyPatchToInput(YUVImage& img, float* input) const
{
  for(PixelTypes::YUVPixel* src = &img(0, 0), * dest = src + img.height * img.width; src < dest; src++)
  {
    *input++ = static_cast<float>(src->y);
    *input++ = static_cast<float>(src->u);
    *input++ = static_cast<float>(src->v);
  }
}

float RefereeGestureClassifier::normalizeDistance(const float distance, const float minVal, const float maxVal) const
{
  return minVal == maxVal ? 0.f : (distance - minVal) / (maxVal - minVal);
}

void RefereeGestureClassifier::softmaxNetworkOutput(const Vector3f& networkOutput, const bool useThirdOutput, RefereeGesture& theRefereeGesture)
{
  float stability_offset = networkOutput.minCoeff() - networkOutput.maxCoeff();

  double exponentialOutput1 = std::exp(networkOutput.x() + stability_offset);
  double exponentialOutput2 = std::exp(networkOutput.y() + stability_offset);
  double exponentialOutput3 = useThirdOutput ? std::exp(networkOutput.z() + stability_offset) : 0.f;
  double sumOfExponentials = exponentialOutput1 + exponentialOutput2 + exponentialOutput3;

  if (exponentialOutput1 >= std::numeric_limits<double>::max())
    exponentialOutput1 = std::numeric_limits<double>::max();
  if (exponentialOutput2 >= std::numeric_limits<double>::max())
    exponentialOutput2 = std::numeric_limits<double>::max();
  if (exponentialOutput3 >= std::numeric_limits<double>::max())
    exponentialOutput3 = std::numeric_limits<double>::max();
  if (sumOfExponentials >= std::numeric_limits<double>::max())
    sumOfExponentials = std::numeric_limits<double>::max();

  theRefereeGesture.netout1 = static_cast<float>(exponentialOutput1 / sumOfExponentials);
  theRefereeGesture.netout2 = static_cast<float>(exponentialOutput2 / sumOfExponentials);
  theRefereeGesture.netout3 = static_cast<float>(exponentialOutput3 / sumOfExponentials);
}
