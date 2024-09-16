/**
 * @file RobotDetector.cpp
 *
 * This file implements a module that detects SPL robots in an image using a neural network.
 * Most Code is taken from PlayersDeeptector and changes simply adapt the code of Bernd Poppinga to a newly created model
 *
 * @author Kelke van Lessen
 * @author Lukas Malte Monnerjahn
 * @author Fynn Boese
 * @author Bernd Poppinga
 */

#include "CompiledNN/Model.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Stopwatch.h"
#include "ImageProcessing/PatchUtilities.h"
#include "ImageProcessing/Resize.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Platform/File.h"
#include "RobotDetector.h"
#include "Streaming/Global.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(RobotDetector);

RobotDetector::RobotDetector() :
    cnnConvModel(&Global::getAsmjitRuntime()),
    onnxConvModel(&Global::getAsmjitRuntime())
{
  NeuralNetwork::CompilationSettings settings;
  settings.useExpApproxInSigmoid = false;
  settings.useExpApproxInTanh = false;
  NeuralNetworkONNX::CompilationSettings onnxSettings; // onnxSettings are effectively ignored, so no need to fill them
#if defined MACOS && defined __arm64__
  settings.useCoreML = true;
#endif

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    // Load model params
    InMapFile stream(model_config_path);
    ASSERT(stream.exists());
    stream >> networkParameters;

    ASSERT(networkParameters.inputChannels == 1 || networkParameters.inputChannels == 3); // single channel grayscale image or three channel YUV image
    useOnnx = model_path.ends_with(".onnx");

    // Model initialization
    if(useOnnx)
    {
      onnxModel = std::make_unique<NeuralNetworkONNX::Model>(std::string(File::getBHDir()) + model_path);
      initializeModel(onnxModel, onnxConvModel, onnxSettings);
    }
    else
    {
      cnnModel = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir()) + model_path);
      initializeModel(cnnModel, cnnConvModel, settings);
    }
  }
}

template<typename Model, typename ConvModel, typename CompilationSettings>
void RobotDetector::initializeModel(const std::unique_ptr<Model>& model, ConvModel& convModel, const CompilationSettings& settings)
{
  model->setInputUInt8(0); // This converts the uint8 image to floats for the model
  convModel.compile(*model, settings);
  ASSERT(convModel.numOfInputs() == 1);
  ASSERT(convModel.input(0).rank() == 3);
  ASSERT(networkParameters.inputHeight == convModel.input(0).dims(HEIGHT_SHAPE_INDEX));
  ASSERT(networkParameters.inputWidth == convModel.input(0).dims(WIDTH_SHAPE_INDEX));
  inputImageSize = Vector2i(networkParameters.inputWidth, networkParameters.inputHeight);
  ASSERT(networkParameters.inputChannels == convModel.input(0).dims(IMAGE_CHANNELS_INDEX));
  ASSERT(convModel.numOfOutputs() == 1);
  ASSERT(convModel.output(0).rank() == 4);
  ASSERT(networkParameters.outputHeight == convModel.output(0).dims(HEIGHT_SHAPE_INDEX));
  ASSERT(networkParameters.outputWidth == convModel.output(0).dims(WIDTH_SHAPE_INDEX));
  ASSERT(networkParameters.outputAnchors == convModel.output(0).dims(ANCHOR_SHAPE_INDEX));
  ASSERT(networkParameters.paramsPerAnchor == convModel.output(0).dims(BOX_SHAPE_INDEX));
}

void RobotDetector::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
  theObstaclesImagePercept.obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
}

void RobotDetector::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{
  DECLARE_DEBUG_DRAWING("module:RobotDetector:image", "drawingOnImage");

  std::vector<ObstaclesImagePercept::Obstacle>& obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
  obstacles.clear();
  theObstaclesFieldPercept.obstacles.clear();
  const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.clear();

  if(!theFieldBoundary.isValid || !theECImage.grayscaled.width || !theECImage.grayscaled.height || theOptionalImageRequest.sendImage)
    return;

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    extractImageObstaclesFromNetwork(obstacles);
  }
  else
  {
    ASSERT(xyStep < theECImage.grayscaled.width && xyStep > 0 && xyStep < theECImage.grayscaled.height && xyStep > 0);

    std::vector<std::vector<Region>> regions(xyRegions, std::vector<Region>(xyRegions, Region()));
    STOPWATCH("module:RobotDetector:scanImage") scanImage(regions);
    STOPWATCH("module:RobotDetector:classifyRegions") classifyRegions(regions);
    STOPWATCH("module:RobotDetector:discardHomogenAreas") discardHomogeneousAreas(regions);
    STOPWATCH("module:RobotDetector:clusterRegions") dbScan(regions, obstacles);
  }

  mergeObstacles(theObstaclesFieldPercept, obstacles);
}

void RobotDetector::update(ObstaclesPerceptorData& theObstaclesPerceptorData)
{
  theObstaclesPerceptorData.cameraInfo = theCameraInfo;
  theObstaclesPerceptorData.cameraMatrix = theCameraMatrix;
  theObstaclesPerceptorData.imageCoordinateSystem = theImageCoordinateSystem;
}

void RobotDetector::extractImageObstaclesFromNetwork(std::vector<ObstaclesImagePercept::Obstacle>& obstacles)
{
  if(useOnnx ? !onnxConvModel.valid() : !cnnConvModel.valid())
    return;

  LabelImage labelImage;

  if(networkParameters.inputChannels == 1)
    applyGrayscaleNetwork();
  else
    applyColorNetwork();

  if(useOnnx)
    STOPWATCH("module:RobotDetector:boundingBoxes") boundingBoxes(labelImage, onnxConvModel);
  else
    STOPWATCH("module:RobotDetector:boundingBoxes") boundingBoxes(labelImage, cnnConvModel);

  STOPWATCH("module:RobotDetector:nonMaximumSuppression") labelImage.nonMaximumSuppression(nonMaximumSuppressionIoUThreshold);
  STOPWATCH("module:RobotDetector:bigBoxSuppression") labelImage.bigBoxSuppression();

  for(const LabelImage::Annotation& box : labelImage.annotations)
  {
    DRAW_TEXT("module:RobotDetector:image", box.upperLeft.x(), box.upperLeft.y() - 2, 10, ColorRGBA::black, "fallen: " << box.fallen);
    DRAW_TEXT("module:RobotDetector:image", box.upperLeft.x(), box.upperLeft.y() - 12, 10, ColorRGBA::black, "conf: " << box.confidence);
    obstacles.emplace_back();
    obstacles.back().top = static_cast<int>(box.upperLeft.y());
    obstacles.back().bottom = static_cast<int>(box.lowerRight.y());
    obstacles.back().left = static_cast<int>(box.upperLeft.x());
    obstacles.back().right = static_cast<int>(box.lowerRight.x());
    obstacles.back().confidence = box.confidence;
    obstacles.back().fallen = box.fallen;
    obstacles.back().distance = box.distance;
    obstacles.back().bottomFound = true;
  }
}

void RobotDetector::fillGrayscaleThumbnail()
{
  const auto scale = static_cast<unsigned int>(std::round(std::log2(theECImage.grayscaled.width / networkParameters.inputWidth)));
  ASSERT(theECImage.grayscaled.width == static_cast<unsigned>(networkParameters.inputWidth) << scale);
  ASSERT(theECImage.grayscaled.height == static_cast<unsigned>(networkParameters.inputHeight) << scale);

  // Can't shrink directly to NN input because it needs a larger buffer :-(
  STOPWATCH("module:RobotDetector:shrinkY") Resize::shrinkY(scale, theECImage.grayscaled, grayscaleThumbnail);
  ASSERT(networkParameters.inputWidth == grayscaleThumbnail.width);
  ASSERT(networkParameters.inputHeight == grayscaleThumbnail.height);
  SEND_DEBUG_IMAGE("GrayscaleThumbnail", grayscaleThumbnail);
}

void RobotDetector::fillChromaThumbnails()
{
  const auto scale = static_cast<unsigned int>(std::round(std::log2(theECImage.blueChromaticity.width / networkParameters.inputWidth)));
  ASSERT(scale == static_cast<unsigned int>(std::round(std::log2(theECImage.redChromaticity.width / networkParameters.inputWidth))));
  ASSERT(theECImage.blueChromaticity.width == theECImage.redChromaticity.width);
  ASSERT(theECImage.blueChromaticity.width == static_cast<unsigned>(networkParameters.inputWidth) << scale);
  ASSERT(theECImage.blueChromaticity.height == theECImage.redChromaticity.height);
  ASSERT(theECImage.blueChromaticity.height == static_cast<unsigned>(networkParameters.inputHeight) << scale);
  STOPWATCH("module:RobotDetector:shrinkBlueChroma") Resize::shrinkY(scale, theECImage.blueChromaticity, redChromaThumbnail);
  STOPWATCH("module:RobotDetector:shrinkRedChroma") Resize::shrinkY(scale, theECImage.redChromaticity, blueChromaThumbnail);
  ASSERT(networkParameters.inputWidth == redChromaThumbnail.width);
  ASSERT(networkParameters.inputWidth == blueChromaThumbnail.width);
  ASSERT(networkParameters.inputHeight == redChromaThumbnail.height);
  ASSERT(networkParameters.inputHeight == blueChromaThumbnail.height);
  SEND_DEBUG_IMAGE("RedChromaThumbnail", redChromaThumbnail);
  SEND_DEBUG_IMAGE("BlueChromaThumbnail", blueChromaThumbnail);
}

void RobotDetector::applyGrayscaleNetwork()
{
  ASSERT(networkParameters.inputChannels == 1);
  fillGrayscaleThumbnail();

  if(useOnnx)
  {
    // Copy image into input of the model
    std::memcpy(reinterpret_cast<unsigned char*>(onnxConvModel.input(0).data()), grayscaleThumbnail[0], grayscaleThumbnail.width * grayscaleThumbnail.height * sizeof(unsigned char));

    STOPWATCH("module:RobotDetector:normalizeContrast") PatchUtilities::normalizeContrast<unsigned char>(
          reinterpret_cast<unsigned char*>(onnxConvModel.input(0).data()), inputImageSize, 0.02f);
    STOPWATCH("module:RobotDetector:apply") onnxConvModel.apply();
  }
  else
  {
    // Copy image into input of the model
    std::memcpy(reinterpret_cast<unsigned char*>(cnnConvModel.input(0).data()), grayscaleThumbnail[0], grayscaleThumbnail.width * grayscaleThumbnail.height * sizeof(unsigned char));

    STOPWATCH("module:RobotDetector:normalizeContrast") PatchUtilities::normalizeContrast<unsigned char>(
          reinterpret_cast<unsigned char*>(cnnConvModel.input(0).data()), inputImageSize, 0.02f);
    STOPWATCH("module:RobotDetector:apply") cnnConvModel.apply();
  }
}

void RobotDetector::applyColorNetwork()
{
  ASSERT(networkParameters.inputChannels == 3);
  fillGrayscaleThumbnail();
  fillChromaThumbnails();

  PixelTypes::GrayscaledPixel* yPos = grayscaleThumbnail[0];
  PixelTypes::GrayscaledPixel* uPos = redChromaThumbnail[0];
  PixelTypes::GrayscaledPixel* vPos = blueChromaThumbnail[0];
  PixelTypes::GrayscaledPixel* inputPos;
  if(useOnnx)
    inputPos = reinterpret_cast<PixelTypes::GrayscaledPixel*>(onnxConvModel.input(0).data());
  else
    inputPos = reinterpret_cast<PixelTypes::GrayscaledPixel*>(cnnConvModel.input(0).data());

  STOPWATCH("module:RobotDetector:copyYUVToInput")
  {
    for(unsigned int pos = 0; pos < networkParameters.inputWidth * networkParameters.inputHeight; ++pos, ++yPos, ++uPos, ++vPos, inputPos += 3)
    {
      inputPos[0] = yPos[0];
      inputPos[1] = uPos[0];
      inputPos[2] = vPos[0];
    }
  }

  if(useOnnx)
    STOPWATCH("module:RobotDetector:apply") onnxConvModel.apply();
  else
    STOPWATCH("module:RobotDetector:apply") cnnConvModel.apply();
}

template<typename ConvModel>
void RobotDetector::boundingBoxes(LabelImage& labelImage, ConvModel& convModel)
{
  const float objectThreshold = logit(objectThres);
  for(unsigned y = 0; y < networkParameters.outputHeight; ++y)
    for(unsigned x = 0; x < networkParameters.outputWidth; ++x)
      for(unsigned b = 0; b < networkParameters.outputAnchors; ++b)
      {
        // Check confidence objectThreshold for every Anchor, calculate bounding box only if confidence is above objectThreshold
        const size_t offset = (y * networkParameters.outputWidth + x)
                              * networkParameters.outputAnchors
                              * networkParameters.paramsPerAnchor
                              + b * networkParameters.paramsPerAnchor;
        if(convModel.output(0)[offset] > objectThreshold)
        {
          Eigen::Map<Eigen::Vector<float, 5>> pred(convModel.output(0).data() + offset);
          LabelImage::Annotation box = predictionToBoundingBox(pred, y, x, b);
          if(static_cast<float>(theFieldBoundary.getBoundaryY(static_cast<int>((box.lowerRight.x() + box.upperLeft.x()) / 2.f))) > box.lowerRight.y())
            continue;
          labelImage.annotations.emplace_back(box);
        }
      }
}

LabelImage::Annotation RobotDetector::predictionToBoundingBox(Eigen::Map<Eigen::Vector<float, 5>>& pred, unsigned int y, unsigned int x, unsigned int b) const
{
  pred.array() = 1.f / (1.f + (pred * -1).array().exp());
  ASSERT(pred(networkParameters.confidenceIndex) >= 0.f && pred(networkParameters.confidenceIndex) <= 1.f);
  ASSERT(pred(networkParameters.yMidIndex) >= 0.f && pred(networkParameters.yMidIndex) <= 1.f);
  ASSERT(pred(networkParameters.xMidIndex) >= 0.f && pred(networkParameters.xMidIndex) <= 1.f);
  ASSERT(pred(networkParameters.widthIndex) >= 0.f && pred(networkParameters.widthIndex) <= 1.f);
  ASSERT(pred(networkParameters.heightIndex) >= 0.f && pred(networkParameters.heightIndex) <= 1.f);

  // Bounding Box Position
  pred(networkParameters.yMidIndex) = (static_cast<float>(y) + pred(networkParameters.yMidIndex)) /
                                      static_cast<float>(networkParameters.outputHeight) *
                                      static_cast<float>(theCameraInfo.height);
  pred(networkParameters.xMidIndex) = (static_cast<float>(x) + pred(networkParameters.xMidIndex)) /
                                      static_cast<float>(networkParameters.outputWidth) *
                                      static_cast<float>(theCameraInfo.width);

  // Bounding Box Size
  // 0.5 = no change in size, above and below will scale exponentially
  pred(networkParameters.heightIndex) = networkParameters.anchors[b].y()
                                        * std::pow(networkParameters.sizeConversionFactor, 2.f * pred(3) - 1.f)
                                        * static_cast<float>(theCameraInfo.height);
  pred(networkParameters.widthIndex) = networkParameters.anchors[b].x()
                                       * std::pow(networkParameters.sizeConversionFactor, 2.f * pred(4) - 1.f)
                                       * static_cast<float>(theCameraInfo.width);

  LabelImage::Annotation box;
  box.upperLeft = Vector2f(pred(networkParameters.xMidIndex) - pred(networkParameters.widthIndex) / 2.f,
                           pred(networkParameters.yMidIndex) - pred(networkParameters.heightIndex) / 2.f);
  box.lowerRight = Vector2f(pred(networkParameters.xMidIndex) + pred(networkParameters.widthIndex) / 2.f,
                            pred(networkParameters.yMidIndex) + pred(networkParameters.heightIndex) / 2.f);
  box.confidence = pred(networkParameters.confidenceIndex);

  if(networkParameters.predictFallen)
    box.fallen = pred(networkParameters.fallenClassIndex) > fallenThres;
  else
    box.fallen = false;

  box.distance = -1.f;

  return box;
}

void RobotDetector::mergeObstacles(ObstaclesFieldPercept& theObstaclesFieldPercept, std::vector<ObstaclesImagePercept::Obstacle>& obstacles)
{
  bool mergeObstacles = false;
  const Vector2f& robotRotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? pRobotRotationDeviationInStand : pRobotRotationDeviation;
  do
  {
    mergeObstacles = mergeLowerObstacles && theCameraInfo.camera == CameraInfo::lower && !mergeObstacles && obstacles.size() >= 2;
    auto it = obstacles.begin();
    while(it != obstacles.end())
    {
      ObstaclesImagePercept::Obstacle& obstacleInImage = *it;
      ObstaclesFieldPercept::Obstacle obstacleOnField;
      Matrix2f leftCovariance, rightCovariance;
      bool validObstacle =
          theMeasurementCovariance.transformWithCovLegacy(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), 0.f,
                                                          robotRotationDeviation, obstacleOnField.left, leftCovariance) &&
          theMeasurementCovariance.transformWithCovLegacy(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)),
                                                          0.f,
                                                          robotRotationDeviation, obstacleOnField.right, rightCovariance);

      if(validObstacle)
      {
        obstacleOnField.fallen = obstacleInImage.fallen;
        obstacleOnField.type = ObstaclesFieldPercept::unknown;
        obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

        const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
        if(theCameraInfo.camera == CameraInfo::upper)
        {
          RECTANGLE("module:RobotDetector:image", obstacleInImage.left, obstacleInImage.top, obstacleInImage.right, obstacleInImage.bottom, 3,
                    Drawings::solidPen, ColorRGBA::violet);
          if(static_cast<float>(obstacleInImage.bottom) > static_cast<float>(theECImage.grayscaled.height) * 0.9f)
            for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
            {
              const Pose2f inverseOdometryOffset = theOdometryData.inverse() * theOtherOdometryData;
              const Rangea rangeLower((inverseOdometryOffset * incompleteObstacle.right).angle(), (inverseOdometryOffset * incompleteObstacle.left).angle());
              if(range.min <= rangeLower.max && rangeLower.min <= range.max)
                obstacleInImage.bottomFound = false;
            }

          if(!trimObstacles || (validObstacle = trimObstacle(false, obstacleInImage) &&
                                                theMeasurementCovariance.transformWithCovLegacy(
                                                    theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), 0.f,
                                                    robotRotationDeviation, obstacleOnField.left, leftCovariance) &&
                                                theMeasurementCovariance.transformWithCovLegacy(
                                                    theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), 0.f,
                                                    robotRotationDeviation, obstacleOnField.right, rightCovariance) &&
                                                (obstacleOnField.right - obstacleOnField.left).squaredNorm() <= sqr(750.f)))
          {
            if(!obstacleInImage.bottomFound)
            {
              STOPWATCH("module:RobotDetector:detectJersey") theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
              const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
            }
            else if((validObstacle = obstacleInImage.right - obstacleInImage.left < static_cast<int>(theECImage.grayscaled.width / 2)))
            {
              obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;
              STOPWATCH("module:RobotDetector:detectJersey") theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
              obstacleOnField.covariance = (leftCovariance + rightCovariance) / 2;
              theObstaclesFieldPercept.obstacles.emplace_back(obstacleOnField);
            }
          }
        }
        else if((validObstacle = obstacleInImage.top < static_cast<int>(theECImage.grayscaled.height / xyRegions) || static_cast<float>(obstacleInImage.bottom - obstacleInImage.top) / static_cast<float>(theECImage.grayscaled.height) > 0.5f))
        {
          if(!mergeObstacles)
            STOPWATCH("module:RobotDetector:detectJersey")
              theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
          float minWidth = minWidthOnFieldNoMatch;
          bool hasMatch = false;
          for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
          {
            const Pose2f inverseOdometryOffset = theOdometryData.inverse() * theOtherOdometryData;
            const Rangea rangeUpper((inverseOdometryOffset * incompleteObstacle.right).angle(), (inverseOdometryOffset * incompleteObstacle.left).angle());
            if((hasMatch = (range.min <= rangeUpper.max && rangeUpper.min <= range.max)))
            {
              obstacleOnField.type = incompleteObstacle.type;
              minWidth = minWidthOnFieldNoMatch / 2;
              break;
            }
          }
          obstacleOnField.fallen = !hasMatch && (obstacleInImage.right - obstacleInImage.left >= static_cast<int>(theECImage.grayscaled.width / 2) ||
                                                 obstacleInImage.bottom - obstacleInImage.top >= static_cast<int>(theECImage.grayscaled.height / 2));

          RECTANGLE("module:RobotDetector:image", obstacleInImage.left, obstacleInImage.top, obstacleInImage.right, obstacleInImage.bottom, 3, Drawings::solidPen, ColorRGBA::violet);
          if((validObstacle = (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidth)) && !mergeObstacles)
          {
            const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
            obstacleOnField.covariance = (leftCovariance + rightCovariance) / 2;
            theObstaclesFieldPercept.obstacles.emplace_back(obstacleOnField);
          }
          else if(!validObstacle && (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidthOnFieldNoMatch / 2))
            const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
        }
      }
      it = validObstacle ? it + 1 : obstacles.erase(it);
    }

    if(mergeObstacles)
    {
      bool mergedSomething;
      do
      {
        std::vector<bool> merged(obstacles.size(), false);
        std::vector<ObstaclesImagePercept::Obstacle> mergedObstacles;
        mergedSomething = false;
        for(size_t i = 0; i < obstacles.size(); ++i)
        {
          if(merged[i]) continue;
          ObstaclesImagePercept::Obstacle& a = obstacles[i];
          const Rangei obstacleRange = Rangei(a.left, a.right);

          for(size_t j = i + 1; j < obstacles.size(); ++j)
          {
            ObstaclesImagePercept::Obstacle& b = obstacles[j];
            const Rangei otherObstacleRange = Rangei(b.left, b.right);
            if(std::min(obstacleRange.max, otherObstacleRange.max) - std::max(obstacleRange.min, otherObstacleRange.min) > 0)
            {
              a.top = std::min(a.top, b.top);
              a.bottom = std::max(a.bottom, b.bottom);
              a.left = std::min(a.left, b.left);
              a.right = std::max(a.right, b.right);
              mergedObstacles.emplace_back(a);
              merged[i] = true;
              merged[j] = true;
              mergedSomething = true;
            }
          }
          if(!merged[i]) mergedObstacles.emplace_back(a);
        }
        obstacles = mergedObstacles;
      }
      while(mergedSomething);
    }
  }
  while(mergeObstacles);
}

bool RobotDetector::trimObstacle(bool trimHeight, ObstaclesImagePercept::Obstacle& obstacleInImage)
{
  const int minX = std::max(obstacleInImage.left, 0);
  const int minY = theCameraInfo.camera == CameraInfo::lower ? std::max(obstacleInImage.top, 0) : std::max(static_cast<int>(obstacleInImage.top + (obstacleInImage.bottom - obstacleInImage.top) / 2), 0);
  const int maxX = std::min(obstacleInImage.right, static_cast<int>(theECImage.grayscaled.width));
  const int maxY = std::min(obstacleInImage.bottom, static_cast<int>(theECImage.grayscaled.height));

  int stepSize = static_cast<int>(xyStep);
  std::vector<std::vector<int>> limits = {{}, {}};
  for(int y = minY; y < maxY; y += stepSize)
  {
    int offset = minX, step = static_cast<int>(xyStep);
    for(int side = 0; side < 2; ++side, offset = maxX - 1, step *= -1)
    {
      if((side == 0 && minX <= 10) || (side == 1 && maxX >= static_cast<int>(theECImage.grayscaled.width - 10)))
        continue;

      const PixelTypes::GrayscaledPixel* secondSpot = theECImage.grayscaled[y] + offset;
      short firstSpot = *secondSpot;
      std::function<bool(int)> comp = (side == 0) ? static_cast<std::function<bool(int)>>([maxX, stepSize](int a) -> bool {return a < maxX - stepSize;})
                                                  : static_cast<std::function<bool(int)>>([minX, stepSize](int a) -> bool {return a >= minX + stepSize;});
      for(int x = offset; comp(x); x += step, firstSpot = *secondSpot)
      {
        secondSpot += step;
        if(std::abs(*secondSpot - firstSpot) > minContrastDiff / 2)
        {
          limits[side].emplace_back(x - step / 2);
          break;
        }
      }
    }
  }
  std::sort(limits[0].begin(), limits[0].end());
  if(!limits[0].empty()) obstacleInImage.left = limits[0][limits[0].size() / 2];
  std::sort(limits[1].begin(), limits[1].end());
  if(!limits[1].empty()) obstacleInImage.right = limits[1][limits[1].size() / 2];

  if(trimHeight)
  {
    const Rangei yRange = Rangei(minY + (maxY - minY) / 4, maxY - 1);
    int step = stepSize, upperY = -1, lowerY = -1, botCan = obstacleInImage.bottom;
    for(int side = 0; side < 2; ++side, step *= -1)
    {
      bool satRow = false, nonSatRow = false;
      for(int y = (minY + maxY) / 2; yRange.isInside(y) && (!nonSatRow || !satRow); y += step)
      {
        const PixelTypes::GrayscaledPixel* spot = theECImage.saturated[y] + obstacleInImage.left;
        int x = obstacleInImage.left;
        for(; x < obstacleInImage.right && *spot > satThreshold; x += stepSize, spot += stepSize);

        if(x >= obstacleInImage.right)
          satRow = true;
        else
        {
          botCan = y;
          nonSatRow = true;
        }
        if(satRow && nonSatRow)(side == 0 ? lowerY : upperY) = botCan;
      }
    }
    obstacleInImage.bottom = upperY != -1 ? upperY : (lowerY != -1 ? lowerY : obstacleInImage.bottom);
  }
  Vector2f relativePosition;
  Geometry::Circle ball;
  const float radius = Transformation::imageToRobotHorizontalPlane(Vector2f((obstacleInImage.right + obstacleInImage.left) / 2, (obstacleInImage.top + obstacleInImage.bottom) / 2), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
                       && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
  const int minWidthInImage = std::max(minPixel, static_cast<int>(radius));
  return obstacleInImage.right - obstacleInImage.left >= minWidthInImage && obstacleInImage.bottom - obstacleInImage.top >= minWidthInImage &&
         static_cast<float>(obstacleInImage.right - obstacleInImage.left) / static_cast<float>(maxX - minX) >= minBeforeAfterTrimRatio;
}

void RobotDetector::scanImage(std::vector<std::vector<Region>>& regions)
{
  std::vector<std::pair<int, int>> yLimits(theECImage.grayscaled.width / xyStep);
  for(unsigned int x = 0, index = 0; index < theECImage.grayscaled.width / xyStep; x += xyStep, ++index)
  {
    yLimits[index] = std::make_pair(theFieldBoundary.getBoundaryY(static_cast<int>(x)), theBodyContour.getBottom(static_cast<int>(x), theCameraInfo.height));
  }
  const float yRegionsDivisor = static_cast<float>(theECImage.grayscaled.height) / static_cast<float>(xyRegions);
  const float xRegionsDivisor = static_cast<float>(theECImage.grayscaled.width) / static_cast<float>(xyRegions);

  std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> upperRow = {theECImage.grayscaled[0], theECImage.saturated[0]};
  std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> midRow = upperRow;
  std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> lowerRow;
  for(unsigned int y = 0; y < theECImage.grayscaled.height - xyStep; y += xyStep, upperRow = midRow, midRow = lowerRow)
  {
    lowerRow = {theECImage.grayscaled[y + xyStep], theECImage.saturated[y + xyStep]};
    short leftLum = *midRow.first, leftSat = *midRow.second, midLum = leftLum, midSat = leftSat;
    const PixelTypes::GrayscaledPixel* rightLum = midRow.first;
    const PixelTypes::GrayscaledPixel* rightSat = midRow.second;

    for(unsigned int x = 0, index = 0; x < theECImage.grayscaled.width - xyStep; x += xyStep, ++index, leftLum = midLum, midLum = *rightLum, leftSat = midSat, midSat = *rightSat)
    {
      rightLum += xyStep;
      rightSat += xyStep;
      if(yLimits[index].first < static_cast<int>(y - xyStep) && yLimits[index].second > static_cast<int>(y + xyStep))
      {
        bool horizontalChange = (leftSat < satThreshold || *rightSat < satThreshold) && std::abs(*rightLum - leftLum) > minContrastDiff;
        bool verticalChange = (*(upperRow.second + x) < satThreshold || *(lowerRow.second + x) < satThreshold) &&
                              std::abs(static_cast<short>(*(upperRow.first + x)) - static_cast<short>(*(lowerRow.first + x))) > minContrastDiff;
        Classification spotClassification = horizontalChange ? (verticalChange ? Nothing : Horizontal) : (verticalChange ? Vertical : Nothing);
        if(spotClassification != Nothing)
        {
          Region& region = regions[static_cast<int>(static_cast<float>(y) / yRegionsDivisor)][static_cast<int>(static_cast<float>(x) / xRegionsDivisor)];
          ++region.contrastChanges[spotClassification];
          if(static_cast<int>(y) > region.maxY)
            region.maxY = static_cast<int>(y);
        }
        if(midSat < satThreshold && midLum >= brightnessThreshold)
          ++regions[static_cast<int>(static_cast<float>(y) / yRegionsDivisor)][static_cast<int>(static_cast<float>(x) / xRegionsDivisor)].brightSpots;
      }
    }
  }
}

void RobotDetector::classifyRegions(std::vector<std::vector<Region>>& regions)
{
  const int minSpotsToClassify = std::max(5, std::min(30, static_cast<int>((theECImage.grayscaled.width * theECImage.grayscaled.height) / (xyRegions * xyRegions) / (xyStep * xyStep) / 10)));

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      region.regionIndices = Vector2i(xRegionIndex, yRegionIndex);

      const float hor = static_cast<float>(region.contrastChanges[0]);
      const float ver = static_cast<float>(region.contrastChanges[1]);
      if(hor + ver >= static_cast<float>(minSpotsToClassify))
        region.classification = (hor > ver ? ver / (ver + hor) : hor / (ver + hor)) > mixedThresh ? Mixed : (hor > ver ? Horizontal : Vertical);
      region.bright = region.brightSpots >= minSpotsToClassify;
    }
}

void RobotDetector::discardHomogeneousAreas(std::vector<std::vector<Region>>& regions)
{
  std::vector<Vector2i> toDiscard;
  std::vector<std::vector<bool>> visited(xyRegions, std::vector<bool>(xyRegions, false));

  auto checkHomogeneity = [this](std::vector<std::vector<Region>>& regions, Vector2i& indices, Classification relevantClass) -> bool
  {
    int hetCounter = 0;
    for(int y = std::max(0, indices.y() - 1); hetCounter < minHetSpots && y <= std::min(static_cast<int>(xyRegions - 1), indices.y() + 1); ++y)
      for(int x = std::max(0, indices.x() - 1); hetCounter < minHetSpots && x <= std::min(static_cast<int>(xyRegions - 1), indices.x() + 1); ++x)
      {
        const Region& neighbor = regions[y][x];
        if(neighbor.classification != Nothing&& neighbor.classification != relevantClass)
          ++hetCounter;
        if(hetCounter >= minHetSpots)
          return false;
      }
    return true;
  };

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      if(region.classification == Nothing || region.classification == Mixed || visited[yRegionIndex][xRegionIndex])
        continue;

      std::vector<Vector2i> toCheck = {Vector2i(xRegionIndex, yRegionIndex)};
      for(size_t index = 0; index < toCheck.size(); ++index)
      {
        Vector2i& indices = toCheck[index];
        if(checkHomogeneity(regions, indices, region.classification))
        {
          toDiscard.emplace_back(indices);
          visited[indices.y()][indices.x()] = true;
          std::vector<Vector2i> neighbors;
          regionQuery(regions, indices, 1, neighbors, region.classification);
          for(Vector2i& neighborIndices : neighbors)
            if(!visited[neighborIndices.y()][neighborIndices.x()])
            {
              visited[neighborIndices.y()][neighborIndices.x()] = true;
              toCheck.emplace_back(neighborIndices);
              toDiscard.emplace_back(neighborIndices);
            }
        }
      }
    }
  for(Vector2i& discardIndices : toDiscard)
    regions[discardIndices.y()][discardIndices.x()].classification = Nothing;

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      if(region.classification == Nothing && region.bright && !checkHomogeneity(regions, region.regionIndices, Unknown))
        region.classification = Unknown;
    }
}

void RobotDetector::dbScan(std::vector<std::vector<Region>>& regions, std::vector<ObstaclesImagePercept::Obstacle>& obstacles)
{
  for(std::vector<Region>& horizontalRegionLine : regions)
    for(Region& region : horizontalRegionLine)
    {
      if(region.classification == Nothing || region.clustered)
        continue;
      std::vector<Vector2i> neighbors;
      regionQuery(regions, region.regionIndices, 1, neighbors);
      if(neighbors.size() >= minNeighborPoints)
      {
        std::vector<Vector2i> cluster;
        Vector2i topLeft = region.regionIndices, bottomRight = Vector2i(region.regionIndices.x(), region.maxY);
        if(expandCluster(regions, region, neighbors, cluster, topLeft, bottomRight))
        {
          ObstaclesImagePercept::Obstacle obstacle;
          obstacle.top = static_cast<int>(static_cast<float>(topLeft.y()) * static_cast<float>(theECImage.grayscaled.height) / static_cast<float>(xyRegions));
          obstacle.bottom = bottomRight.y();
          obstacle.left = static_cast<int>(static_cast<float>(topLeft.x()) * static_cast<float>(theECImage.grayscaled.width) / static_cast<float>(xyRegions));
          obstacle.right = static_cast<int>(static_cast<float>(bottomRight.x() + 1) * static_cast<float>(theECImage.grayscaled.width) / static_cast<float>(xyRegions));
          obstacle.bottomFound = true;
          obstacle.fallen = false;

          if(!trimObstacles || trimObstacle(true, obstacle))
            obstacles.emplace_back(obstacle);
        }
      }
    }
}

bool RobotDetector::expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight)
{
  bool containsMixedRegions = false;
  cluster.emplace_back(region.regionIndices);
  region.clustered = true;
  containsMixedRegions = containsMixedRegions || region.classification == Mixed;

  for(unsigned int i = 0; i < neighbors.size(); ++i)
  {
    Vector2i& neighborIndices = neighbors[i];
    Region& currentRegion = regions[neighborIndices.y()][neighborIndices.x()];
    if(currentRegion.clustered)
      continue;

    cluster.emplace_back(neighborIndices);
    containsMixedRegions = containsMixedRegions || currentRegion.classification == Mixed;
    currentRegion.clustered = true;
    if(currentRegion.classification != Unknown)
    {
      topLeft = Vector2i(std::min(currentRegion.regionIndices.x(), topLeft.x()), std::min(currentRegion.regionIndices.y(), topLeft.y()));
      bottomRight = Vector2i(std::max(currentRegion.regionIndices.x(), bottomRight.x()), std::max(currentRegion.maxY, bottomRight.y()));
    }

    std::vector<Vector2i> nextNeighbors;
    regionQuery(regions, neighborIndices, 1, nextNeighbors);
    if(nextNeighbors.size() >= minNeighborPoints)
      for(Vector2i& nextNeighbor : nextNeighbors)
        if(!regions[nextNeighbor.y()][nextNeighbor.x()].clustered)
          neighbors.emplace_back(nextNeighbor);
  }
  for(int yRegionIndex = topLeft.y() - 1; yRegionIndex >= 0 && yRegionIndex == topLeft.y() - 1; --yRegionIndex)
    for(int xRegionIndex = topLeft.x(); xRegionIndex <= bottomRight.x(); ++xRegionIndex)
      if(regions[yRegionIndex][xRegionIndex].bright)
      {
        cluster.emplace_back(xRegionIndex, yRegionIndex);
        topLeft = Vector2i(topLeft.x(), yRegionIndex);
      }
  return containsMixedRegions;
}

void RobotDetector::regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& regionIndices, int dis, std::vector<Vector2i>& neighbors, Classification classificationToSearch)
{
  for(int y = std::max(regionIndices.y() - dis, 0); y < std::min(regionIndices.y() + dis + 1, static_cast<int>(xyRegions)); ++y)
    for(int x = std::max(regionIndices.x() - dis, 0); x < std::min(regionIndices.x() + dis + 1, static_cast<int>(xyRegions)); ++x)
      if((classificationToSearch == Default && regions[y][x].classification != Nothing) || (classificationToSearch == regions[y][x].classification))
        neighbors.emplace_back(x, y);
}
