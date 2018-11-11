/**
 * @file NNBallPerceptor.cpp
 *
 * This file implements a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 */

#include "NNBallPerceptor.h"
#include "Platform/File.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Tools/NeuralNetwork/SimpleNN.h"

MAKE_MODULE(NNBallPerceptor, perception)

NNBallPerceptor::NNBallPerceptor() { compile(); }

void NNBallPerceptor::update(ConfirmedBallSpot& ballSpot)
{
  MODIFY("module:NNBallPerceptor:config", config);
  MODIFY("module:NNBallPerceptor:stats", stats);

  DECLARE_DEBUG_DRAWING("module:NNBallPerceptor:spots", "drawingOnImage");
  ballSpot.status = BallPercept::notSeen;

  if(theLabelImage.valid && theLabelImage.hasLabel(Annotation::Ball))
    stats.total++;

  if(path != lastType)
    compile();

  if(!convModel.valid() || (config.useMeta && !metaModel.valid()))
    return;

  if(theBallSpots.ballSpots.empty())
    return;

  probs.resize(theBallSpots.ballSpots.size());
  int maxIdx;
  float max = 0.f;

  for(unsigned int i =  0; i < theBallSpots.ballSpots.size(); i++)
  {
    float prob = apply(theBallSpots.ballSpots[i], i, Vector2f(0, 0));
    if(config.useResampling && prob > config.resampleThreshold)
    {
      float tmp = apply(theBallSpots.ballSpots[i], i, Vector2f(0.5, 0));
      tmp = std::max(tmp, apply(theBallSpots.ballSpots[i], i, Vector2f(-0.5, 0)));
      tmp = std::max(tmp, apply(theBallSpots.ballSpots[i], i, Vector2f(0, 0.5)));
      tmp = std::max(tmp, apply(theBallSpots.ballSpots[i], i, Vector2f(0, -0.5)));
      if(tmp > prob)
      {
        //OUTPUT_TEXT("Upgraded from " << prob << " to " << tmp);
        prob = tmp;
      }
    }

    probs[i] = prob;

    //if(probs[i] > acceptThreshold)
    //{
    std::stringstream ss;
    ss << i << ": " << static_cast<int>(probs[i] * 100) << "\n";
    DRAWTEXT("module:NNBallPerceptor:spots", theBallSpots.ballSpots[i].x(), theBallSpots.ballSpots[i].y(), 15, ColorRGBA::red, ss.str());
    //}

#ifdef TARGET_ROBOT
    if(probs[i] >= config.ensureThreshold)
    {
      ballSpot.positionInImage = theBallSpots.ballSpots[i];
      ballSpot.status = BallPercept::seen;
      return;
    }
#endif
  }

  max = probs.maxCoeff(&maxIdx);

  if(max > config.guessedThreshold)
  {
    ballSpot.positionInImage = theBallSpots.ballSpots[maxIdx];
    ballSpot.status = max >= config.acceptThreshold ? BallPercept::seen : BallPercept::guessed;
  }

  // evaluation: super slow
#ifndef TARGET_ROBOT
  for(unsigned i = 0; i < theBallSpots.ballSpots.size(); i++)
  {
    float radius = IISC::getImageBallRadiusByCenter(theBallSpots.ballSpots[i].cast<float>(), theCameraInfo, theCameraMatrix, theBallSpecification);
    int ballArea = static_cast<int>(radius * config.ballAreaFactor);
    ballArea += 4 - (ballArea % 4);

    int hasLabel = -1;
    if(theLabelImage.valid)
    {
      hasLabel = static_cast<int>(theLabelImage.hasLabel(theBallSpots.ballSpots[i], Vector2i(ballArea, ballArea), Annotation::Ball));
      stats.add(hasLabel, probs[i]);
    }

    if(config.logImages && (!config.logOnlyFalse || theLabelImage.valid))
    {
      GrayscaledImage debug;
      NNUtilities::extractPatch(theBallSpots.ballSpots[i], Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, debug, config.extractionMode);
      if(config.useContrastNormalization)
        NNUtilities::normalizeContrast(debug, config.contrastNormalizationPercent);

      if((!config.logOnlyFalse || std::fabs(hasLabel - probs[i]) > 0.5))
        debug.exportImage(std::to_string(static_cast<int>(10000 * probs[i])) + "_" + std::to_string(hasLabel) + "_" + TypeRegistry::getEnumName(config.extractionMode) + "_" + std::to_string(i), theFrameInfo.time, GrayscaledImage::grayscale);
    }
  }
#endif
}

float NNBallPerceptor::apply(const Vector2i& ballSpot, int i, const Vector2f& offset)
{
  Vector2f relativePoint;
  if(!Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePoint))
    return 0.f;

  float radius = IISC::getImageBallRadiusByCenter(ballSpot.cast<float>(), theCameraInfo, theCameraMatrix, theBallSpecification);
  int ballArea = static_cast<int>(radius * config.ballAreaFactor);
  ballArea += 4 - (ballArea % 4);

  Vector2i ballSpot_ = ballSpot - (offset.cast<float>().array() * radius).matrix().cast<int>();
  STOPWATCH("module:NNBallPerceptor:getImageSection")
    if(config.useFloat)
    {
      NNUtilities::extractPatch(ballSpot_, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, convModel.input().data(), config.extractionMode);
      if(config.useContrastNormalization)
        NNUtilities::normalizeContrast(convModel.input().data(), Vector2i(patchSize, patchSize), config.contrastNormalizationPercent);
    }
    else
    {
      NNUtilities::extractPatch(ballSpot_, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, reinterpret_cast<unsigned char*>(convModel.input().data()), config.extractionMode);
      if(config.useContrastNormalization)
        NNUtilities::normalizeContrast(reinterpret_cast<unsigned char*>(convModel.input().data()), Vector2i(patchSize, patchSize), config.contrastNormalizationPercent);
    }

  NeuralNetwork::Tensor3 outputTensor;
  if(config.useVerification)
  {
    outputTensor = NeuralNetwork::Tensor3(convModel.output());
    NeuralNetwork::Tensor3 inputTensor(convModel.input());
    convModel.apply();
    NeuralNetwork::SimpleNN::apply(inputTensor, outputTensor, *model);
    if((outputTensor[0] > 0.5)  != (convModel.output()[0] > 0.5))
      OUTPUT_TEXT("" << outputTensor[0] - convModel.output()[0]);
  }
  else
    convModel.apply();

  float result = config.useVerification ? outputTensor[0] :  convModel.output()[0];

  if(!config.useMeta)
    return result;

  // Copy output to input of Meta model
  std::copy_n(convModel.output().data(), convModel.output().size(), metaModel.input().data());

  // Fill in the rest of the Meta model input
  float* metaInput = &metaModel.input()[convModel.output().size()];
  *metaInput++ = static_cast<float>(theCameraInfo.camera);
  *metaInput++ = static_cast<float>(ballSpot.x()) / theCameraInfo.width;
  *metaInput++ = static_cast<float>(ballSpot.y()) / theCameraInfo.height;
  *metaInput = relativePoint.norm() / 1000.f;

  metaModel.apply();
  return handleReturn(metaModel.output());
}

float NNBallPerceptor::handleReturn(const NeuralNetwork::Tensor3& output)
{
  if(output.dims(0) == 1)
    return output[0];
  else
  {
    if((output[0] > output[1]) && (output[0] > output[2]))
      return 1.0f;
    if((output[1] > output[0]) && (output[1] > output[2])) // && output[0] > output[2])
      return 0.3f;
    else
      return 0.f;
  }
}

void NNBallPerceptor::compile()
{
  lastType = path;
  InMapFile stream("NNBallPerceptor/" + std::string(TypeRegistry::getEnumName(path)) + "/nnBallPerceptor.cfg");
  ASSERT(stream.exists());
  stream >> config;
  models.clear();

  NeuralNetwork::CompilationSettings convSettings;
  convSettings.uint8Input = !config.useFloat;
  model.reset(new NeuralNetwork::Model("NNBallPerceptor/" + std::string(TypeRegistry::getEnumName(path)) + "/conv.model"));
  convModel.compile(*model, convSettings);
  models.push_back(&convModel);

  if(config.useMeta)
  {
    metaModel.compile("NNBallPerceptor/" + std::string(TypeRegistry::getEnumName(path)) + "/meta.model");
    models.push_back(&metaModel);
  }
  ASSERT(models.front()->input().dims(0) == models.front()->input().dims(1));
  ASSERT(models.front()->input().dims(2) == 1);
  ASSERT(models.back()->output().dims(0) == 1 || models.back()->output().dims(0) == 3);
  ASSERT(models.back()->output().dims(1) == 1);
  ASSERT(models.back()->output().dims(2) == 1);
  patchSize = models.front()->input().dims(0);
}
