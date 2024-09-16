/**
 * @file GoalPostsPerceptor.cpp
 *
 * This file implements the GoalPostsPerceptor, see GoalPostsPerceptor.h for further documentation.
 *
 * @author Laurens Schiefelbein
 */

#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "GoalPostsPerceptor.h"
#include "ImageProcessing/PatchUtilities.h"
#include "Libs/ImageProcessing/PixelTypes.h"
#include "Libs/ImageProcessing/Image.h"
#include "Math/Geometry.h"
#include "Platform/File.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(GoalPostsPerceptor);

GoalPostsPerceptor::GoalPostsPerceptor() :
  classifier(&Global::getAsmjitRuntime()),
  detector(&Global::getAsmjitRuntime())
{
  // Initialize models for the neural net
  classifier_model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir())
                                                            + "/Config/NeuralNets/GoalPostsPerceptor/classifier_model.h5");
  classifier.compile(*classifier_model);

  detector_model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir())
                                                          + "/Config/NeuralNets/GoalPostsPerceptor/detector_model.h5");
  detector.compile(*detector_model);

  ASSERT(classifier.numOfInputs() == 1);
  ASSERT(detector.numOfInputs() == 1);

  ASSERT(classifier.input(0).rank() == 3);
  ASSERT(detector.input(0).rank() == 3);

  ASSERT(classifier.input(0).dims(2) == 1);
  ASSERT(detector.input(0).dims(2) == 1);

  ASSERT(classifier.numOfOutputs() == 1);
  ASSERT(detector.numOfOutputs() == 1);

  ASSERT(classifier.output(0).dims(0) == 1);
  ASSERT(detector.output(0).dims(0) == 2);
}

std::vector<GoalPostsPerceptor::RegionWithPosition> GoalPostsPerceptor::getRegionsFromScanlines()
{
  std::vector<RegionWithPosition> candidateRegions;

  for(const auto& line : theColorScanLineRegionsHorizontal.scanLines)
  {
    for(const ScanLineRegion& r : line.regions)
    {
      // Filter out ScanLineRegions that are:
      // - not in the color of the goal post
      // - on top of FieldLines
      // - too far away from any actual goal post
      // - to big along the x-Axis (minimalRegionSizeX)
      if(r.color == goalFrameColor &&
         !GoalPostsPerceptor::isLineInRegion(r, line.y) &&
         isCloseToGoalPost(r, line.y) &&
         (r.range.to - r.range.from) >= minimalRegionSizeX)
      {
        candidateRegions.emplace_back(line.y, r);

        LINE("module:GoalPostsPerceptor:candidateLines", r.range.from, line.y, r.range.to - 1, line.y, 4, Drawings::solidPen, ColorRGBA::white);
      }
    }
  }
  return candidateRegions;
}

std::vector<GoalPostsPerceptor::GoalPostRegion> GoalPostsPerceptor::expandRegions(const std::vector<RegionWithPosition>& scanLineRegions)
{
  std::vector<GoalPostRegion> expandedRegions;

  // Middle point of the ScanLineRegion
  Vector2f regionMiddle;

  // Corner points for the GoalPostRegion
  Vector2f upperLeft;
  Vector2f lowerRight;

  for(const RegionWithPosition& region : scanLineRegions)
  {
    regionMiddle = Vector2f(region.region.range.from + (region.region.range.to - region.region.range.from) / 2, region.yPosition);

    // Expansion
    upperLeft = Vector2f(region.region.range.from - regionExtensionSideways, region.yPosition - regionExtensionUpwards);
    lowerRight = Vector2f(region.region.range.to - 1 + regionExtensionSideways, region.yPosition + regionExtensionDownwards);

    expandedRegions.emplace_back(GoalPostRegion(upperLeft.x(), upperLeft.y(), lowerRight.x(), lowerRight.y()));

    CROSS("module:GoalPostsPerceptor:regionMiddle", regionMiddle.x(), regionMiddle.y(), 4, 4, Drawings::solidPen, ColorRGBA::blue);
    RECTANGLE("module:GoalPostsPerceptor:expandedRegions", upperLeft.x(), upperLeft.y(), lowerRight.x(), lowerRight.y(), 4, Drawings::solidPen, ColorRGBA::red);
  }

  return expandedRegions;
}

std::vector<GoalPostsPerceptor::GoalPostRegion> GoalPostsPerceptor::combineOverlappingRegions(const std::vector<GoalPostRegion>& regionList)
{
  std::vector<GoalPostRegion> regionsToCheck = regionList;

  // New corner points for the GoalPostRegion that is created of two overlapping GoalPostRegions
  Vector2f newUpperLeft;
  Vector2f newLowerRight;

  // Corner points for both GoalPostRegions
  Vector2f upperLeft1;
  Vector2f upperLeft2;
  Vector2f lowerRight1;
  Vector2f lowerRight2;

  // Region for outer loop
  GoalPostRegion currentRegion;

  // Region for inner loop
  GoalPostRegion innerRegion;

  // True if an overlap of sufficient size between two regions has been found
  bool foundOverlap;

  do
  {
    foundOverlap = false;

    for(unsigned long i = 0; i < regionsToCheck.size(); i++)
    {
      currentRegion = regionsToCheck[i];
      for(unsigned long j = i + 1; j < regionsToCheck.size(); j++)
      {
        innerRegion = regionsToCheck[j];
        if(isOverlapPresent(currentRegion, innerRegion))
        {
          foundOverlap = true;

          // Set corner points for the combined region
          newUpperLeft = Vector2f(std::min(currentRegion.upperLeft.x(), innerRegion.upperLeft.x()), std::min(currentRegion.upperLeft.y(), innerRegion.upperLeft.y()));
          newLowerRight = Vector2f(std::max(currentRegion.lowerRight.x(), innerRegion.lowerRight.x()), std::max(currentRegion.lowerRight.y(), innerRegion.lowerRight.y()));

          currentRegion = GoalPostRegion(newUpperLeft.x(), newUpperLeft.y(), newLowerRight.x(), newLowerRight.y());

          // Erase region that has been combined with currentRegion
          regionsToCheck.erase(regionsToCheck.begin() + j--);
        }
      }
      // Replace currentRegion in the vector with the new combined and bigger GoalPostRegion
      regionsToCheck[i] = currentRegion;
    }
  }
  while(foundOverlap);

  // Draw the combined regions
  for(const GoalPostRegion& combined : regionsToCheck)
  {
    RECTANGLE("module:GoalPostsPerceptor:combinedRegions", combined.upperLeft.x(), combined.upperLeft.y(), combined.lowerRight.x(), combined.lowerRight.y(), 4, Drawings::solidPen, ColorRGBA::cyan);
  }

  return regionsToCheck;
}

std::vector<Vector2f> GoalPostsPerceptor::generatePatch(const std::vector<GoalPostRegion>& regionList)
{
  const bool writeFiles = false;

  // Center of GoalPostRegion
  Vector2f pInImg;
  Vector2f pInImgRelative;
  Image<PixelTypes::GrayscaledPixel> patch(patchSize, patchSize);
  // Vector of all found goalPostBases
  std::vector<Vector2f> foundGoalPostBases;

  for(int i = 0; i < candidateLimit; i++)
  {
    if((int)regionList.size() <= i) break;

    // center of the patch to extract. The patch is a square of which the edge length is the same as the horizontal edge of the GoalPostRegion
    pInImg = Vector2f(regionList[i].upperLeft.x() + (regionList[i].lowerRight.x() - regionList[i].upperLeft.x()) / 2,
                      regionList[i].lowerRight.y() - (regionList[i].lowerRight.x() - regionList[i].upperLeft.x()) / 2);

    // Check whether the center of the patch would even be within the bounds of the camera image.
    if(!isWithinBounds(pInImg, patchSize))
      continue;

    CROSS("module:GoalPostsPerceptor:centerOfPatch", pInImg.x(), pInImg.y(), 4, 4, Drawings::solidPen, ColorRGBA::orange);

    if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pInImg), theCameraMatrix, theCameraInfo, pInImgRelative))
    {
      // distance from robot to center of patch (in mm)
      const float distanceToGoalPost = pInImgRelative.squaredNorm();

      if(distanceToGoalPost > sqr(maxDistanceToCandidate))
        continue;

      // Directory to save patches to
      const std::string target_dir = ""; // < Absolute path

      // used to name the files
      const std::string time = std::to_string(theFrameInfo.time);
      const std::string competition = ""; // < Example: RoboCup2022
      const std::string opponent = ""; // < Example: HULKs
      const std::string half = ""; // < Can be 1st or 2nd
      const std::string robot = ""; // < IP of playing robot
      const std::string camera = theCameraInfo.camera == CameraInfo::lower ? "lower" : "upper";
      const Vector2f center(pInImg.x(), pInImg.y());

      // Capture the whole width of GoalPostRegion
      const int inputSize = (int)round(regionList[i].lowerRight.x() - regionList[i].upperLeft.x());

      const std::string fileName = competition + "-" + opponent + "-" + half + "-" + robot + "-" + time + "-" + camera + "-" + std::to_string((int)round(distanceToGoalPost));

      PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(patchSize, patchSize), theECImage.grayscaled, patch);

      RECTANGLE("module:GoalPostsPerceptor:candidatePatch", pInImg.x() - inputSize / 2, pInImg.y() - inputSize / 2, pInImg.x() + inputSize / 2, pInImg.y() + inputSize / 2, 4, Drawings::solidPen, ColorRGBA::yellow);

      const std::string file = target_dir + fileName + ".bin";

      // Put patch into file
      if(writeFiles)
      {
        OutBinaryFile stream(file);
        for(unsigned int i = 0; i < patch.height; i++)
        {
          for(unsigned int j = 0; j < patch.width; j++)
          {
            stream << patch[i][j];
          }
        }
      }
      // Use neural networks to classify candidate and detect goal post base.
      else
      {
        Vector2f goalPostBase;
        if(classifyGoalPost(patch))
        {
          RECTANGLE("module:GoalPostsPerceptor:classifiedGoalPosts", pInImg.x() - inputSize / 2, pInImg.y() - inputSize / 2, pInImg.x() + inputSize / 2, pInImg.y() + inputSize / 2, 4, Drawings::solidPen, ColorRGBA::green);

          goalPostBase = getGoalPostBase(patch, pInImg, inputSize);
          foundGoalPostBases.emplace_back(goalPostBase);
        }
      }
    }
  }
  return foundGoalPostBases;
}

bool GoalPostsPerceptor::classifyGoalPost(const Image<PixelTypes::GrayscaledPixel>& patch)
{
  // Put patch into network
  PatchUtilities::extractPatch(Vector2i(patchSize / 2, patchSize / 2), Vector2i(patchSize, patchSize), Vector2i(patchSize, patchSize), patch, classifier.input(0).data());

  // Run network
  STOPWATCH("module:GoalPostsPerceptor:classifier") classifier.apply();

  return classifier.output(0)[0] >= minClassificationThreshold;
}

Vector2f GoalPostsPerceptor::getGoalPostBase(const Image<PixelTypes::GrayscaledPixel>& patch, const Vector2f& pInImg, const int inputSize)
{
  // Put patch into network
  PatchUtilities::extractPatch(Vector2i(patchSize / 2, patchSize / 2), Vector2i(patchSize, patchSize), Vector2i(patchSize, patchSize), patch, detector.input(0).data());

  STOPWATCH("module:GoalPostsPerceptor:detector") detector.apply();

  // Correct the network output, so that the coordinates are on the patch and in the correct scale.
  // detector_network.output(0)[0] * (inputSize / patchSize) --- corrects the scaling of the prediction.
  // + (pInImg.x() - inputSize/2) --- put the prediction point over the patch.
  const Vector2f goalPostBase = Vector2f(detector.output(0)[0] * (inputSize / patchSize) + (pInImg.x() - inputSize / 2),
                                         detector.output(0)[1] * (inputSize / patchSize) + (pInImg.y() - inputSize / 2));

  CROSS("module:GoalPostsPerceptor:goalPostBase", goalPostBase.x(), goalPostBase.y(), 4, 4, Drawings::solidPen, ColorRGBA::green);

  return goalPostBase;
}

bool GoalPostsPerceptor::isLineInRegion(const ScanLineRegion& region, const unsigned short y)
{
  // Starting point of FieldLine in image coordinates
  Vector2f lineFirstImg;
  // End point of FieldLine in image coordinates
  Vector2f lineLastImg;

  // Check every fieldLine if it intersects with the ScanLineRegion in question
  for(const auto& line : theFieldLines.lines)
  {
    // Transform point on FieldLine in relative coordinates to image coordinates and check if results are valid.
    // Then check for an intersection between the region in question and the FieldLine. If an intersection is present return true
    if(Transformation::robotToImage(line.first, theCameraMatrix, theCameraInfo, lineFirstImg) &&
       Transformation::robotToImage(line.last, theCameraMatrix, theCameraInfo, lineLastImg) &&
       Geometry::checkIntersectionOfLines(Vector2f(region.range.from, y), Vector2f(region.range.to - 1, y), lineFirstImg, lineLastImg))
      return true;
  }
  return false;
}

bool GoalPostsPerceptor::isCloseToGoalPost(const ScanLineRegion& region, const unsigned short y)
{
  // Middle point of the ScanLineRegion (image coordinates)
  const Vector2f regionMiddle = Vector2f(region.range.from + (region.range.to - region.range.from) / 2, y);

  // transform middle point to robot coordinates
  Vector2f regionMiddleRobot;
  if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(regionMiddle), theCameraMatrix, theCameraInfo, regionMiddleRobot))
  {
    // Transform goal post position to robot coordinates
    const Vector2f goalPostOwnLowerRobot = theRobotPose.inverse() * goalPostOwnLower;
    const Vector2f goalPostOwnUpperRobot = theRobotPose.inverse() * goalPostOwnUpper;
    const Vector2f goalPostOpponentLowerRobot = theRobotPose.inverse() * goalPostOpponentLower;
    const Vector2f goalPostOpponentUpperRobot = theRobotPose.inverse() * goalPostOpponentUpper;

    // Distance from regionMiddle to goal post
    const float distanceRegionToGoalPostOwnLower = (goalPostOwnLowerRobot - regionMiddleRobot).squaredNorm();
    const float distanceRegionToGoalPostOwnUpper = (goalPostOwnUpperRobot - regionMiddleRobot).squaredNorm();
    const float distanceRegionToGoalPostOpponentLower = (goalPostOpponentLowerRobot - regionMiddleRobot).squaredNorm();
    const float distanceRegionToGoalPostOpponentUpper = (goalPostOpponentUpperRobot - regionMiddleRobot).squaredNorm();

    // if the ScanLineRegion is close enough to any actual goal post return true
    if(distanceRegionToGoalPostOwnLower <= sqr(maxDistanceFromGoalToRegion) ||
       distanceRegionToGoalPostOwnUpper <= sqr(maxDistanceFromGoalToRegion) ||
       distanceRegionToGoalPostOpponentLower <= sqr(maxDistanceFromGoalToRegion) ||
       distanceRegionToGoalPostOpponentUpper <= sqr(maxDistanceFromGoalToRegion))
      return true;
  }
  return false;
}

bool GoalPostsPerceptor::isOverlapPresent(const GoalPostRegion& region1, const GoalPostRegion& region2)
{
  // Corner points for both GoalPostRegions
  const Vector2f& upperLeft1 = region1.upperLeft;
  const Vector2f& upperLeft2 = region2.upperLeft;
  const Vector2f& lowerRight1 = region1.lowerRight;
  const Vector2f& lowerRight2 = region2.lowerRight;

  // Horizontal and vertical sides for the regions at hand. This is used to determine the intersectional area of both GoalPostRegions
  const Rangef horizontal1 = {upperLeft1.y(), lowerRight1.y()};
  const Rangef horizontal2 = {upperLeft2.y(), lowerRight2.y()};
  const Rangef vertical1 = {upperLeft1.x(), lowerRight1.x()};
  const Rangef vertical2 = {upperLeft2.x(), lowerRight2.x()};

  const float areaOfRegion1 = getAreaOfRectangle(lowerRight1.x(), lowerRight1.y(), upperLeft1.x(), upperLeft1.y());
  const float areaOfRegion2 = getAreaOfRectangle(lowerRight2.x(), lowerRight2.y(), upperLeft2.x(), upperLeft2.y());

  // Calculates the area of intersection of region1 and region2
  const float intersectingArea = horizontal1.intersectionSizeWith(horizontal2) * vertical1.intersectionSizeWith(vertical2);

  const float overlapPercentage = intersectingArea / (areaOfRegion1 + areaOfRegion2 - intersectingArea) * 100;

  // Check if overlap is significant enough
  if(overlapPercentage < minimumLappedRegionPercentage)
    return false;

  return true;
}

bool GoalPostsPerceptor::isWithinBounds(const Vector2f& point, const int patchSize)
{
  // TODO: does this really need the patchSize? GoalPost width is probably more suitable.
  return (point.x() - patchSize / 2 >= 0 &&
          point.x() + patchSize / 2 <= theCameraInfo.width &&
          point.y() - patchSize / 2 >= 0 &&
          point.y() + patchSize / 2 <= theCameraInfo.height);
}

float GoalPostsPerceptor::getAreaOfRectangle(const float x1, const float y1, const float x2, const float y2)
{
  return std::abs((x1 - x2) * (y1 - y2));
}

void GoalPostsPerceptor::update(GoalPostsPercept& goalPostsPercept)
{
  // Check whether networks have been successfully compiled.
  if(!(classifier.valid() && detector.valid())) return;

  DECLARE_DEBUG_RESPONSE("module:GoalPostsPerceptor:relativePosition");

  // White scan line regions which could be part of a goal post
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:candidateLines", "drawingOnImage");
  // The middle point of each candidateLine
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:regionMiddle", "drawingOnImage");
  // Rectangles that have been expanded from white scan lines (candidateLines)
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:expandedRegions", "drawingOnImage");
  // All expandedRegions that are overlapping by at least minimumLappedRegionPercentage
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:overlappingRegions", "drawingOnImage");
  // Combined Region from all overlapping regions
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:combinedRegions", "drawingOnImage");

  // Candidate patches that are calculated from combined regions. These will be used for classification
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:candidatePatch", "drawingOnImage");
  // The center point of each candidatePatch
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:centerOfPatch", "drawingOnImage");

  // The candidates that have been classified as goal posts.
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:classifiedGoalPosts", "drawingOnImage");
  // Position of the goal post base.
  DECLARE_DEBUG_DRAWING("module:GoalPostsPerceptor:goalPostBase", "drawingOnImage");

  goalPostsPercept.goalPosts.clear();

  // Extract filtered candidate lines
  const std::vector<RegionWithPosition> candidateRegions = getRegionsFromScanlines();

  // Expand regions
  const std::vector<GoalPostsPerceptor::GoalPostRegion> expandedRegions = expandRegions(candidateRegions);

  // Bind overlapping candidate regions together
  const std::vector<GoalPostRegion> combinedRegions = combineOverlappingRegions(expandedRegions);

  // Generate patches from combined regions, classify and detect goal post base
  const std::vector<Vector2f> foundGoalPostBases = generatePatch(combinedRegions);

  // ======== Write into GoalPostsPercept ========
  for(const Vector2f& goalPostBase : foundGoalPostBases)
  {
    GoalPostsPercept::GoalPost goalPost;
    Vector2f goalPostBaseRelative;
    Matrix2f covarianceOnField;

    goalPost.baseInImage = isWithinBounds(goalPostBase, 0); // Check whether base in within image bounds.
    goalPost.positionInImage = goalPostBase;

    if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(goalPostBase), theCameraMatrix, theCameraInfo, goalPostBaseRelative))
    {
      goalPost.positionOnField = goalPostBaseRelative;
      if(theMeasurementCovariance.transformPointWithCov(goalPostBase, 0.f, goalPostBaseRelative, covarianceOnField))
      {
        goalPost.covarianceOnField = covarianceOnField;
        goalPostsPercept.goalPosts.push_back(goalPost);
      }
    }
  }
  // Clear list, if there have been too many goal posts
  if(goalPostsPercept.goalPosts.size() > 2)
  {
    ANNOTATION("GoalPostsPerceptor", "Found more than two goal posts: " << static_cast<int>(goalPostsPercept.goalPosts.size()));
    goalPostsPercept.goalPosts.clear();
  }
}
