/**
 * @file AutoLabelImageProvider.h
 *
 * Provider for LabelImage based on simulated data.
 * Randomly places other robots of the field to random location.
 * This can be logged in the simulator and used for further processing.
 *
 * @author Jan Blumenkamp
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

MODULE(AutoLabelImageProvider,
{,
  REQUIRES(GroundTruthWorldState),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  PROVIDES(LabelImage),
  DEFINES_PARAMETERS(
  {,
    (float)(573.f) robotHeight, ///< Bounding box robot height
    (float)(220.f) robotHandsHeight, ///< Bounding box height of the hands
    (float)(200.f) robotWidthBody, ///< Bounding box robot width for the body
    (float)(150.f) robotWidthHead, ///< Bounding box robot width for the head
    (float)(350.f) robotWidthHands, ///< Bounding box robot width for the hands
    (float)(140.f) robotDepthFeet, ///< Bounding box robot depth of feet
    (float)(110.f) robotDepthHead, ///< Bounding box robot depth of head
    (float)(60.f) robotCenter, ///< Bounding Box center of depth (offset) of bounding box
    (int)(20) marginBoundingBox, ///< Only if an object is at least this many pixels inside a frame a bounding box will be created
    (int)(370) minDistanceToRobots, ///< Minimum distance the teleported robot should keep to other robots
    (float)(4800.f) robotPlacementAreaLength, ///< Range (-n to n) along the field length where robots will be placed randomly
    (float)(3000.f) robotPlacementAreaWidth, ///< Range (-n to n) along the field width where robots will be placed randomly
    (int)(8) minRobotPlacementId, ///< Minimum ID of robot whose position will be manipulated randomly
    (int)(12) maxRobotPlacementId, ///< Maximum ID of robot whose position will be manipulated randomly
  }),
});

class AutoLabelImageProvider : public AutoLabelImageProviderBase
{
private:
  void update(LabelImage& labelImage) override;

  /**
   * Populates the given vector with points representing a 3D bounding box for a robot
   * @param box vector to be populated. Will be overwritten.
   */
  std::vector<Vector3f> getPlainRobot3DBoundingBox();

  /**
   * Translates and rotates the given vector of 3D points (representing a polygone) by the given pose in place
   * @param box vector of 3D points to be moved (will not be changed)
   * @param moved
   * @param pose
   */
  void move3DBoundingBox(std::vector<Vector3f>& box, const Pose2f& pose);

  /**
   * Draws a 3D bounding box in robot relative coordinates to the field
   * @param relative3DBoundingBox 3D bounding box in robot relative coordinates
   */
  void drawRelative3DBoundingBox(const std::vector<Vector3f>& relative3DBoundingBox);

  /**
   * Projects a robot relative 3D Bounding box to an 2D annotation bounding box
   * @param box vector of 3D bounding box points to be projected
   * @param annotation annotation object to write the bounding box data in.
   * Only manipulates correct, upperLeft, lowerRight
   * @return true on success, otherwise false
   */
  bool projectRelative3DBoundingBoxToAnnotation(const std::vector<Vector3f>& box, LabelImage::Annotation& annotation);

  /**
   * Teleport the robot to the given absolute field coordinate
   * @param robotId robot with the given id to move
   * @param pos absolute field coordinate
   */
  void teleportRobot(int robotId, const Pose2f& pose);

  /**
   * Calculates the distance to the closest robot from the given position
   * @param position distance to consider
   * @return distance to the closes robot in mm. infinity if there are no robots.
   */
  float calculateDistanceToClosestRobot(const Vector2f& position);
};
