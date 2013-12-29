/*
 * ObstacleSpotProvider.h
 *
 *  Created on: Aug 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/PossibleObstacleSpots.h"
#include "Representations/Perception/ObstacleSpots.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Sensing/GroundContactState.h"
#include <list>

MODULE(ObstacleSpotProvider)
  REQUIRES(PossibleObstacleSpots)
  REQUIRES(Image)
  REQUIRES(ColorReference)
  REQUIRES(BodyContour)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(Odometer)
  REQUIRES(GroundContactState)
  REQUIRES(ImageCoordinateSystem)
  PROVIDES_WITH_DRAW(ObstacleSpots)
  DEFINES_PARAMETER(int, nonGreenCount, 11)
  DEFINES_PARAMETER(float, minObstacleHeight, 200.0f) /**< in mm */
  DEFINES_PARAMETER(float, allowedHeightDifference, 0.3f) /**< in percent of the expected height */
  DEFINES_PARAMETER(int, allowedDifferenceFromTop, 1) /**< Num of pixels that an obstacle is allowed to be away from the image border if the obstacle should be bigger than the image */
  DEFINES_PARAMETER(float, maxNeighborDistance, 150.0f) /**< Max distance between two spots when clustering. In mm in field coordinates */
  DEFINES_PARAMETER(int, legHeight, 150); /**< Length of the NAO's legs. This is used to determine at what height the ban zone should start.*/
  DEFINES_PARAMETER(int, banZoneWidth, 130) /**< Width of the hands ban zone in field coordinates. Note: This has nothing to do with the ban zone that is part of the representation */
  DEFINES_PARAMETER(int, robotWidth, 400) /**< width of the robot. Used to calc ban zone */
  DEFINES_PARAMETER(int, robotHeight, 600) /**< height of a robot. Used to calc ban zone */
  DEFINES_PARAMETER(int, shouldHaveNeighborDistance, 400) /**< If a cluster is closer than this it should contain at least two spots */
  DEFINES_PARAMETER(int, lowerCameraLegHeight, 50) /**< special case for lower camera ban zone creation */
  DEFINES_PARAMETER(int, MaxTriesPerScanline, 6) /**<Only the first n spots per scanline will be considered */
  DEFINES_PARAMETER(int, tooCloseToBottom, 5) /**< If an obstacle is closer to the bottom of the upper image than tooCloseToBottom + nonGreenCount all obstacles will be ignored */
END_MODULE


/**
 * The ObstacleSpotProvider takes PossibleObstacleSpots and refines them into
 * clustered spots at the feet of obstacles.
 *
 * It consists of a small pipeline that refines the spots in several steps:
 * 1. Check how much not-green is above and below a possible obstacle spot.
 *    This determines the upper and lower boundary of the obstacle. If the obstacle is noth
 *    high enough it is discarded.
 *    If we are looking at an image from the lower camera and the obstacle is
 *    touching the upper border of the image the obstacle is buffered and evaluated
 *    when the next upper image comes around.
 *    This steps include several hacks to avoid certain types of false positives
 *    which are not part of this documentation :)
 *    The lower border of every obstacle is saved. All further steps operate
 *
 * 2. Spots are sorted from bottom to top. This is important for the following steps.
 *
 * 3. Obstacle spots at the hands/arms of robots are removed. This is done by
 *    assuming that the bottom most spot inside any image is at the foot of a robot.
 *    Based on the bottom most spot ban zones are calculated that should contain the arms/hands
 *    of the robot. All spots inside those ban zones are removed. This step
 *    is repeated until no spots are removed.
 *
 * 4. Spots are clustered and a center of mass is calculated for each cluster.
 *
 * 5. A rectangle around the robot is calculated for each cluster based on it's
 *    center of mass. This rectangle is also called a ban zone (sorry for that :) )
 *
 * 6. If the center of mass of one cluster is inside the ban zone of another cluster
 *    it is removed. This is done in a way that clusters that are closer to the robot
 *    remain while the ones further away are removed.
 *
 * @note This comment is a very brief overview. It is by no means complete.
 *       There are lots and lots and lots of small bugfixes, workarounds and hacks
 *       woven into this code :)
 */
class ObstacleSpotProvider : public ObstacleSpotProviderBase
{
public:
  ObstacleSpotProvider();
private:

  struct BufferedSpot
  {
    Vector2<int> spotInImg; /**< spot in image coordinates */
    Vector2<float> spotInWorld; //< location of the spot in robot relative field coordinates.
    Vector2<float> borderPoint; //< location where the obstacle hit the upper image border in relative field coordinates.
    int height; //< Height that the obstacle should have in the upper image. (Scale this to the resolution of the upper image!!!)
  };

  void update(ObstacleSpots& grid);

  /** Returns true if the bottom end of the obstacle was found. false otherwise.
   *  If the bottom end was found outBottomCoordinate contains the location*/
  bool searchDownward(const int x, int y, int yEnd, Vector2<int>& outBottomCoordinate);

  Vector2<int> searchUpward(const int x, int y, const int yEnd);

  /**counts the consecutive green pixels between [pStart .. pEnd].
   * If pStart is bigger than pEnd it is scanned backwards.
   * Otherwise forward
   */
  int countGreen(const Image::Pixel* pStart,const Image::Pixel* pEnd,
                 const int widthstep);

  /**
   * Compare measured height to expected height of a nao.
   * Note: If this function is not sure about the result it writes the point
   * to bufferedSpots and returns false. Call handleBufferedSpots() to
   * process those spots in the next frame.
   * @param height
   * @param footPoint
   * @return true if it does. False otherwise
   */
  bool fullFillsHeightConstraint(const int height, const Vector2<int>& footPoint);

  /**Check if height and expectedHeight are close enough together*/
  inline bool checkHeight(const int height, const int expectedHeight);


  /**Check if the specified pixel belongs to the ground*/
  inline bool isGroundColor(const Image::Pixel* pPixel) const;

  /**In most cases it is not possible to do a robot height sanity check if obstacles
   * are detected in the lower image. Those spots are buffered and handled by this
   * method once the next upper image comes around */
  void handleBufferedSpots(std::list<Vector2<int> >& possibleSpots);

  /**Handle nonLineSpots from this frame.*/
  void handleNewSpots(std::list<Vector2<int> >& possibleSpots);

  /**Remove all spots that are inside a ban zone from the possible spots*/
  void removeBanZone(std::list<Vector2<int> >& possibleSpots);

  /**Calculate the ban zone for a given point.
   * @param spot in image coordinates.*/
  void calcBanZone(const Vector2<int>& spot, int& outXLeft, int& outXRight, int& outYTop, int& outYBottom, int legHeight);

  /**sorts the possible spots using the y coordinate. Highest to lowest.*/
  void sortSpots(std::list<Vector2<int> >& possibleSpots);

  /**Use the 4 highest buffered spots to filter the possible spots.
   * This method should only be used in the lower image.
   * In very rare cases the hands of an obstacle are so far inside the lower image
   * that they become obstacles. However the feet are usually buffered spots because
   * their height leaves the image border. This method assumes that the 4 highest buffered
   * spots belong to the robots feet and uses them to create ban zones for the hands.
   *
   * @note this method partially sorts the spotBuffer
   */
  void removeLowerImageBanZone(std::list<Vector2<int> >& possibleSpots, std::vector<BufferedSpot>& spotBuffer);

  /**Square distance between two possible spots*/
  int squareDistance(const Vector2<int>& a, const Vector2<int>& b) const;

  /**Removes all spots from possibleSpots and clusters them. Save them in spots.obstacles.*/
  void clusterSpots(std::list<Vector2<int> >& possibleSpots, ObstacleSpots& spots) const;

  /**Extracts one cluster from the possibleSpots*/
  void extractCluster(std::list<Vector2<int> >& possibleSpots, ObstacleSpots::Obstacle& cluster) const;

  /**extract all spots that are close to the reference spot from possibleSpots and put
     them into the cluster. */
  void extractFitting(std::list<Vector2<int> >& possibleSpots, ObstacleSpots::Obstacle& cluster,
                      const Vector2<int>& reference) const;
  /**Calculate a ban zone for each cluster*/
  void calcClusterBanZone(ObstacleSpots& spots);
  /**Checks if a centerOfMass is inside the banzone of another obstacle. If yes it is removed*/
  void removeOverlappingBanZones(ObstacleSpots& spots) const;

  /**Use image coordinate system to correct spot location
     (remove rolling shutter distortion)*/
  void correctSpots(ObstacleSpots& spots);

  /**calculates distance from robot to spot in field coordinates*/
  float calculateDistanceTo(const Vector2<int>& spot) const;
  float calculateDistanceTo(const Vector2<float>& spot) const;

  /**Calculate the size of an object that is <B>distance<\B> away from the camera.
   * This method ignores the height an angle of the camera. It just assumes
   * that the object is directly in front of the camera. */
  int calculateLineSize(const float distance, const int size) const;

  /**Calculate center of mass for the specified cluster*/
  void calculateCenterOfMass(ObstacleSpots::Obstacle& cluster) const;

  /**returns y coordinate of horizon*/
  int calcHorizon() const;
  /**Remove all obstacles if one obstacle is too close to the bottom of the image */
  void checkIfTooCloseToBottom(ObstacleSpots& spots) const;


  /**Some spots are detected very close to the upper image border of the lower camera.
     If that is the case we cannot say for sure if they are obstacles or not.
     Those spots are stored in this vector until we receive the next image from the upper
     camera.*/
  std::vector<BufferedSpot> spotBuffer;
  int lastFrameImageHeight; /**< stores the camera image height from one frame before */

  /**Contains spots that might be obstacles. In image coordinates*/
  std::list<Vector2<int> > possibleSpots;
  /** counter used to remember that we saw a spot in the lower image a few frames ago :)*/
  int sawSpotsInLowerCamLastFrame;


};
