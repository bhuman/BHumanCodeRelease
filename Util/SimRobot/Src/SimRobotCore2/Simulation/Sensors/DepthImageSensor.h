/**
* @file Simulation/Sensors/DepthImageSensor.h
* Declaration of class DepthImageSensor
* @author Colin Graf
*/

#pragma once

#include "Simulation/Sensors/Sensor.h"

/**
* @class DepthImageSensor
* A simulated depth image sensor.
* It can simulate 2-D depth images such as the Kinect delivers
* (perspective projection), but also 1-D distance measurements
* as delivered by laser scanners (spherical projection). Image
* data is provided in rows from top to bottom and each row
* from left to right.
*/
class DepthImageSensor : public Sensor
{
public:
  unsigned int imageWidth; /**< The width of a depth image */
  unsigned int imageHeight; /**< The height of a depth image */
  float angleX;
  float angleY;
  float min;
  float max;

  enum Projection
  {
    perspectiveProjection,
    sphericalProjection
  } projection;

  /** Default constructor */
  DepthImageSensor();

private:
  /**
  * @class DistanceSensor
  * The distance sensor interface
  */
  class DistanceSensor : public Sensor::Port
  {
  public:
    ::PhysicalObject* physicalObject; /** The physical object were the distance sensor is mounted on */
    DepthImageSensor* depthImageSensor;
    float* imageBuffer; /**< A buffer for rendered image data */
    Pose3f offset; /**< Offset of the camera relative to the body it mounted on */
    float projection[16]; /**< The perspective projection matrix */
    float min; /**< Smallest measurable value in m. */
    float max; /**< Largest measurable value in m. */
    float* renderBuffer; /** The buffer used for rendering. Only differs from imageBuffer for spherical projection. */
    unsigned int renderWidth; /** The horizontal number of pixels to render. Only differs from depthImageSensor->imageWidth for spherical projection. */
    unsigned int renderHeight; /** The vertical number of pixels to render. Equals depthImageSensor->imageHeight. */
    float renderAngleX; /**< The horizontal opening angle of the render context. */
    float** lut; /**< Lookup table for transforming perspective projection to spherical projection. */
    unsigned int numOfBuffers; /**< Number of rending buffers. Actually, the same buffer is reused numOfBuffers times. */
    unsigned int bufferWidth; /**< The number of values in single buffer for multipart rendering. */

    /** Update the sensor value. Is called when required. */
    void updateValue() override;

    //API
    bool getMinAndMax(float& min, float& max) const override;
  } sensor;

  /** Destructor */
  ~DepthImageSensor();

  /** Initializes the camera after all attributes have been set */
  void createPhysics() override;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

  /** Registers this object with children, actuators and sensors at SimRobot's GUI. */
  void registerObjects() override;

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  void drawPhysics(unsigned int flags) const override;
};
