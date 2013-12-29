/**
* @file Simulation/Sensors/Camera.h
* Declaration of class Camera
* @author Colin Graf
*/

#pragma once

#include "Simulation/Sensors/Sensor.h"

/**
* @class Camera
* A simulated camera
*/
class Camera : public Sensor
{
public:
  unsigned int imageWidth; /**< The width of a camera image */
  unsigned int imageHeight; /**< The height of a camera image */
  float angleX;
  float angleY;

  /** Default constructor */
  Camera();

private:
  /**
  * @class CameraSensor
  * The camera sensor interface
  */
  class CameraSensor : public Sensor::Port
  {
  public:
    ::PhysicalObject* physicalObject; /** The physical object were the camera is mounted on */
    Camera* camera;
    unsigned char* imageBuffer; /**< A buffer for rendered image data */
    unsigned int imageBufferSize;
    Pose3<> offset; /**< Offset of the camera relative to the body it mounted on */
    float projection[16]; /**< The perspective projection matrix */

    /** Update the sensor value. Is called when required. */
    virtual void updateValue();

    //API
    virtual bool getMinAndMax(float& min, float& max) const {min = 0; max = 0xff; return true;}
    virtual bool renderCameraImages(SimRobotCore2::SensorPort** cameras, unsigned int count);
  } sensor;

  /** Destructor */
  ~Camera();

  /** Initializes the camera after all attributes have been set */
  virtual void createPhysics();

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  /** Registers this object with children, actuators and sensors at SimRobot's GUI */
  virtual void registerObjects();

  /** Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;
};
