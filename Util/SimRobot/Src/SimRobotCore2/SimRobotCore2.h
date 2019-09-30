/**
* @file SimRobotCore2/SimRobotCore2.h
* Declaration of an interface to the SimRobotCore2
* @author Colin Graf
*/

#pragma once

#include "../SimRobot/SimRobot.h"

template<typename T> class QList;
class QStringList;

namespace SimRobotCore2
{
  class Renderer;
  class Controller3DDrawing;
  class Object;
  class PhysicalObject;
  class Body;
  class Appearance;
  class CollisionCallback;
  class Geometry;
  class Actuator;
  class Mass;
  class Sensor;
  class Compound;
  class Scene;
  class SensorPort;
  class ActuatorPort;

  /** The different SimRobotCore objet types */
  enum Kind
  {
    body = 2, /**< An object of the type SimRobotCore2::Body */
    appearance, /**< An object of the type SimRobotCore2::Appearance */
    geometry, /**< An object of the type SimRobotCore2::Geometry */
    actuator, /**< An object of the type SimRobotCore2::Actuator */
    mass, /**< An object of the type SimRobotCore2::Mass */
    sensor, /**< An object of the type SimRobotCore2::Sensor */
    compound, /**< An object of the type SimRobotCore2::Compound */
    scene, /**< An object of the type SimRobotCore2::Scene */
    actuatorPort, /**< An object of the type SimRobotCore2::Actuator */
    sensorPort, /**< An object of the type SimRobotCore2::Sensor */
  };

  /**
  * @class Renderer
  * An interface to a renderer that can be used to render objects on an OpenGL context
  */
  class Renderer
  {
  public:
    /** The shading technique used to draw appearances or physical representations */
    enum ShadeMode
    {
      noShading = 0,
      wireframeShading,
      flatShading,
      smoothShading,
    };

    /** A camera control mode */
    enum CameraMode
    {
      targetCam = 0,
      freeCam,
    };

    /** Specifies the current plane for Drag&Drop translations */
    enum DragAndDropPlane
    {
      xyPlane,
      xzPlane,
      yzPlane
    };

    /** The drag and drop modes for physical objects */
    enum DragAndDropMode
    {
      keepDynamics = 0,
      resetDynamics,
      adoptDynamics,
      applyDynamics
    };

    /** The kind of drag and drop to initiate */
    enum DragType
    {
      dragNormal, /**< drag for moving objects along the axis of the scene or for rotating the camera */
      dragNormalObject, /**< drag for moving object along their axis */
      dragRotate, /**< drag for rotating objects relative to their orientation */
      dragRotateWorld, /**< drag for rotating objects around the axis of the scene */
    };

    /** Flags enable or disable some render features */
    enum RenderFlags
    {
      enableLights           = (1 << 0),
      enableTextures         = (1 << 2),
      enableMultisample      = (1 << 3),
      showPhysics            = (1 << 4),
      showCoordinateSystem   = (1 << 5),
      showSensors            = (1 << 6),
      showControllerDrawings = (1 << 7),
      showAsGlobalView       = (1 << 8),
      enableDrawingsTransparentOcclusion = (1 << 9),
      enableDrawingsOcclusion = (1 << 10),
    };

    /** Virtual destructor */
    virtual ~Renderer() = default;

    /** Initializes the currently selected OpenGL context.
    * @parem hasSharedDisplayLists Whether the OpenGL has shared display lists and textures with another context that is already initialized.
    */
    virtual void init(bool hasSharedDisplayLists) = 0;

    /** Draws the scene object on the currently selected OpenGL context. */
    virtual void draw() = 0;

    /** Sets the size of the currently selected OpenGL renderer device. Call this once at the begining to initialize the size.
    * @param width The width of the renderer device
    * @param height The height of the renderer device
    */
    virtual void resize(float fovy, unsigned int width, unsigned int height) = 0;

    /**
    * Accesses the size of the OpenGL rendering device that has been set using \c resize before
    * @param width The width of the rendering device
    * @param width The height of the rendering device
    */
    virtual void getSize(unsigned int& width, unsigned int& height) const = 0;

    /**
    * Sets the ShadeMode that is used to render appearances.
    * @param shadeMode The shade mode
    */
    virtual void setSurfaceShadeMode(ShadeMode shadeMode) = 0;

    /**
    * Returns the ShadeMode used to render appearances
    * @return The mode currently used
    */
    virtual ShadeMode getSurfaceShadeMode() const = 0;

    /**
    * Sets the ShadeMode that is used to render physical primitives
    * @return The ShadeMode used to render physics
    */
    virtual void setPhysicsShadeMode(ShadeMode shadeMode) = 0;

    /**
    * Returns the ShadeMode used to render physics
    * @return The mode currently used
    */
    virtual ShadeMode getPhysicsShadeMode() const = 0;

    /**
    * Sets the ShadeMode that is used to render controller 3d drawings
    * @return The ShadeMode used to render physics
    */
    virtual void setDrawingsShadeMode(ShadeMode shadeMode) = 0;

    /**
    * Returns the ShadeMode used to controller 3d drawings
    * @return The mode currently used
    */
    virtual ShadeMode getDrawingsShadeMode() const = 0;

    /**
    * Sets the render flags to enable or disable some render features
    * @param The new render flags
    */
    virtual void setRenderFlags(unsigned int renderFlags) = 0;

    /**
    * Returns the render flags used to enable or disable some render features
    * @return The flags currently set
    */
    virtual unsigned int getRenderFlags() const = 0;

    virtual void zoom(float change, float x, float y) = 0;

    virtual void setCameraMode(CameraMode mode) = 0;
    virtual CameraMode getCameraMode() const = 0;
    virtual void toggleCameraMode() = 0;
    virtual void resetCamera() = 0;
    virtual void fitCamera() = 0; // not implemented

    virtual int getFovY() const = 0;

    virtual void setDragPlane(DragAndDropPlane plane) = 0;
    virtual DragAndDropPlane getDragPlane() const = 0;
    virtual void setDragMode(DragAndDropMode mode) = 0;
    virtual DragAndDropMode getDragMode() const = 0;

    virtual bool startDrag(int x, int y, DragType type) = 0;

    /**
    * Accesses the object currently manipulated with a drag & drop operation
    * @return The object
    */
    virtual Object* getDragSelection() = 0;

    virtual bool moveDrag(int x, int y, DragType type) = 0;
    virtual bool releaseDrag(int x, int y) = 0;

    /** Sets the camera moving state (useful for camera navigation with WASD keys)
    * @param left Whether the camera should move to the left
    * @param right Whether the camera should move to the right
    * @param up Whether the camera should move up
    * @param down Whether the camera should move down
    */
    virtual void setCameraMove(bool left, bool right, bool up, bool down) = 0;

    virtual void setCamera(const float* pos, const float* target) = 0;
    virtual void getCamera(float* pos, float* target) = 0;
    virtual void rotateCamera(float x, float y) = 0;
  };

  /**
  * This is an abstract base class for drawings, which can be implemented
  * inside the controller and executed while drawing the scene.
  */
  class Controller3DDrawing
  {
  public:
    /** Empty virtual destructor */
    virtual ~Controller3DDrawing() = default;

    /**
    * Virtual function for drawing commands. Derived classes have to
    * override this function.
    */
    virtual void draw() = 0;
  };

  /**
  * General interface for all renderable scene graph objects
  */
  class Object : public SimRobot::Object
  {
  public:
    /**
    * Creates a new instance of a renderer that can be used for rendering
    * an object within an OpenGL context.
    */
    virtual Renderer* createRenderer() = 0;
  };

  /**
  * Interface for scene graph objects with physical representation
  */
  class PhysicalObject : public Object
  {
  public:
    /**
    * Registers controller drawings at an object in the simulation scene
    * @param drawing The drawing
    */
    virtual bool registerDrawing(Controller3DDrawing& drawing) = 0;

    /**
    * Unregisters controller drawings at an object in the simulation scene
    * @param drawing The drawing
    */
    virtual bool unregisterDrawing(Controller3DDrawing& drawing) = 0;

    /**
    * Accesses a superior body object
    * @return The body object (might be 0)
    */
    virtual Body* getParentBody() = 0;
  };

  /**
  * Interface to bodies
  */
  class Body : public PhysicalObject
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return body;}

    /**
    * Returns the position of the object
    * @return The position
    */
    virtual const float* getPosition() const = 0;

    /**
    * Returns the pose of the object in the three-dimensional space
    * @param position A buffer for the position vector
    * @param rotation A buffer for the rotation matrix (3x3)
    * @return Whether the pose has successfully been computed
    */
    virtual bool getPose(float* position, float (*rotation)[3]) const = 0;

    /**
    * Moves the  object to target position.
    * @param object The object to move.
    * @param position The target position.
    */
    virtual void move(const float* position) = 0;

    /**
    * Moves the object to target position and rotation specified as 3x3 rotation matrix.
    * @param object The object to move.
    * @param position The target position.
    * @param rotation The target rotation.
    */
    virtual void move(const float* position, const float (*rotation)[3]) = 0;

    /**
    * Resets the linear and angular velocity of all bodies connected to this object
    */
    virtual void resetDynamics() = 0;

    /**
    * Accesses the first body in the chain of bodies to which this body is connected
    * @return The body object
    */
    virtual Body* getRootBody() = 0;

    /**
     * Enables or disables the physics simulation of the body
     * @param enable Whether to enable or disable the physics simulation
     */
    virtual void enablePhysics(bool enable) = 0;
  };

  /**
  * Interface to appearances
  */
  class Appearance : public Object
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return appearance;}

    /**
     * Registers controller drawings at an object in the simulation scene
     * @param drawing The drawing
     */
    virtual bool registerDrawing(Controller3DDrawing& drawing) = 0;

    /**
     * Unregisters controller drawings at an object in the simulation scene
     * @param drawing The drawing
     */
    virtual bool unregisterDrawing(Controller3DDrawing& drawing) = 0;
  };

  /*
  * An interface for a collision callback function
  */
  class CollisionCallback
  {
  public:
    /**
    * The callback function.
    * Called whenever the geometry at which this interface is registered collides with another geometry.
    * @param geom1 The geometry at which the interface has been registered
    * @param geom2 The other geometry
    */
    virtual void collided(Geometry& geom1, Geometry& geom2) = 0;
  };

  /**
  * Interface to geometries
  */
  class Geometry : public PhysicalObject
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return geometry;}

    /**
    * Registers a collision callback function that will be called whenever the geometry
    * collides with a geometry.
    * @param collisionCallback A class in which the CollisionCallback is implemented
    * @return Whether the collision callback function was successfully registered
    */
    virtual bool registerCollisionCallback(CollisionCallback& collisionCallback) = 0;

    /**
    * Unregisters a collision callback function.
    * @param collisionCallback The callback function to unregister
    * @return Whether the collision callback function has successfully been unregistered
    */
    virtual bool unregisterCollisionCallback(CollisionCallback& collisionCallback) = 0;
  };

  /**
  * Interface to actuators (e.g. joints)
  */
  class Actuator : public PhysicalObject
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return actuator;}
  };

  /**
  * Interface to masses
  */
  class Mass : public Object
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return mass;}
  };

  /**
  * Interface to sensors
  */
  class Sensor : public PhysicalObject
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return sensor;}
  };

  /**
  * Interface to compounds
  */
  class Compound : public PhysicalObject
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return compound;}
  };

  /**
  * Interface to the scene (root of the scene graph)
  */
  class Scene : public PhysicalObject
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return scene;}

    /** Returns the length of one simulation step
    * @return The time which is simulated by one step (in s)
    */
    virtual double getStepLength() const = 0;

    /** Returns the current simulation step
    * @return The step
    */
    virtual unsigned int getStep() const = 0;

    /** Returns the current simulation time in seconds, starting with 0.0
    * @return The time (in s)
    */
    virtual double getTime() const = 0;

    /** Returns the current frame rate
    * @return The frame rate in frames per second
    */
    virtual unsigned int getFrameRate() const = 0;
  };

  /**
  * Interface to sensor ports
  */
  class SensorPort : public SimRobot::Object
  {
  public:
    enum SensorType
    {
      boolSensor,
      floatSensor,
      cameraSensor,
      floatArraySensor,
      noSensor
    };

    union Data
    {
      bool boolValue;
      float floatValue;
      const float* floatArray;
      const unsigned char* byteArray;
    };

    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return sensorPort;}

    virtual SensorType getSensorType() const = 0;
    virtual Data getValue() = 0;
    virtual bool getMinAndMax(float& min, float& max) const = 0;
    virtual const QList<int>& getDimensions() const = 0;
    virtual const QStringList& getDescriptions() const = 0;

    /**
    * Returns the name of the unit of the sensor reading(s)
    * @return The name
    */
    virtual const QString& getUnit() const = 0;

    /**
    * Pre-renders the images of multiple camera sensors of the same type at once which improves the performance of camera image rendering.
    * @param cameras An array of camera sensors
    * @param count The amount of camera sensors in the array
    */
    virtual bool renderCameraImages(SensorPort** cameras, unsigned int count) = 0;
  };

  /**
  * Interface to actuator ports
  */
  class ActuatorPort : public SimRobot::Object
  {
  public:
    /**
    * Returns an object type identifier
    * @return The identifier
    */
    int getKind() const override {return actuatorPort;}

    virtual void setValue(float value) = 0;
    virtual bool getMinAndMax(float& min, float& max) const = 0;

    /**
    * Returns the name of the unit of the actuator's setpoint
    * @return The name
    */
    virtual const QString& getUnit() const = 0;
  };
}
