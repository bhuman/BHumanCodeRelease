/**
 * @file SimpleVehicleController
 * 
 * Controller for the demo specified in SimpleVehicle.ros2.
 * A simplistic car is equipped with a laser range finder, 
 * searches a ball and drives towards it.
 *
 * This demo includes:
 * - DepthImageSensor
 * - Hinge joint
 * - Controller3DDrawing
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 */
#define _USE_MATH_DEFINES // for C++

#include <SimRobotCore2.h>
#include <Platform/OpenGL.h>
#include <QString>
#include <QVector>
#include <cmath>

/**
* @class TestDrawing
* An object (a simple sphere) to be drawn by this controller.
*/
class TestDrawing : public SimRobotCore2::Controller3DDrawing
{
  void draw()
  {
    GLUquadricObj* q = gluNewQuadric();
    gluSphere(q, 0.5, 15, 15);
    gluDeleteQuadric(q);
  }
};


/**
* @class SimpleVehicleController
* The controller class for the SimpleVehicle demo
*/
class SimpleVehicleController : public SimRobot::Module
{
private:
  SimRobot::Application&   simRobot;            /** Reference to the SimRobot application */
  SimRobotCore2::ActuatorPort* frontLeftWheel;  /** Access to front left wheel */
  SimRobotCore2::ActuatorPort* frontRightWheel; /** Access to front right wheel */
  SimRobotCore2::ActuatorPort* backLeftWheel;   /** Access to back left wheel */
  SimRobotCore2::ActuatorPort* backRightWheel;  /** Access to back right wheel */
  SimRobotCore2::SensorPort*   distanceSensor;  /** Access to distance sensor */
  bool ballFound;                               /** Flag for behavior */
  float angleToBall;                            /** Angle for behavior */
  float distanceToBall;                         /** Distance for behavior */

  enum VehicleState
  {
    SEARCH_FOR_BALL = 0,
    GO_TO_BALL
  } vehicleState;         /** Different states for the robot behavior */


public:
  /** Constructor */
  SimpleVehicleController(SimRobot::Application& simRobot):simRobot(simRobot)
  {}


  /** Initializes drawing and objects for interfacing with actuators and sensor */
  bool compile() override
  {
    // Init 3D drawing, attach it to the ball
    TestDrawing* testDrawing = new TestDrawing();
    SimRobotCore2::Body* ballObj = (SimRobotCore2::Body*)simRobot.resolveObject("SimpleVehicle.Ball", SimRobotCore2::body);
    ballObj->registerDrawing(*testDrawing);

    // Get all necessary actuator and sensor objects
    SimRobotCore2::Object* vehicleObj = (SimRobotCore2::Object*)simRobot.resolveObject("SimpleVehicle.car", SimRobotCore2::body);
    QVector<QString> parts;
    parts.resize(1);
    parts[0] = "frontLeft.velocity";
    frontLeftWheel = (SimRobotCore2::ActuatorPort*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuatorPort);
    parts[0] = "frontRight.velocity";
    frontRightWheel = (SimRobotCore2::ActuatorPort*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuatorPort);
    parts[0] = "backLeft.velocity";
    backLeftWheel = (SimRobotCore2::ActuatorPort*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuatorPort);
    parts[0] = "backRight.velocity";
    backRightWheel = (SimRobotCore2::ActuatorPort*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuatorPort);
    parts[0] = "image";
    distanceSensor = (SimRobotCore2::SensorPort*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::sensorPort);
 
    // Init behavior member
    vehicleState = SEARCH_FOR_BALL;
    ballFound = false;
    simRobot.setStatusMessage("Initial search for ball.");
    return true;
  }


  /** This function is called in every execution cycle of the simulation*/
  void update() override
  {
    detectBall();
    // Oh behave:
    if(vehicleState == SEARCH_FOR_BALL)
    {
      frontLeftWheel->setValue((float) M_PI / 1.5f);
      frontRightWheel->setValue((float) -M_PI / 1.5f);
      backLeftWheel->setValue((float) M_PI / 1.5f);
      backRightWheel->setValue((float) -M_PI / 1.5f);
      if(ballFound)
      {
        simRobot.setStatusMessage("Ball detected. Driving to ball.");
        vehicleState = GO_TO_BALL;
      }
    }
    else if(vehicleState == GO_TO_BALL)
    {
      driveToBall();
      if(ballFound == false)
      {
        simRobot.setStatusMessage("Lost ball. Searching ball again.");
        vehicleState = SEARCH_FOR_BALL;
      }
    }
  }


  /** Tries to find the ball in the laser range finder data. Sets some members. */
  void detectBall()
  {
    ballFound = false;
    // Find closest measurement:
    const float* distanceData = distanceSensor->getValue().floatArray;
    float minDist = distanceData[0];
    int minDistIdx = 0;
    const int numOfDist = distanceSensor->getDimensions()[0];
    for(int i=1; i<numOfDist; ++i)
    {
      if(distanceData[i] < minDist)
      {
        minDist = distanceData[i];
        minDistIdx = i;
      }
    }
    // Compute relative ball position (if anything has been measured):
    float sensorMinDist, sensorMaxDist;
    distanceSensor->getMinAndMax(sensorMinDist, sensorMaxDist);
    if(minDist/sensorMaxDist < 0.9)
    {
      ballFound = true;
      const float openingAngle = (float) M_PI * 2.0f / 3.0f;
      angleToBall = openingAngle/2.0f - (minDistIdx+0.5f)*(openingAngle/numOfDist);
      distanceToBall = minDist;
    }
  }


  /** Drives to the ball. */
  void driveToBall()
  {
    if(angleToBall < -0.1f)
    {
      frontLeftWheel->setValue((float) M_PI / 1.5f);
      frontRightWheel->setValue((float) -M_PI / 1.5f);
      backLeftWheel->setValue((float) M_PI / 1.5f);
      backRightWheel->setValue((float) -M_PI / 1.5f);
    }
    else if(angleToBall > 0.1f)
    {
      frontLeftWheel->setValue((float) -M_PI / 1.5f);
      frontRightWheel->setValue((float) M_PI / 1.5f);
      backLeftWheel->setValue((float) -M_PI / 1.5f);
      backRightWheel->setValue((float) M_PI / 1.5f);
    }
    else //(ball is in front of robot)
    {
      frontLeftWheel->setValue((float) M_PI / 1.5f);
      frontRightWheel->setValue((float) M_PI / 1.5f);
      backLeftWheel->setValue((float) M_PI / 1.5f);
      backRightWheel->setValue((float) M_PI / 1.5f);
    }
  }
};


extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  return new SimpleVehicleController(simRobot);
}
