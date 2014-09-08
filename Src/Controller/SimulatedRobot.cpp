/**
 * @file Controller/SimulatedRobot.cpp
 * Implementation of class SimulatedRobot for SimRobotQt.
 * @author Colin Graf
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SimulatedRobot.h"
#include "Controller/TeamComm3DCtrl.h"

#include <QString>
#include <QVector>

#include <algorithm>

#include "Controller/RoboCupCtrl.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Tools/Streams/InStreams.h"

SimRobotCore2::SensorPort* SimulatedRobot::activeCameras[14]; // should be 12, but TeamComm3DCtrl allocates two unsued ones
unsigned SimulatedRobot::activeCameraCount = 0;

SimRobot::Object* SimulatedRobot::ball = 0;

SimulatedRobot::SimulatedRobot() :
  blue(false), robot(0), leftFoot(0), rightFoot(0), lastTimeStamp(0), activeCameraIndex(activeCameraCount++)
{
  ASSERT(activeCameraCount <= sizeof(activeCameras) / sizeof(activeCameras[0]));
}

SimulatedRobot::~SimulatedRobot()
{
  --activeCameraCount;
}

void SimulatedRobot::init(SimRobot::Object* robot)
{
  ASSERT(this->robot == 0);
  this->robot = (SimRobotCore2::Object*)robot;
  application = RoboCupCtrl::application ? RoboCupCtrl::application : TeamComm3DCtrl::application;

  // get the robot's team color and number
  blue = isBlue(robot);
  robotNumber = getNumber(robot);
  bool exists;
  // load camera Parameters
  InMapFile intrStream("cameraIntrinsics.cfg");
  VERIFY(exists = intrStream.exists());
  intrStream >> cameraIntrinsics;
  InMapFile resStream("cameraResolution.cfg");
  VERIFY(exists = resStream.exists());
  resStream >> cameraResolution;

  // build cameraInfo
  switch(cameraResolution.resolution)
  {
    case CameraResolution::Resolutions::upper640:
      upperCameraInfo.width = 640;
      upperCameraInfo.height = 480;
      lowerCameraInfo.width = 320;
      lowerCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::lower640:
      upperCameraInfo.width = 320;
      upperCameraInfo.height = 240;
      lowerCameraInfo.width = 640;
      lowerCameraInfo.height = 480;
      break;
    case CameraResolution::Resolutions::both320:
      upperCameraInfo.width = 320;
      upperCameraInfo.height = 240;
      lowerCameraInfo.width = 320;
      lowerCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::both640:
      upperCameraInfo.width = 640;
      upperCameraInfo.height = 480;
      lowerCameraInfo.width = 640;
      lowerCameraInfo.height = 480;
      break;
    default:
      ASSERT(false);
      break;
  }

  upperCameraInfo.camera = CameraInfo::upper;
  lowerCameraInfo.camera = CameraInfo::lower;
  // set opening angle
  upperCameraInfo.openingAngleWidth = cameraIntrinsics.upperOpeningAngleWidth;
  upperCameraInfo.openingAngleHeight = cameraIntrinsics.upperOpeningAngleHeight;
  lowerCameraInfo.openingAngleWidth = cameraIntrinsics.lowerOpeningAngleWidth;
  lowerCameraInfo.openingAngleHeight = cameraIntrinsics.lowerOpeningAngleHeight;
  // set optical center
  upperCameraInfo.opticalCenter.x = cameraIntrinsics.upperOpticalCenter.x * upperCameraInfo.width;
  upperCameraInfo.opticalCenter.y = cameraIntrinsics.upperOpticalCenter.y * upperCameraInfo.height;
  lowerCameraInfo.opticalCenter.x = cameraIntrinsics.lowerOpticalCenter.x * lowerCameraInfo.width;
  lowerCameraInfo.opticalCenter.y = cameraIntrinsics.lowerOpticalCenter.y * lowerCameraInfo.height;
  // update focal length
  upperCameraInfo.updateFocalLength();
  lowerCameraInfo.updateFocalLength();

  // get feet (for pose and odometry)
  QVector<QString> parts;
  parts.resize(1);
  parts[0] = "RFoot";
  VERIFY(rightFoot = (SimRobotCore2::Body*)application->resolveObject(parts, robot, SimRobotCore2::body));
  parts[0] = "LFoot";
  VERIFY(leftFoot = (SimRobotCore2::Body*)application->resolveObject(parts, robot, SimRobotCore2::body));

  // get joints
  parts.resize(1);
  QString position(".position");
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    parts[0] = QString(JointData::getName(JointData::Joint(i))) + position;
    VERIFY(jointSensors[i] = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
    VERIFY(jointActuators[i] = (SimRobotCore2::ActuatorPort*)application->resolveObject(parts, robot, SimRobotCore2::actuatorPort));
  }

  // imu sensors
  parts.resize(1);
  parts[0] = "Gyroscope.angularVelocities";
  gyroSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);

  parts[0] = "Accelerometer.acceleration";
  accSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);

  // cameras
  parts[0] = "CameraTop.image";
  upperCameraSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "CameraBottom.image";
  lowerCameraSensor = application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  cameraSensor = lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::SensorPort*)cameraSensor;

  // sonars
  parts[0] = "SonarLeft.distance";
  leftUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "SonarRight.distance";
  rightUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "SonarCenterLeft.distance";
  centerLeftUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);
  parts[0] = "SonarCenterRight.distance";
  centerRightUsSensor = (SimRobotCore2::SensorPort*)application->resolveObject(parts, robot, SimRobotCore2::sensorPort);

  // load calibration
  InMapFile stream("jointCalibration.cfg");
  ASSERT(stream.exists());
  stream >> jointCalibration;

  // fill arrays with pointers to other robots
  blueRobots.clear();
  redRobots.clear();
  // Parse "robots" group:
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::compound);
  for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = (SimRobot::Object*)application->getObjectChild(*group, currentRobot);
    const int number = getNumber(robot);
    if(number != robotNumber)
    {
      if(number <= 6)
        blueRobots.push_back(robot);
      else
        redRobots.push_back(robot);
    }
  }
  // Parse "extras" group:
  group = application->resolveObject("RoboCup.extras", SimRobotCore2::compound);
  for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = (SimRobot::Object*)application->getObjectChild(*group, currentRobot);
    const int number = getNumber(robot);
    if(number != robotNumber)
    {
      if(number <= 6)
        blueRobots.push_back(robot);
      else
        redRobots.push_back(robot);
    }
  }
}

void SimulatedRobot::setBall(SimRobot::Object* ball)
{
  SimulatedRobot::ball = ball;
}

void SimulatedRobot::getRobotPose(Pose2D& robotPose) const
{
  ASSERT(rightFoot && leftFoot);

  getPose2D(robot, robotPose);
  robotPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;

  if(blue)
    robotPose = Pose2D(pi) + robotPose;
}

void SimulatedRobot::getWorldState(GroundTruthWorldState& worldState) const
{
  // Initialize world state
  worldState.blueRobots.clear();
  worldState.redRobots.clear();
  worldState.balls.clear();

  // Get the standard ball position from the scene
  if(ball)
  {
    Vector2<> ballPosition = getPosition(ball);
    if(blue)
      ballPosition = -ballPosition;
    worldState.balls.push_back(ballPosition);
  }

  // Determine the robot's own pose and number
  getPose2D(robot, worldState.ownPose);
  worldState.ownPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;
  if(blue)
    worldState.ownPose = Pose2D(pi) + worldState.ownPose;

  // Add all other robots that are in this scene
  for(unsigned int i = 0; i < blueRobots.size(); ++i)
  {
    GroundTruthWorldState::GroundTruthRobot newGTRobot;
    newGTRobot.number = getNumber(blueRobots[i]);
    getPose2D(blueRobots[i], newGTRobot.pose);
    if(blue)
      newGTRobot.pose = Pose2D(pi) + newGTRobot.pose;
    worldState.blueRobots.push_back(newGTRobot);
  }
  for(unsigned int i = 0; i < redRobots.size(); ++i)
  {
    GroundTruthWorldState::GroundTruthRobot newGTRobot;
    newGTRobot.number = getNumber(redRobots[i]) - 6;
    getPose2D(redRobots[i], newGTRobot.pose);
    if(blue)
      newGTRobot.pose = Pose2D(pi) + newGTRobot.pose;
    worldState.redRobots.push_back(newGTRobot);
  }
}

void SimulatedRobot::getOdometryData(const Pose2D& robotPose, OdometryData& odometryData) const
{
  ASSERT(robot);
  (Pose2D&)odometryData = blue ? (Pose2D(pi) + robotPose) : robotPose;
}

void SimulatedRobot::getAbsoluteBallPosition(Vector2<>& ballPosition)
{
  if(ball)
  {
    ballPosition = getPosition(ball);
  }
}

void SimulatedRobot::getImage(Image& image, CameraInfo& cameraInfo)
{
  ASSERT(robot);

  if(cameraSensor)
  {
    ((SimRobotCore2::SensorPort*)cameraSensor)->renderCameraImages(activeCameras, activeCameraCount);

    ASSERT(!image.isReference);
    if(cameraSensor == upperCameraSensor)
      cameraInfo = upperCameraInfo;
    else
      cameraInfo = lowerCameraInfo;

    image.setResolution(cameraInfo.width, cameraInfo.height, false);
    const int w = image.width;
    const int h = image.height;

    const int w3 = w * 3, w2 = int(image[1] - image[0]);
    unsigned char* src = (unsigned char*)((SimRobotCore2::SensorPort*)cameraSensor)->getValue().byteArray;
    unsigned char* srcLineEnd = src;
    Image::Pixel* destBegin = image[0];
    Image::Pixel* dest;
    int b1, g1, r1, yy, cr;

    for(int y = h - 1; y >= 0; --y)
    {
      for(srcLineEnd += w3, dest = destBegin + y * w2; src < srcLineEnd;)
      {
        for(int i = 0; i < 4; ++i)
        {
          yy = 306 * (b1 = *src++);
          cr = 130560 - 429 * (g1 = *src++) + 512 * b1;
          dest->cb = (unsigned char)((130560 - 173 * b1 - 339 * g1 + 512 * (r1 = *src++)) >> 10);
          yy += 117 * r1 + 601 * g1;
          cr -= 83 * r1;
          dest->y = dest->yCbCrPadding = (unsigned char)(yy >> 10);
          (dest++)->cr = (unsigned char)(cr >> 10);
        }
      }
    }
  }

  image.timeStamp = SystemCall::getCurrentSystemTime();
}

void SimulatedRobot::getCameraInfo(CameraInfo& cameraInfo)
{
  if(cameraSensor)
  {
    if(cameraSensor == upperCameraSensor)
      cameraInfo = upperCameraInfo;
    else
      cameraInfo = lowerCameraInfo;
  }
}

void SimulatedRobot::getAndSetJointData(const JointData& jointRequest, JointData& jointData) const
{
  ASSERT(robot);

  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    // Get angles
    if(jointSensors[i])
      jointData.angles[i] = float(((SimRobotCore2::SensorPort*)jointSensors[i])->getValue().floatValue * jointCalibration.joints[i].sign - jointCalibration.joints[i].offset);

    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if(targetAngle != JointData::off &&
       targetAngle != JointData::ignore &&
       jointActuators[i]) // if joint does exist
      ((SimRobotCore2::ActuatorPort*)jointActuators[i])->setValue((targetAngle + jointCalibration.joints[i].offset) * jointCalibration.joints[i].sign);
  }
  jointData.timeStamp = SystemCall::getCurrentSystemTime();
}

void SimulatedRobot::setJointData(const JointData& jointRequest) const
{
  ASSERT(robot);
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if(targetAngle != JointData::off &&
       targetAngle != JointData::ignore &&
       jointActuators[i]) // if joint does exist
      ((SimRobotCore2::ActuatorPort*)jointActuators[i])->setValue((targetAngle + jointCalibration.joints[i].offset) * jointCalibration.joints[i].sign);
  }
}

void SimulatedRobot::toggleCamera()
{
  cameraSensor = cameraSensor == lowerCameraSensor ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::SensorPort*)cameraSensor;
}

void SimulatedRobot::getSensorData(SensorData& sensorData, const USRequest& usRequest)
{
  ASSERT(robot);

  sensorData.timeStamp = SystemCall::getCurrentSystemTime();

  // Gyro
  const float* floatArray = ((SimRobotCore2::SensorPort*)gyroSensor)->getValue().floatArray;
  sensorData.data[SensorData::gyroX] = floatArray[0] + 12.f; // 12.f = bias
  sensorData.data[SensorData::gyroY] = floatArray[1] + 6.f; // 6.f = bias
  sensorData.data[SensorData::gyroZ] = 0.f; //float(doubleArray[2]); // nao style :(

  // Acc
  floatArray = ((SimRobotCore2::SensorPort*)accSensor)->getValue().floatArray;
  sensorData.data[SensorData::accX] = -1 * floatArray[0]  + 0.1f; // 0.1f = bias
  sensorData.data[SensorData::accY] = -1 * floatArray[1]  + 0.2f; // 0.2f = bias
  sensorData.data[SensorData::accZ] = -1 * floatArray[2]  + 0.05f; // 0.05f = bias

  // angle
  {
    float position[3];
    float world2robot[3][3];
    ((SimRobotCore2::Body*)robot)->getPose(position, world2robot);
    sensorData.data[SensorData::angleX] = float(atan2(world2robot[1][2], world2robot[2][2]));
    sensorData.data[SensorData::angleY] = -float(atan2(world2robot[0][2], world2robot[2][2]));
  }

  // Battery
  sensorData.data[SensorData::batteryLevel] = 1.0f;

  // ultrasonic (model approximation. not absolutely correct in reality)

  static const float scale = 1000; //meter to millimeter
  if(usRequest.receiveMode != -1)
  {
    sensorData.usActuatorMode = (SensorData::UsActuatorMode) usRequest.receiveMode;

    //FIXME simulate additional sensor values
    for(int i = SensorData::usL; i < SensorData::usREnd; ++i)
      sensorData.data[i] = 2550.f;

    switch(sensorData.usActuatorMode)
    {
      case SensorData::leftToLeft:
      {
        float leftUsValue = float(((SimRobotCore2::SensorPort*)leftUsSensor)->getValue().floatValue * scale);
        sensorData.data[SensorData::us] = addJitter(leftUsValue);
        break;
      }
      case SensorData::leftToRight:
      {
        float centerLeftUsValue = float(((SimRobotCore2::SensorPort*)centerLeftUsSensor)->getValue().floatValue * scale);
        sensorData.data[SensorData::us] = addJitter(centerLeftUsValue);
        break;
      }
      case SensorData::rightToLeft:
      {
        float centerRightUsValue = float(((SimRobotCore2::SensorPort*)centerRightUsSensor)->getValue().floatValue * scale);
        sensorData.data[SensorData::us] = addJitter(centerRightUsValue);
        break;
      }
      case SensorData::rightToRight:
      {
        float rightUsValue = float(((SimRobotCore2::SensorPort*)rightUsSensor)->getValue().floatValue * scale);
        sensorData.data[SensorData::us] = addJitter(rightUsValue);
        break;
      }
      case SensorData::bothToSame:
      {
        float leftUsValue = float(((SimRobotCore2::SensorPort*)leftUsSensor)->getValue().floatValue * scale);
        float rightUsValue = float(((SimRobotCore2::SensorPort*)rightUsSensor)->getValue().floatValue * scale);
        sensorData.data[SensorData::usL] = addJitter(leftUsValue);
        sensorData.data[SensorData::us] = addJitter(rightUsValue);
        break;
      }
      case SensorData::bothToOther:
      {
        float centerLeftUsValue = float(((SimRobotCore2::SensorPort*)centerLeftUsSensor)->getValue().floatValue * scale);
        float centerRightUsValue = float(((SimRobotCore2::SensorPort*)centerRightUsSensor)->getValue().floatValue * scale);
        sensorData.data[SensorData::usL] = addJitter(centerLeftUsValue);
        sensorData.data[SensorData::usR] = addJitter(centerRightUsValue);
        break;
      }
      default:
        ASSERT(false);
    }
    sensorData.usTimeStamp = sensorData.timeStamp;
  }
}

void SimulatedRobot::getOrientationData(const SensorData& sensorData, OrientationData& orientationData)
{
  ASSERT(robot);
  getPose3D(robot, robotPose3D);
  const Pose3D offset(lastRobotPose3D.invert().conc(robotPose3D));
  orientationData.rotation = robotPose3D.rotation;
  float timeScale = 1.f / (float(sensorData.timeStamp - lastTimeStamp) * 0.001f);
  orientationData.velocity.x = float(offset.translation.y * timeScale);
  orientationData.velocity.y = -float(offset.translation.x * timeScale);
  orientationData.velocity.z = float(offset.translation.z * timeScale);
  lastRobotPose3D = robotPose3D;
  lastTimeStamp = sensorData.timeStamp;
}

void SimulatedRobot::moveRobot(const Vector3<>& pos, const Vector3<>& rot, bool changeRotation)
{
  ASSERT(robot);

  Vector3<> position = pos * 0.001f;
  if(changeRotation)
  {
    RotationMatrix rotation(rot);
    float rotation2[3][3];
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        rotation2[i][j] = rotation[i][j];
    ((SimRobotCore2::Body*)robot)->move(&position.x, rotation2);
  }
  else
    ((SimRobotCore2::Body*)robot)->move(&position.x);
}

void SimulatedRobot::enablePhysics(bool enable)
{
  ((SimRobotCore2::Body*)robot)->enablePhysics(enable);
}

void SimulatedRobot::moveBall(const Vector3<>& pos, bool resetDynamics)
{
  Vector3<> position = pos * 0.001f;
  ((SimRobotCore2::Body*)ball)->move(&position.x);
  if(resetDynamics)
    ((SimRobotCore2::Body*)ball)->resetDynamics();
}

Vector2<> SimulatedRobot::getPosition(SimRobot::Object* obj)
{
  const float* position = ((SimRobotCore2::Body*)obj)->getPosition();
  return Vector2<>(position[0] * 1000.f, position[1] * 1000.f);
}

void SimulatedRobot::getPose2D(SimRobot::Object* obj, Pose2D& pose2D) const
{
  float position[3];
  float rot3d[3][3];
  ((SimRobotCore2::Body*)obj)->getPose(position, rot3d);

  pose2D.translation.x = position[0] * 1000.f;
  pose2D.translation.y = position[1] * 1000.f;

  // compute z-rotation

  /*
  Vector3<> d = Vector3<>(-rot3d[0][2], -rot3d[1][2], rot3d[2][2]);
  Vector3<> g = Vector3<>(0, 0, 1.f) ^ d;
  float w = atan2(sqrt(d.x * d.x + d.y * d.y), d.z);
  RotationMatrix withoutZ(g, w);
  RotationMatrix zOnly = RotationMatrix(Vector3<>(rot3d[0][0], rot3d[0][1], rot3d[0][2]), Vector3<>(rot3d[1][0], rot3d[1][1], rot3d[1][2]), Vector3<>(rot3d[2][0], rot3d[2][1], rot3d[2][2])) * withoutZ.invert();
  pose2D.rotation = atan2(zOnly.c0.y, zOnly.c0.x);
  */

  // (this is an optimized version of the code above)
  float x = rot3d[1][2], y = -rot3d[0][2];
  const float z = rot3d[2][2];
  const float gLenSqr = x * x + y * y;
  const float gLen = std::sqrt(gLenSqr);
  const float wLen = std::sqrt(gLenSqr + z * z);
  if(gLen != 0.f)
  {
    x /= gLen;
    y /= gLen;
  }
  const float si = -gLen / wLen, co = z / wLen;
  const float v = 1 - co;
  const float d0x = x * x * v + co;
  const float d0y = x * y * v;
  const float d0z = -y * si;
  const float c0x = rot3d[0][0] * d0x + rot3d[1][0] * d0y + rot3d[2][0] * d0z;
  const float c0y = rot3d[0][1] * d0x + rot3d[1][1] * d0y + rot3d[2][1] * d0z;
  pose2D.rotation = std::atan2(c0y, c0x);
}

void SimulatedRobot::getPose3D(SimRobot::Object* obj, Pose3D& pose3D) const
{
  float rotation[3][3];
  ((SimRobotCore2::Body*)obj)->getPose(&pose3D.translation.x, rotation);

  pose3D.translation *= 1000.f;
  pose3D.rotation.c0.x = rotation[0][0];
  pose3D.rotation.c0.y = rotation[0][1];
  pose3D.rotation.c0.z = rotation[0][2];
  pose3D.rotation.c1.x = rotation[1][0];
  pose3D.rotation.c1.y = rotation[1][1];
  pose3D.rotation.c1.z = rotation[1][2];
  pose3D.rotation.c2.x = rotation[2][0];
  pose3D.rotation.c2.y = rotation[2][1];
  pose3D.rotation.c2.z = rotation[2][2];
}

float SimulatedRobot::addJitter(float value)
{
  if(value >= 1400.f) // nothing was measured
    return value;
  else
  {
    value = std::max(260.f, value);
    float randValue = (float)(0.1f * (rand() % 100) + -0.1f * (rand() % 100));

    if(rand() % 100 <= 10)
    {
      return 2200;
    }
    else
    {
      return value + randValue;
    }
  }
}

bool SimulatedRobot::isBlue(SimRobot::Object* obj)
{
  return getNumber(obj) <= 6;
}

int SimulatedRobot::getNumber(SimRobot::Object* obj)
{
  QString robotNumberString(obj->getFullName());
  int pos = robotNumberString.lastIndexOf('.');
  robotNumberString.remove(0, pos + 6);
  return robotNumberString.toInt();
}
