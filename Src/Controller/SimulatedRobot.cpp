/**
 * @file Controller/SimulatedRobot.cpp
 * Implementation of class SimulatedRobot for SimRobotQt.
 * @author Colin Graf
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SimulatedRobot.h"

#include <QString>
#include <QVector>

#include <algorithm>

#include "Controller/RoboCupCtrl.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/UsSensorData.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Streams/InStreams.h"

SimRobotCore2::SensorPort* SimulatedRobot::activeCameras[12];
unsigned SimulatedRobot::activeCameraCount = 0;

SimRobot::Object* SimulatedRobot::ball = nullptr;

SimulatedRobot::SimulatedRobot() :
  blue(false), robot(0), leftFoot(0), rightFoot(0), activeCameraIndex(activeCameraCount++)
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
  this->robot = dynamic_cast<SimRobotCore2::Object*>(robot);
  application = RoboCupCtrl::application;

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
  upperCameraInfo.opticalCenter.x() = cameraIntrinsics.upperOpticalCenter.x() * upperCameraInfo.width;
  upperCameraInfo.opticalCenter.y() = cameraIntrinsics.upperOpticalCenter.y() * upperCameraInfo.height;
  lowerCameraInfo.opticalCenter.x() = cameraIntrinsics.lowerOpticalCenter.x() * lowerCameraInfo.width;
  lowerCameraInfo.opticalCenter.y() = cameraIntrinsics.lowerOpticalCenter.y() * lowerCameraInfo.height;
  // update focal length
  upperCameraInfo.updateFocalLength();
  lowerCameraInfo.updateFocalLength();

  // get feet (for pose and odometry)
  QVector<QString> parts;
  parts.resize(1);
  parts[0] = "RFoot";
  VERIFY(rightFoot = dynamic_cast<SimRobotCore2::Body*>(application->resolveObject(parts, robot, SimRobotCore2::body)));
  parts[0] = "LFoot";
  VERIFY(leftFoot = dynamic_cast<SimRobotCore2::Body*>(application->resolveObject(parts, robot, SimRobotCore2::body)));

  // get joints
  parts.resize(1);
  QString position(".position");
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    parts[0] = QString(Joints::getName(static_cast<Joints::Joint>(i))) + position;
    parts[0] = QString(parts[0].left(1)).toUpper() + parts[0].mid(1);
    jointSensors[i] = dynamic_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
    jointActuators[i] = dynamic_cast<SimRobotCore2::ActuatorPort*>(application->resolveObject(parts, robot, SimRobotCore2::actuatorPort));
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
  activeCameras[activeCameraIndex] = dynamic_cast<SimRobotCore2::SensorPort*>(cameraSensor);

  // sonars
  parts[0] = "SonarLeft.distance";
  leftUsSensor = dynamic_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
  parts[0] = "SonarRight.distance";
  rightUsSensor = dynamic_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
  parts[0] = "SonarCenterLeft.distance";
  centerLeftUsSensor = dynamic_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
  parts[0] = "SonarCenterRight.distance";
  centerRightUsSensor = dynamic_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));

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
    SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
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
    SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
    const int number = getNumber(robot);
    if(number != robotNumber)
    {
      if(number <= 6)
        blueRobots.push_back(robot);
      else
        redRobots.push_back(robot);
    }
  }

  // read UsConfiguration
  InMapFile usStream("usConfiguration.cfg");
  ASSERT(usStream.exists());
  usStream >> usConfig;
}

void SimulatedRobot::setBall(SimRobot::Object* ball)
{
  SimulatedRobot::ball = ball;
}

void SimulatedRobot::getRobotPose(Pose2f& robotPose) const
{
  ASSERT(rightFoot && leftFoot);

  getPose2f(robot, robotPose);
  robotPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;

  if(blue)
    robotPose = Pose2f(pi) + robotPose;
}

void SimulatedRobot::getWorldState(GroundTruthWorldState& worldState) const
{
  // Initialize world state
  worldState.bluePlayers.clear();
  worldState.redPlayers.clear();
  worldState.balls.clear();

  // Get the standard ball position from the scene
  if(ball)
  {
    Vector2f ballPosition = getPosition(ball);
    if(blue)
      ballPosition = -ballPosition;
    worldState.balls.push_back(ballPosition);
  }

  // Determine the robot's own pose and number
  Pose2f tmp;
  getPose2f(robot, tmp);
  worldState.ownPose = tmp;
  worldState.ownPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;
  if(blue)
    worldState.ownPose = Pose2f(pi) + worldState.ownPose;

  // Add all other robots that are in this scene
  for(unsigned int i = 0; i < blueRobots.size(); ++i)
  {
    GroundTruthWorldState::GroundTruthPlayer newGTPlayer;
    newGTPlayer.number = getNumber(blueRobots[i]);
    newGTPlayer.upright = getPose2f(blueRobots[i], tmp);
    newGTPlayer.pose = tmp;
    if(blue)
      newGTPlayer.pose = Pose2f(pi) + newGTPlayer.pose;
    worldState.bluePlayers.push_back(newGTPlayer);
  }
  for(unsigned int i = 0; i < redRobots.size(); ++i)
  {
    GroundTruthWorldState::GroundTruthPlayer newGTPlayer;
    newGTPlayer.number = getNumber(redRobots[i]) - 6;
    newGTPlayer.upright = getPose2f(redRobots[i], tmp);
    newGTPlayer.pose = tmp;
    if(blue)
      newGTPlayer.pose = Pose2f(pi) + newGTPlayer.pose;
    worldState.redPlayers.push_back(newGTPlayer);
  }
}

void SimulatedRobot::getOdometryData(const Pose2f& robotPose, OdometryData& odometryData) const
{
  ASSERT(robot);
  (Pose2f&)odometryData = blue ? (Pose2f(pi) + robotPose) : robotPose;
}

void SimulatedRobot::getAbsoluteBallPosition(Vector2f& ballPosition)
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
    dynamic_cast<SimRobotCore2::SensorPort*>(cameraSensor)->renderCameraImages(activeCameras, activeCameraCount);

    ASSERT(!image.isReference);
    if(cameraSensor == upperCameraSensor)
      cameraInfo = upperCameraInfo;
    else
      cameraInfo = lowerCameraInfo;

    image.setResolution(cameraInfo.width, cameraInfo.height);
    const int w = image.width;
    const int h = image.height;

    const int w3 = w * 3, w2 = int(image[1] - image[0]);
    const unsigned char* src = dynamic_cast<SimRobotCore2::SensorPort*>(cameraSensor)->getValue().byteArray;
    const unsigned char* srcLineEnd = src;
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
          dest->cb = static_cast<unsigned char>((130560 - 173 * b1 - 339 * g1 + 512 * (r1 = *src++)) >> 10);
          yy += 117 * r1 + 601 * g1;
          cr -= 83 * r1;
          dest->y = dest->yCbCrPadding = static_cast<unsigned char>(yy >> 10);
          (dest++)->cr = static_cast<unsigned char>(cr >> 10);
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

void SimulatedRobot::getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData) const
{
  ASSERT(robot);

  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    // Get angles
    if(jointSensors[i])
      jointSensorData.angles[i] = static_cast<SimRobotCore2::SensorPort*>(jointSensors[i])->getValue().floatValue - jointCalibration.joints[i].offset;
    jointSensorData.currents.fill(SensorData::off);
    jointSensorData.temperatures.fill(0);

    // Set angles
    const float& targetAngle = jointRequest.angles[i];
    if(targetAngle != JointAngles::off && targetAngle != JointAngles::ignore && jointActuators[i]) // if joint does exist
      dynamic_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setValue(targetAngle + jointCalibration.joints[i].offset);
  }
  jointSensorData.timestamp = SystemCall::getCurrentSystemTime();
}

void SimulatedRobot::setJointRequest(const JointRequest& jointRequest) const
{
  ASSERT(robot);
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if(targetAngle != JointAngles::off && targetAngle != JointAngles::ignore && jointActuators[i]) // if joint does exist
      dynamic_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setValue(targetAngle + jointCalibration.joints[i].offset);
  }
}

void SimulatedRobot::toggleCamera()
{
  cameraSensor = cameraSensor == lowerCameraSensor ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = dynamic_cast<SimRobotCore2::SensorPort*>(cameraSensor);
}

void SimulatedRobot::getSensorData(InertialSensorData& inertialSensorData, UsSensorData& usSensorData, const USRequest& usRequest)
{
  ASSERT(robot);

  // Gyro
  const float* floatArray = dynamic_cast<SimRobotCore2::SensorPort*>(gyroSensor)->getValue().floatArray;
  inertialSensorData.gyro.x() = floatArray[0];
  inertialSensorData.gyro.y() = floatArray[1];
  inertialSensorData.gyro.z() = floatArray[2];

  // Acc
  floatArray = dynamic_cast<SimRobotCore2::SensorPort*>(accSensor)->getValue().floatArray;
  inertialSensorData.acc.x() = floatArray[0];
  inertialSensorData.acc.y() = floatArray[1];
  inertialSensorData.acc.z() = floatArray[2];

  // angle
  float position[3];
  float world2robot[3][3];
  dynamic_cast<SimRobotCore2::Body*>(robot)->getPose(position, world2robot);
  /*
  RotationMatrix rotMat;
  rotMat << world2robot[0][0], world2robot[1][0], world2robot[2][0],
      world2robot[0][1], world2robot[1][1], world2robot[2][1],
      world2robot[0][2], world2robot[1][2], world2robot[2][2];
  inertialSensorData.angle.x() = Rotation::Aldebaran::getXAngle(rotMat);
  inertialSensorData.angle.y() = Rotation::Aldebaran::getYAngle(rotMat);
  */
  const float axis[2] = {world2robot[1][2], -world2robot[0][2]}; // (world2robot.transpose()*[0;0;1]).cross([0;0;1])
  const float axisLength = sqrtf(axis[0]*axis[0] + axis[1]*axis[1]); // Also the sine of the angle.
  if(axisLength == 0.0f)
  {
    inertialSensorData.angle.x() = 0.0f;
    inertialSensorData.angle.y() = 0.0f;
  }
  else
  {
    const float w = std::atan2(axisLength, world2robot[2][2]) / axisLength;
    inertialSensorData.angle.x() = axis[0] * w;
    inertialSensorData.angle.y() = axis[1] * w;
  }

  // ultrasonic (model approximation. not absolutely correct in reality)
  static const float scale = 1000; //meter to millimeter
  if(usRequest.receiveMode != -1)
  {
    usSensorData.actuatorMode = static_cast<UsSensorData::UsActuatorMode>(usRequest.receiveMode);

    //FIXME simulate additional sensor values
    usSensorData.left.fill(usConfig.max);
    usSensorData.right.fill(usConfig.max);

    switch(usSensorData.actuatorMode)
    {
      case UsSensorData::leftToLeft:
      {
        const float leftUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(leftUsSensor)->getValue().floatValue * scale;
        usSensorData.left[0] = addUsJitter(leftUsValue);
        break;
      }
      case UsSensorData::leftToRight:
      {
        const float centerRightUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(centerRightUsSensor)->getValue().floatValue * scale;
        usSensorData.right[0] = addUsJitter(centerRightUsValue);
        break;
      }
      case UsSensorData::rightToLeft:
      {
        const float centerLeftUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(centerLeftUsSensor)->getValue().floatValue * scale;
        usSensorData.left[0] = addUsJitter(centerLeftUsValue);
        break;
      }
      case UsSensorData::rightToRight:
      {
        const float rightUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(rightUsSensor)->getValue().floatValue * scale;
        usSensorData.right[0] = addUsJitter(rightUsValue);
        break;
      }
      case UsSensorData::bothToSame:
      {
        const float leftUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(leftUsSensor)->getValue().floatValue * scale;
        const float rightUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(rightUsSensor)->getValue().floatValue * scale;
        usSensorData.left[0] = addUsJitter(leftUsValue);
        usSensorData.right[0] = addUsJitter(rightUsValue);
        break;
      }
      case UsSensorData::bothToOther:
      {
        const float centerLeftUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(centerLeftUsSensor)->getValue().floatValue * scale;
        const float centerRightUsValue = dynamic_cast<SimRobotCore2::SensorPort*>(centerRightUsSensor)->getValue().floatValue * scale;
        usSensorData.left[0] = addUsJitter(centerLeftUsValue);
        usSensorData.right[0] = addUsJitter(centerRightUsValue);
        break;
      }
      default:
        ASSERT(false);
    }
    usSensorData.timeStamp = SystemCall::getCurrentSystemTime();
  }
}

void SimulatedRobot::moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation)
{
  ASSERT(robot);

  Vector3f position = pos * 0.001f;
  if(changeRotation)
  {
    Matrix3f rotation = RotationMatrix::fromEulerAngles(rot);
    float rotation2[3][3];
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        rotation2[i][j] = rotation(j, i);
    dynamic_cast<SimRobotCore2::Body*>(robot)->move(&position.x(), rotation2);
  }
  else
    dynamic_cast<SimRobotCore2::Body*>(robot)->move(&position.x());
}

void SimulatedRobot::enablePhysics(bool enable)
{
  dynamic_cast<SimRobotCore2::Body*>(robot)->enablePhysics(enable);
}

void SimulatedRobot::moveBall(const Vector3f& pos, bool resetDynamics)
{
  Vector3f position = pos * 0.001f;
  dynamic_cast<SimRobotCore2::Body*>(ball)->move(&position.x());
  if(resetDynamics)
    dynamic_cast<SimRobotCore2::Body*>(ball)->resetDynamics();
}

Vector2f SimulatedRobot::getPosition(SimRobot::Object* obj)
{
  const float* position = dynamic_cast<SimRobotCore2::Body*>(obj)->getPosition();
  return Vector2f(position[0], position[1]) * 1000.f;
}

bool SimulatedRobot::getPose2f(SimRobot::Object* obj, Pose2f& Pose2f) const
{
  float position[3];
  float rot3d[3][3];
  dynamic_cast<SimRobotCore2::Body*>(obj)->getPose(position, rot3d);

  Pose2f.translation = Vector2f(position[0], position[1]) * 1000.f;

  // compute z-rotation

  /*
  Vector3f d = Vector3f(-rot3d[0][2], -rot3d[1][2], rot3d[2][2]);
  Vector3f g = Vector3f(0, 0, 1.f) ^ d;
  float w = std::atan2(std::sqrt(d.x * d.x + d.y * d.y), d.z);
  RotationMatrix withoutZ(g, w);
  RotationMatrix zOnly = RotationMatrix(Vector3f(rot3d[0][0], rot3d[0][1], rot3d[0][2]), Vector3f(rot3d[1][0], rot3d[1][1], rot3d[1][2]), Vector3f(rot3d[2][0], rot3d[2][1], rot3d[2][2])) * withoutZ.invert();
  Pose2f.rotation = atan2(zOnly.c0.y, zOnly.c0.x);
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
  Pose2f.rotation = std::atan2(c0y, c0x);
  return rot3d[2][2] >= 0.3;
}

void SimulatedRobot::getPose3f(SimRobot::Object* obj, Pose3f& pose3f) const
{
  float rotation[3][3];
  dynamic_cast<SimRobotCore2::Body*>(obj)->getPose(&pose3f.translation.x(), rotation);

  pose3f.translation *= 1000.f;
  Matrix3f rot;
  rot << rotation[0][0], rotation[1][0], rotation[2][0],
      rotation[0][1], rotation[1][1], rotation[2][1],
      rotation[0][2], rotation[1][2], rotation[2][2];
  pose3f.rotation = rot;
}

float SimulatedRobot::addUsJitter(float value)
{
  if(value >= usConfig.max) // Nothing was measured
    return value;
  else
  {
    if(randomFloat() <= 0.1f)
      return usConfig.max; // In 10% of all cases nothing is measured.
    else
      return std::max(usConfig.min, value) + randomFloat(-10.f, 10.f);
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
