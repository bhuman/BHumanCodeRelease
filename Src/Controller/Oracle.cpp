/**
 * @file Controller/Oracle.cpp
 * Implementation of class Oracle for SimRobotQt.
 * @author Colin Graf
 */

#include <QString>
#include <QVector>

#include <algorithm>

#include "Oracle.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/TeamComm3DCtrl.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Tools/Streams/InStreams.h"

SimRobotCore2::SensorPort* Oracle::activeCameras[12]; // should be 10, but TeamComm3DCtrl allocates two unsued ones
unsigned Oracle::activeCameraCount = 0;

SimRobot::Object* Oracle::ball = 0;
Pose2D Oracle::lastBallContactPose;
FieldDimensions Oracle::fieldDimensions;
CameraInfo Oracle::upperCameraInfo;
CameraInfo Oracle::lowerCameraInfo;

Oracle::Oracle() :
  blue(false), robot(0), leftFoot(0), rightFoot(0), lastTimeStamp(0), activeCameraIndex(activeCameraCount++)
{
  ASSERT(activeCameraCount <= sizeof(activeCameras) / sizeof(activeCameras[0]));
}

Oracle::~Oracle()
{
  --activeCameraCount;
}

void Oracle::init(SimRobot::Object* robot)
{
  ASSERT(this->robot == 0);
  this->robot = (SimRobotCore2::Object*)robot;
  application = RoboCupCtrl::application ? RoboCupCtrl::application : TeamComm3DCtrl::application;

  // get the robot's team color
  blue = isBlue(robot);

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
}

Oracle::BallOut Oracle::updateBall()
{
  if(!ball)
    return NONE;

  BallOut result = NONE;
  Vector2<> ballPos = getPosition(ball);
  if(!fieldDimensions.isInsideField(ballPos))
  {
    if(fabs(ballPos.y) < fieldDimensions.yPosLeftGoal) // goal
    {
      result = ballPos.x > fieldDimensions.xPosOpponentGroundline ? GOAL_BY_RED : GOAL_BY_BLUE;
    }
    else
    {
      float x;
      if((Pose2D(ballPos) - lastBallContactPose).translation.x > 0) // 1m behind robot
        x = (lastBallContactPose + Pose2D(-1000, 0)).translation.x;
      else // 1m behind where ball went out
        x = (Pose2D(lastBallContactPose.rotation, ballPos) + Pose2D(-1000, 0)).translation.x;

      if(fabs(ballPos.x) > fieldDimensions.xPosOpponentGroundline && (Pose2D(x, 0) - Pose2D(lastBallContactPose.rotation)).translation.x > 0)
        x = 0; // center line
      else if(x < fieldDimensions.xPosOwnDropInLine)
        x = fieldDimensions.xPosOwnDropInLine; // clip
      else if(x > fieldDimensions.xPosOpponentDropInLine)
        x = fieldDimensions.xPosOpponentDropInLine; // clip
      ballPos.x = x;

      if(ballPos.y < 0)
        ballPos.y = fieldDimensions.yPosRightDropInLine; // right throw-in line
      else
        ballPos.y = fieldDimensions.yPosLeftDropInLine; // left throw-in line

      moveBall(Vector3<>(ballPos.x, ballPos.y, 100.f), true);
      result = lastBallContactPose.rotation == 0 ? OUT_BY_RED : OUT_BY_BLUE;
    }
  }

  return result;
}

void Oracle::init()
{
  fieldDimensions.load();
  InMapFile upperStream("upperCameraInfo.cfg");
  ASSERT(upperStream.exists());
  upperStream >> upperCameraInfo;
  InMapFile lowerStream("lowerCameraInfo.cfg");
  ASSERT(lowerStream.exists());
  lowerStream >> lowerCameraInfo;
}

void Oracle::setBall(SimRobot::Object* ball)
{
  Oracle::ball = ball;
}

void Oracle::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2D(isBlue(robot) ? pi : 0, getPosition(robot));
}

void Oracle::getRobotPose(RobotPose& robotPose) const
{
  ASSERT(rightFoot && leftFoot);

  getPose2D(robot, (Pose2D&)robotPose);
  robotPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;

  if(blue)
    robotPose = Pose2D(pi) + robotPose;

  robotPose.validity = 1.f;
  robotPose.deviation = 1.f;
}

void Oracle::getOdometryData(const RobotPose& robotPose, OdometryData& odometryData) const
{
  ASSERT(robot);
  (Pose2D&)odometryData = blue ? (Pose2D(pi) + robotPose) : (const Pose2D&)robotPose;
}

void Oracle::getBallModel(const RobotPose& robotPose, BallModel& ballModel)
{
  ASSERT(robot);
  if(ball)
  {
    unsigned int timeWhenLastSeen = SystemCall::getCurrentSystemTime();
    Vector2<> ballPosition = getPosition(ball);
    if(blue)
      ballPosition = -ballPosition;
    Vector2<float> velocity((ballPosition - lastBallPosition) / float(timeWhenLastSeen - ballModel.timeWhenLastSeen) * 1000.0f);
    ballModel.estimate.position = robotPose.invert() * ballPosition;
    ballModel.estimate.velocity = Vector2<>(velocity).rotate(-robotPose.rotation);
    ballModel.lastPerception = ballModel.estimate.position;
    ballModel.timeWhenLastSeen = timeWhenLastSeen;
    ballModel.timeWhenDisappeared = timeWhenLastSeen;
    lastBallPosition = ballPosition;
  }
}

void Oracle::getImage(Image& image, CameraInfo& cameraInfo)
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

    image.setResolution(cameraInfo.width, cameraInfo.height);
    const int w = image.width;
    const int h = image.height;

    const int w3 = w * 3, w2 = image[1] - image[0];
    unsigned char* src = (unsigned char*)((SimRobotCore2::SensorPort*)cameraSensor)->getValue().byteArray;
    unsigned char* srcLineEnd = src;
    Image::Pixel* destBegin = image[0];
    Image::Pixel* dest;
    int r1, g1, b1, yy, cr;
    for(int y = h - 1; y >= 0; --y)
    {
      for(srcLineEnd += w3, dest = destBegin + y * w2; src < srcLineEnd;)
        for(int i = 0; i < 4; ++i)
        {
          yy = 306 * (r1 = *(src++));
          cr = 130560 - 429 * (g1 = *(src++)) + 512 * r1;
          dest->cb = (unsigned char)((130560 - 173 * r1 - 339 * g1 + 512 * (b1 = *(src++))) >> 10);
          yy += 117 * b1 + 601 * g1;
          cr -= 83 * b1;
          dest->y = dest->yCbCrPadding = (unsigned char)(yy >> 10);
          (dest++)->cr = (unsigned char)(cr >> 10);
        }
    }
  }

  image.timeStamp = SystemCall::getCurrentSystemTime();
}

void Oracle::getAndSetJointData(const JointData& jointRequest, JointData& jointData) const
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

void Oracle::setJointData(const JointData& jointRequest) const
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

void Oracle::toggleCamera()
{
  cameraSensor = cameraSensor == lowerCameraSensor ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::SensorPort*)cameraSensor;
}

void Oracle::getSensorData(SensorData& sensorData, const USRequest& usRequest)
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

void Oracle::getOrientationData(const SensorData& sensorData, OrientationData& orientationData)
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

void Oracle::moveRobot(const Vector3<>& pos, const Vector3<>& rot, bool changeRotation)
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

void Oracle::enablePhysics(bool enable)
{
  ((SimRobotCore2::Body*)robot)->enablePhysics(enable);
}

void Oracle::moveBall(const Vector3<>& pos, bool resetDynamics)
{
  Vector3<> position = pos * 0.001f;
  ((SimRobotCore2::Body*)ball)->move(&position.x);
  if(resetDynamics)
    ((SimRobotCore2::Body*)ball)->resetDynamics();
}

Vector2<> Oracle::getPosition(SimRobot::Object* obj)
{
  const float* position = ((SimRobotCore2::Body*)obj)->getPosition();
  return Vector2<>(position[0] * 1000.f, position[1] * 1000.f);
}

void Oracle::getPose2D(SimRobot::Object* obj, Pose2D& pose2D) const
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

void Oracle::getPose3D(SimRobot::Object* obj, Pose3D& pose3D) const
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

float Oracle::addJitter(float value)
{
  if(value >= 1400.f) // nothing was measured
    return value;
  else
  {
    value = std::max(260.f, value);
    float randValue = (float) (0.1f * (rand() % 100) + -0.1f * (rand() % 100));

    if (rand() % 100 <= 10)
    {
      return 2200;
    }
    else
    {
      return value + randValue;
    }
  }
}

bool Oracle::isBlue(SimRobot::Object* obj)
{
  QString robotNumberString(obj->getFullName());
  int pos = robotNumberString.lastIndexOf('.');
  robotNumberString.remove(0, pos + 6);
  return robotNumberString.toInt() < 6;
}
