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
#include "Platform/Time.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/ImageProcessing/AVX.h"

SimRobotCore2::SensorPort* SimulatedRobot::activeCameras[12];
unsigned SimulatedRobot::activeCameraCount = 0;

SimRobot::Object* SimulatedRobot::ball = nullptr;

SimulatedRobot::SimulatedRobot() :
  activeCameraIndex(activeCameraCount++)
{
  ASSERT(activeCameraCount <= sizeof(activeCameras) / sizeof(activeCameras[0]));
}

SimulatedRobot::~SimulatedRobot()
{
  --activeCameraCount;
}

void SimulatedRobot::init(SimRobot::Object* robot)
{
  ASSERT(this->robot == nullptr);
  this->robot = reinterpret_cast<SimRobotCore2::Object*>(robot);
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
  switch(cameraResolution.resolutionUpper)
  {
    case CameraResolution::Resolutions::w320h240:
      upperCameraInfo.width = 320;
      upperCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::w640h480:
      upperCameraInfo.width = 640;
      upperCameraInfo.height = 480;
      break;
    case CameraResolution::Resolutions::w1280h960:
      upperCameraInfo.width = 1280;
      upperCameraInfo.height = 960;
      break;
    default:
      ASSERT(false);
      break;
  }
  switch(cameraResolution.resolutionLower)
  {
    case CameraResolution::Resolutions::w320h240:
      lowerCameraInfo.width = 320;
      lowerCameraInfo.height = 240;
      break;
    case CameraResolution::Resolutions::w640h480:
      lowerCameraInfo.width = 640;
      lowerCameraInfo.height = 480;
      break;
    case CameraResolution::Resolutions::w1280h960:
      lowerCameraInfo.width = 1280;
      lowerCameraInfo.height = 960;
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
  VERIFY(rightFoot = reinterpret_cast<SimRobotCore2::Body*>(application->resolveObject(parts, robot, SimRobotCore2::body)));
  parts[0] = "LFoot";
  VERIFY(leftFoot = reinterpret_cast<SimRobotCore2::Body*>(application->resolveObject(parts, robot, SimRobotCore2::body)));

  // get joints
  parts.resize(1);
  QString position(".position");
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    parts[0] = QString(Joints::getName(static_cast<Joints::Joint>(i))) + position;
    parts[0] = QString(parts[0].left(1)).toUpper() + parts[0].mid(1);
    jointSensors[i] = reinterpret_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
    jointActuators[i] = reinterpret_cast<SimRobotCore2::ActuatorPort*>(application->resolveObject(parts, robot, SimRobotCore2::actuatorPort));
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
  activeCameras[activeCameraIndex] = reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor);

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

  // read RobotDimensions
  InMapFile rdStream("robotDimensions.cfg");
  ASSERT(rdStream.exists());
  rdStream >> robotDimensions;
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
    Vector3f ballPosition = getPosition3D(ball);
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
    ballPosition = getPosition(ball);
}

template<bool avx> inline __m_auto_i toYUYV(const __m_auto_i rgb0, const __m_auto_i rgb1)
{
  static const __m_auto_i bMask = _mmauto_setr128_epi8(2, char(0xFF), char(0xFF), char(0xFF), 5, char(0xFF), char(0xFF), char(0xFF), 8, char(0xFF), char(0xFF), char(0xFF), 11, char(0xFF), char(0xFF), char(0xFF));
  static const __m_auto_i grMask = _mmauto_setr128_epi8(1, char(0xFF), 0, char(0xFF), 4, char(0xFF), 3, char(0xFF), 7, char(0xFF), 6, char(0xFF), 10, char(0xFF), 9, char(0xFF));
  static constexpr int scaleExponent = 14;
  static const __m_auto_i scaledYCoeffB = _mmauto_set1_epi32(static_cast<short>(ColorModelConversions::yCoeffB * static_cast<float>(1 << scaleExponent)));
  static const __m_auto_i scaledYCoeffGR = _mmauto_set1_epi32(static_cast<short>(ColorModelConversions::yCoeffG * static_cast<float>(1 << scaleExponent)) | (static_cast<short>(ColorModelConversions::yCoeffR * static_cast<float>(1 << scaleExponent)) << 16));
  static const __m_auto_i scaledUCoeff = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::uCoeff * static_cast<float>(1 << scaleExponent)));
  static const __m_auto_i scaledVCoeff = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::vCoeff * static_cast<float>(1 << scaleExponent)));
  static const __m_auto_i c_128 = _mmauto_set1_epi16(128);
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();

  const __m_auto_i gr0 = _mmauto_shuffle_epi8(rgb0, grMask);
  const __m_auto_i gr1 = _mmauto_shuffle_epi8(rgb1, grMask);
  const __m_auto_i b0 = _mmauto_shuffle_epi8(rgb0, bMask);
  const __m_auto_i b1 = _mmauto_shuffle_epi8(rgb1, bMask);

  const __m_auto_i y = _mmauto_packs_epi32(
                         _mmauto_srai_epi32(_mmauto_add_epi32(_mmauto_madd_epi16(gr0, scaledYCoeffGR), _mmauto_madd_epi16(b0, scaledYCoeffB)), scaleExponent),
                         _mmauto_srai_epi32(_mmauto_add_epi32(_mmauto_madd_epi16(gr1, scaledYCoeffGR), _mmauto_madd_epi16(b1, scaledYCoeffB)), scaleExponent)
                       );

  const __m_auto_i uv = _mmauto_add_epi16(
                          _mmauto_unpacklo_epi16(
                            _mmauto_packs_epi32(_mmauto_srai_epi32(_mmauto_madd_epi16(_mmauto_sub_epi16(_mmauto_packs_epi32(b0, b1), y), scaledUCoeff), scaleExponent + 1), c_0),
                            _mmauto_packs_epi32(_mmauto_srai_epi32(_mmauto_madd_epi16(_mmauto_sub_epi16(_mmauto_packs_epi32(_mmauto_srli_epi32(gr0, 16), _mmauto_srli_epi32(gr1, 16)), y), scaledVCoeff), scaleExponent + 1), c_0)
                          ),
                          c_128
                        );

  return _mmauto_unpacklo_epi8(
           _mmauto_packus_epi16(y, c_0),
           _mmauto_packus_epi16(uv, c_0)
         );
}

template<bool srcAligned, bool destAligned, bool avx> void convertImage(const unsigned char* const src, Image& dest)
{
  ASSERT((dest.width * dest.height * 12) % 96 == 0); // source data alignment
  ASSERT((dest.width * 4) % 64 == 0); // dest data alignment
  const __m_auto_i* const pSrcEnd = reinterpret_cast<const __m_auto_i*>(src + dest.width * dest.height * 12);

  __m_auto_i* pDestEnd = reinterpret_cast<__m_auto_i*>(dest[dest.height]);
  const ptrdiff_t lineLength = reinterpret_cast<__m_auto_i*>(&dest[0][dest.width]) - reinterpret_cast<__m_auto_i*>(dest[0]);
  __m_auto_i* pDest = pDestEnd - lineLength;

  for(const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(src); pSrc < pSrcEnd;)
  {
    const __m_auto_i p0 = _mmauto_loadt_si_all<srcAligned>(pSrc++);
    const __m_auto_i p1 = _mmauto_loadt_si_all<srcAligned>(pSrc++);
    const __m_auto_i p2 = _mmauto_loadt_si_all<srcAligned>(pSrc++);

    if(avx)
    {
      const __m_auto_i tmp0 = _mmauto_permute2x128_si256(p0, p1, 3 << 4);
      const __m_auto_i tmp1 = _mmauto_permute2x128_si256(p0, p2, 1 | (2 << 4));
      const __m_auto_i tmp2 = _mmauto_permute2x128_si256(p1, p2, 3 << 4);
      const __m_auto_i yuyv0 = toYUYV<avx>(tmp0, _mmauto_alignr_epi8(tmp1, tmp0, 12));
      const __m_auto_i yuyv1 = toYUYV<avx>(_mmauto_alignr_epi8(tmp2, tmp1, 8), _mmauto_srli_si_all(tmp2, 4));
      _mmauto_storet_si_all<destAligned>(pDest++, _mmauto_permute2x128_si256(yuyv0, yuyv1, 2 << 4));
      _mmauto_storet_si_all<destAligned>(pDest++, _mmauto_permute2x128_si256(yuyv0, yuyv1, 1 | (3 << 4)));
    }
    else
    {
      _mmauto_storet_si_all<destAligned>(pDest++, toYUYV<avx>(p0, _mmauto_alignr_epi8(p1, p0, 12)));
      _mmauto_storet_si_all<destAligned>(pDest++, toYUYV<avx>(_mmauto_alignr_epi8(p2, p1, 8), _mmauto_srli_si_all(p2, 4)));
    }

    if(pDest == pDestEnd)
    {
      pDestEnd -= lineLength;
      pDest -= lineLength * 2;
    }
  }
}

void SimulatedRobot::getImage(Image& image, CameraInfo& cameraInfo)
{
  ASSERT(robot);

  if(cameraSensor)
  {
    reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor)->renderCameraImages(activeCameras, activeCameraCount);

    ASSERT(!image.isReference);
    if(cameraSensor == upperCameraSensor)
    {
      cameraInfo = upperCameraInfo;
    }
    else
    {
      cameraInfo = lowerCameraInfo;
    }

    image.setResolution(cameraInfo.width / 2, cameraInfo.height / 2, true);

    const unsigned char* const src = reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor)->getValue().byteArray;
    if(_supportsAVX2)
    {
      if(simdAligned<true>(src))
      {
        if(simdAligned<true>(image[0]))
          convertImage<true, true, true>(src, image);
        else
          convertImage<true, false, true>(src, image);
      }
      else
      {
        if(simdAligned<true>(image[0]))
          convertImage<false, true, true>(src, image);
        else
          convertImage<false, false, true>(src, image);
      }
    }
    else
    {
      if(simdAligned<false>(src))
      {
        if(simdAligned<false>(image[0]))
          convertImage<true, true, false>(src, image);
        else
          convertImage<true, false, false>(src, image);
      }
      else
      {
        if(simdAligned<false>(image[0]))
          convertImage<false, true, false>(src, image);
        else
          convertImage<false, false, false>(src, image);
      }
    }
  }

  image.timeStamp = Time::getCurrentSystemTime();
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
    const float targetAngle = jointRequest.angles[i];
    if(targetAngle != JointAngles::off && targetAngle != JointAngles::ignore && jointActuators[i]) // if joint does exist
      reinterpret_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setValue(targetAngle + jointCalibration.joints[i].offset);
  }
  jointSensorData.timestamp = Time::getCurrentSystemTime();
}

void SimulatedRobot::setJointRequest(const JointRequest& jointRequest) const
{
  ASSERT(robot);
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    // Set angles
    const float targetAngle = jointRequest.angles[i];
    if(targetAngle != JointAngles::off && targetAngle != JointAngles::ignore && jointActuators[i]) // if joint does exist
      reinterpret_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setValue(targetAngle + jointCalibration.joints[i].offset);
  }
}

void SimulatedRobot::toggleCamera()
{
  cameraSensor = cameraSensor == lowerCameraSensor ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor);
}

void SimulatedRobot::getSensorData(FsrSensorData& fsrSensorData, InertialSensorData& inertialSensorData)
{
  ASSERT(robot);

  // FSR
  if(leftFoot && rightFoot)
  {
    const SimRobot::Object* feet[2] = {leftFoot, rightFoot};
    std::array<float, FsrSensors::numOfFsrSensors>* fsrs[2] = {&fsrSensorData.left, &fsrSensorData.right};
    std::array<Vector2f, FsrSensors::numOfFsrSensors>* fsrPositions[2] = {&robotDimensions.leftFsrPositions, &robotDimensions.rightFsrPositions};
    float* totals[2] = {&fsrSensorData.leftTotal, &fsrSensorData.rightTotal};
    static constexpr float weight = 0.415f;
    for(int i = 0; i < 2; ++i)
    {
      Pose3f pose;
      getPose3f(feet[i], pose);

      *totals[i] = 0.f;
      for(int j = 0; j < 4; ++j)
      {
        const Vector2f& frsPos = (*fsrPositions[i])[j];
        Vector3f pos = (pose + Vector3f(frsPos.x(), frsPos.y(), -robotDimensions.footHeight)).translation;
        (*fsrs[i])[j] = std::max(0.f, -pos.z() * weight);
        *totals[i] += (*fsrs[i])[j];
      }
    }
  }

  // Gyro
  const float* floatArray = reinterpret_cast<SimRobotCore2::SensorPort*>(gyroSensor)->getValue().floatArray;
  inertialSensorData.gyro.x() = floatArray[0];
  inertialSensorData.gyro.y() = floatArray[1];
  inertialSensorData.gyro.z() = floatArray[2];

  // Acc
  floatArray = reinterpret_cast<SimRobotCore2::SensorPort*>(accSensor)->getValue().floatArray;
  inertialSensorData.acc.x() = floatArray[0];
  inertialSensorData.acc.y() = floatArray[1];
  inertialSensorData.acc.z() = floatArray[2];

  // angle
  float position[3];
  float world2robot[3][3];
  reinterpret_cast<SimRobotCore2::Body*>(robot)->getPose(position, world2robot);
  /*
  RotationMatrix rotMat;
  rotMat << world2robot[0][0], world2robot[1][0], world2robot[2][0],
      world2robot[0][1], world2robot[1][1], world2robot[2][1],
      world2robot[0][2], world2robot[1][2], world2robot[2][2];
  inertialSensorData.angle.x() = Rotation::Aldebaran::getXAngle(rotMat);
  inertialSensorData.angle.y() = Rotation::Aldebaran::getYAngle(rotMat);
  */
  const float axis[2] = { world2robot[1][2], -world2robot[0][2] }; // (world2robot.transpose()*[0;0;1]).cross([0;0;1])
  const float axisLength = sqrtf(axis[0] * axis[0] + axis[1] * axis[1]); // Also the sine of the angle.
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
    reinterpret_cast<SimRobotCore2::Body*>(robot)->move(&position.x(), rotation2);
  }
  else
    reinterpret_cast<SimRobotCore2::Body*>(robot)->move(&position.x());
}

void SimulatedRobot::enablePhysics(bool enable)
{
  reinterpret_cast<SimRobotCore2::Body*>(robot)->enablePhysics(enable);
}

void SimulatedRobot::moveBall(const Vector3f& pos, bool resetDynamics)
{
  if(!ball)
    return;
  Vector3f position = pos * 0.001f;
  reinterpret_cast<SimRobotCore2::Body*>(ball)->move(&position.x());
  if(resetDynamics)
    reinterpret_cast<SimRobotCore2::Body*>(ball)->resetDynamics();
}

Vector2f SimulatedRobot::getPosition(const SimRobot::Object* obj)
{
  const float* position = reinterpret_cast<const SimRobotCore2::Body*>(obj)->getPosition();
  return Vector2f(position[0], position[1]) * 1000.f;
}

Vector3f SimulatedRobot::getPosition3D(const SimRobot::Object* obj)
{
  const float* position = reinterpret_cast<const SimRobotCore2::Body*>(obj)->getPosition();
  return Vector3f(position[0], position[1], position[2]) * 1000.f;
}

bool SimulatedRobot::getPose2f(const SimRobot::Object* obj, Pose2f& Pose2f) const
{
  float position[3];
  float rot3d[3][3];
  reinterpret_cast<const SimRobotCore2::Body*>(obj)->getPose(position, rot3d);

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

void SimulatedRobot::getPose3f(const SimRobot::Object* obj, Pose3f& pose3f) const
{
  float rotation[3][3];
  reinterpret_cast<const SimRobotCore2::Body*>(obj)->getPose(&pose3f.translation.x(), rotation);

  pose3f.translation *= 1000.f;
  Matrix3f rot;
  rot << rotation[0][0], rotation[1][0], rotation[2][0],
      rotation[0][1], rotation[1][1], rotation[2][1],
      rotation[0][2], rotation[1][2], rotation[2][2];
  pose3f.rotation = rot;
}

bool SimulatedRobot::isBlue(const SimRobot::Object* obj)
{
  return getNumber(obj) <= 6;
}

int SimulatedRobot::getNumber(const SimRobot::Object* obj)
{
  QString robotNumberString = obj->getFullName();
  int pos = robotNumberString.lastIndexOf('.');
  robotNumberString.remove(0, pos + 6);
  return robotNumberString.toInt();
}
