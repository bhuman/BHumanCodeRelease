/**
 * @file SimulatedRobot3D.cpp
 *
 * This file implements the interface to a robot with the 3D simulation core.
 *
 * @author Arne Hasselbring et al
 */

#include "SimulatedRobot3D.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "ImageProcessing/AVX.h"
#include "Math/Pose2f.h"
#include "Math/Pose3f.h"
#include "Math/Approx.h"
#include "Streaming/InStreams.h"
#include <SimRobotCore2.h>

SimRobotCore2::SensorPort* SimulatedRobot3D::activeCameras[SimulatedRobot::robotsPerTeam * 2] = {nullptr};
unsigned SimulatedRobot3D::activeCameraCount = 0;

SimulatedRobot3D::SimulatedRobot3D(SimRobot::Object* robot) :
  SimulatedRobot(robot),
  activeCameraIndex(activeCameraCount++)
{
  ASSERT(activeCameraCount <= sizeof(activeCameras) / sizeof(activeCameras[0]));

  // load camera parameters
  InMapFile intrStream("cameraIntrinsics.cfg");
  CameraIntrinsics cameraIntrinsics;
  ASSERT(intrStream.exists());
  intrStream >> cameraIntrinsics;

  CameraResolutionRequest cameraResolutionRequest;
  InMapFile resStream("cameraResolution.cfg");
  ASSERT(resStream.exists());
  resStream >> cameraResolutionRequest;

  // build cameraInfo
  FOREACH_ENUM(CameraInfo::Camera, camera)
  {
    cameraInfos[camera].camera = camera;
    cameraResolutionRequest.apply(camera, cameraInfos[camera]);

    // set opening angle
    cameraInfos[camera].openingAngleWidth = cameraIntrinsics.cameras[camera].openingAngleWidth;
    cameraInfos[camera].openingAngleHeight = cameraIntrinsics.cameras[camera].openingAngleHeight;
    // set optical center
    cameraInfos[camera].opticalCenter.x() = cameraIntrinsics.cameras[camera].opticalCenter.x() * cameraInfos[camera].width;
    cameraInfos[camera].opticalCenter.y() = cameraIntrinsics.cameras[camera].opticalCenter.y() * cameraInfos[camera].height;
    // update focal length
    cameraInfos[camera].updateFocalLength();
  }

  SimRobot::Application* const application = RoboCupCtrl::application;

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
  QString velocity(".velocity");
  QString jointName;
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    jointName = QString(TypeRegistry::getEnumName(static_cast<Joints::Joint>(i)));
    jointName = QString(jointName.left(1)).toUpper() + jointName.mid(1);

    parts[0] = jointName + position;
    jointSensors[i] = reinterpret_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
    jointActuators[i] = reinterpret_cast<SimRobotCore2::ActuatorPort*>(application->resolveObject(parts, robot, SimRobotCore2::actuatorPort));

    parts[0] = jointName + velocity;
    jointVelocitySensors[i] = reinterpret_cast<SimRobotCore2::SensorPort*>(application->resolveObject(parts, robot, SimRobotCore2::sensorPort));
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
  cameraSensor = upperCameraSensor;
  activeCameras[activeCameraIndex] = reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor);

  // load calibration
  InMapFile stream("jointCalibration.cfg");
  ASSERT(stream.exists());
  stream >> jointCalibration;

  // read RobotDimensions
  InMapFile rdStream("robotDimensions.cfg");
  ASSERT(rdStream.exists());
  rdStream >> robotDimensions;
}

SimulatedRobot3D::~SimulatedRobot3D()
{
  --activeCameraCount;
}

void SimulatedRobot3D::getRobotPose(Pose2f& robotPose) const
{
  ASSERT(rightFoot && leftFoot);

  getPose2f(robot, robotPose);
  robotPose.translation = (getPosition(leftFoot) + getPosition(rightFoot)) * 0.5f;

  if(firstTeam)
    robotPose = Pose2f(pi) + robotPose;
}

void SimulatedRobot3D::getTorsoMatrix(TorsoMatrix& torsoMatrix)
{
  Pose3f robotInWorld;
  getPose3f(robot, robotInWorld);

  robotInWorld.translate(Vector3f(0.f, 0.f, -85.f));
  Pose2f robotOnGroundInWorld;
  getPose2f(robot, robotOnGroundInWorld);
  const Pose2f worldInRobotOnGround = robotOnGroundInWorld.inverse();
  Pose3f worldInRobotOnGround3D;
  worldInRobotOnGround3D.translation = Vector3f(worldInRobotOnGround.translation.x(), worldInRobotOnGround.translation.y(), 0.f);
  worldInRobotOnGround3D.rotation = RotationMatrix::aroundZ(worldInRobotOnGround.rotation);

  static_cast<Pose3f&>(torsoMatrix) = worldInRobotOnGround3D * robotInWorld;
  torsoMatrix.isValid = true;
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

template<bool srcAligned, bool destAligned, bool avx> void convertImage(const unsigned char* const src, CameraImage& dest)
{
  ASSERT((dest.width * dest.height * 6) % 96 == 0); // source data alignment
  ASSERT((dest.width * 4) % 64 == 0); // dest data alignment
  const __m_auto_i* const pSrcEnd = reinterpret_cast<const __m_auto_i*>(src + dest.width * dest.height * 6);

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

#if defined MACOS && !defined NDEBUG
#undef _supportsAVX2
#define _supportsAVX2 false
#endif

void SimulatedRobot3D::getImage(CameraImage& cameraImage, CameraInfo& cameraInfo)
{
  ASSERT(robot);

  if(cameraSensor)
  {
    reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor)->renderCameraImages(activeCameras, activeCameraCount);

    ASSERT(!cameraImage.isReference());

    cameraInfo = cameraInfos[cameraSensor == upperCameraSensor ? CameraInfo::upper : CameraInfo::lower];
    cameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);

    const unsigned char* const src = reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor)->getValue().byteArray;
    if(simdAligned<_supportsAVX2>(src))
    {
      if(simdAligned<_supportsAVX2>(cameraImage[0]))
        convertImage<true, true, _supportsAVX2>(src, cameraImage);
      else
        convertImage<true, false, _supportsAVX2>(src, cameraImage);
    }
    else
    {
      if(simdAligned<_supportsAVX2>(cameraImage[0]))
        convertImage<false, true, _supportsAVX2>(src, cameraImage);
      else
        convertImage<false, false, _supportsAVX2>(src, cameraImage);
    }
  }

  cameraImage.timestamp = Time::getCurrentSystemTime();
}

void SimulatedRobot3D::getCameraInfo(CameraInfo& cameraInfo)
{
  if(cameraSensor)
    cameraInfo = cameraInfos[cameraSensor == upperCameraSensor ? CameraInfo::upper : CameraInfo::lower];
}

void SimulatedRobot3D::toggleCamera()
{
  cameraSensor = cameraSensor == lowerCameraSensor || !lowerCameraSensor ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = reinterpret_cast<SimRobotCore2::SensorPort*>(cameraSensor);
}

void SimulatedRobot3D::setJointCalibration(const JointCalibration& jointCalibration)
{
  this->jointCalibration.offsets = jointCalibration.offsets;
}

void SimulatedRobot3D::getAndSetJointData(const JointRequest& jointRequest, JointSensorData& jointSensorData) const
{
  ASSERT(robot);

  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    // Get angles
    if(jointSensors[i])
    {
      jointSensorData.angles[i] = applyDiscretization(
                                    static_cast<SimRobotCore2::SensorPort*>(jointSensors[i])->getValue().floatValue, jointDiscretizationStep);
      jointSensorData.velocity[i] = static_cast<SimRobotCore2::SensorPort*>(jointVelocitySensors[i])->getValue().floatValue;
    }

    // Set angles
    if(jointActuators[i])
    {
      const float targetAngle = jointRequest.angles[i];
      if(targetAngle != JointAngles::off && targetAngle != JointAngles::ignore)    // if joint does exist
        reinterpret_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setValue(targetAngle + jointCalibration.offsets[i]);
      dynamic_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setStiffness(jointRequest.stiffnessData.stiffnesses[i]);
    }
  }
  jointSensorData.currents.fill(static_cast<short>(SensorData::off));
  jointSensorData.temperatures.fill(0);
  jointSensorData.status.fill(JointSensorData::regular);
  jointSensorData.timestamp = Time::getCurrentSystemTime();
}

void SimulatedRobot3D::setJointRequest(const JointRequest& jointRequest, const bool isPuppet) const
{
  ASSERT(robot);

  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    // Set angles
    if(jointActuators[i])
    {
      const float targetAngle = jointRequest.angles[i];
      if(targetAngle != JointAngles::off && targetAngle != JointAngles::ignore)   // if joint does exist
        reinterpret_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setValue(targetAngle + jointCalibration.offsets[i]);
      dynamic_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setStiffness(isPuppet ? 100 : jointRequest.stiffnessData.stiffnesses[i]);
      dynamic_cast<SimRobotCore2::ActuatorPort*>(jointActuators[i])->setPuppetState(isPuppet);
    }
  }
}

void SimulatedRobot3D::getSensorData(FsrSensorData& fsrSensorData, RawInertialSensorData& rawInertialSensorData)
{
  ASSERT(robot);

  // FSR
  if(leftFoot && rightFoot)
  {
    const SimRobot::Object* feet[Legs::numOfLegs] = {leftFoot, rightFoot};
    std::array<Vector2f, FsrSensors::numOfFsrSensors>* fsrPositions[Legs::numOfLegs] = {&robotDimensions.leftFsrPositions, &robotDimensions.rightFsrPositions};
    static constexpr float weight = 0.415f;
    FOREACH_ENUM(Legs::Leg, leg)
    {
      Pose3f pose;
      getPose3f(feet[leg], pose);

      fsrSensorData.totals[leg] = 0.f;
      FOREACH_ENUM(FsrSensors::FsrSensor, sensor)
      {
        const Vector2f& frsPos = (*fsrPositions[leg])[sensor];
        Vector3f pos = (pose + Vector3f(frsPos.x(), frsPos.y(), -robotDimensions.footHeight)).translation;
        fsrSensorData.pressures[leg][sensor] = std::max(0.f, -pos.z() * weight);
        fsrSensorData.totals[leg] += fsrSensorData.pressures[leg][sensor];
      }
    }
  }

  if(accSensor && gyroSensor)
  {
    const float* accArray = reinterpret_cast<SimRobotCore2::SensorPort*>(accSensor)->getValue().floatArray;
    const float* gyroArray = reinterpret_cast<SimRobotCore2::SensorPort*>(gyroSensor)->getValue().floatArray;

    if(!newGyroMeasurement || !useTimeDelay)
    {
      //save data from the gyro
      lastInertialData.gyro.x() = gyroArray[0];
      lastInertialData.gyro.y() = gyroArray[1];
      lastInertialData.gyro.z() = gyroArray[2];
    }

    // Gyro
    if(newGyroMeasurement || !useTimeDelay)
    {
      rawInertialSensorData.gyro.x() = applyWhiteNoise((gyroArray[0] + lastInertialData.gyro.x()) / 2, gyroVariance);
      rawInertialSensorData.gyro.y() = applyWhiteNoise((gyroArray[1] + lastInertialData.gyro.y()) / 2, gyroVariance);
      rawInertialSensorData.gyro.z() = applyWhiteNoise((gyroArray[2] + lastInertialData.gyro.z()) / 2, gyroVariance);

      // save data from the acc
      lastInertialData.acc.x() = accArray[0];
      lastInertialData.acc.y() = accArray[1];
      lastInertialData.acc.z() = accArray[2];
    }

    // Acc
    if(!newGyroMeasurement || !useTimeDelay)
    {
      rawInertialSensorData.acc.x() = applyWhiteNoise((accArray[0] + lastInertialData.acc.x()) / 2, accVariance);
      rawInertialSensorData.acc.y() = applyWhiteNoise((accArray[1] + lastInertialData.acc.y()) / 2, accVariance);
      rawInertialSensorData.acc.z() = applyWhiteNoise((accArray[2] + lastInertialData.acc.z()) / 2, accVariance);
    }
    newGyroMeasurement = !newGyroMeasurement;

    // angle
    float position[3];
    float world2robot[3][3];
    reinterpret_cast<SimRobotCore2::Body*>(robot)->getPose(position, world2robot);

    const float axis[2] = {world2robot[1][2], -world2robot[0][2]}; // (world2robot.transpose()*[0;0;1]).cross([0;0;1])
    const float axisLength = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1]); // Also the sine of the angle.
    if(axisLength == 0.0f)
    {
      rawInertialSensorData.angle.x() = 0.0f;
      rawInertialSensorData.angle.y() = 0.0f;
    }
    else
    {
      const float w = std::atan2(axisLength, world2robot[2][2]) / axisLength;
      rawInertialSensorData.angle.x() = axis[0] * w;
      rawInertialSensorData.angle.y() = axis[1] * w;
    }

    const float h = std::sqrt(world2robot[0][0] * world2robot[0][0] + world2robot[1][0] * world2robot[1][0]);
    if(Approx::isZero(h))
      rawInertialSensorData.angle.z() = 0.f;
    else
      rawInertialSensorData.angle.z() = std::acos(world2robot[0][0] / h) * sgnPos(world2robot[0][1]);
  }
}

void SimulatedRobot3D::getAndSetMotionData(const MotionRequest&, MotionInfo&)
{}

void SimulatedRobot3D::moveRobot(const Vector3f& pos, const Vector3f& rot, bool changeRotation, bool resetDynamics)
{
  const Vector3f position = pos * 0.001f;

  if(changeRotation)
  {
    Matrix3f rotation = RotationMatrix::fromEulerAngles(rot);
    float rotation2[3][3];
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        rotation2[i][j] = rotation(j, i); // Rotation of 180deg per definition (team/opponent)
    static_cast<SimRobotCore2::Body*>(robot)->move(&position.x(), rotation2);
  }
  else
    static_cast<SimRobotCore2::Body*>(robot)->move(&position.x());

  if(resetDynamics)
    static_cast<SimRobotCore2::Body*>(robot)->resetDynamics();
}

void SimulatedRobot3D::enablePhysics(bool enable)
{
  static_cast<SimRobotCore2::Body*>(robot)->enablePhysics(enable);
}

void SimulatedRobot3D::enableGravity(bool enable)
{
  static_cast<SimRobotCore2::Body*>(robot)->enableGravity(enable);
}

void SimulatedRobot3D::enableSensorWhiteNoise(const bool enable)
{
  useWhiteNoise = enable;
}

void SimulatedRobot3D::enableSensorDelay(const bool enable)
{
  useTimeDelay = enable;
}

void SimulatedRobot3D::enableSensorDiscretization(const bool enable)
{
  useDiscretization = enable;
}

bool SimulatedRobot3D::getPose2f(const SimRobot::Object* obj, Pose2f& pose) const
{
  float position[3];
  float rot3d[3][3];
  static_cast<const SimRobotCore2::Body*>(obj)->getPose(position, rot3d);

  pose.translation = Vector2f(position[0], position[1]) * 1000.f;

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
  pose.rotation = std::atan2(c0y, c0x);
  return rot3d[2][2] >= 0.3f;
}

void SimulatedRobot3D::getPose3f(const SimRobot::Object* obj, Pose3f& pose) const
{
  float rotation[3][3];
  static_cast<const SimRobotCore2::Body*>(obj)->getPose(pose.translation.data(), rotation);

  pose.translation *= 1000.f;
  Matrix3f rot;
  rot << rotation[0][0], rotation[1][0], rotation[2][0],
         rotation[0][1], rotation[1][1], rotation[2][1],
         rotation[0][2], rotation[1][2], rotation[2][2];
  pose.rotation = rot;
}

float SimulatedRobot3D::applyDiscretization(float value, float discretizationStep) const
{
  if(!useDiscretization)
    return value;

  return floorf(value / discretizationStep) * discretizationStep;
}

float SimulatedRobot3D::applyWhiteNoise(float value, float variance)
{
  if(!useWhiteNoise)
    return value;

  return value + generalNormalDistribution(randomGenerator) * std::sqrt(variance);
}
