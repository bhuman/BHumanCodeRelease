/**
 * @file SimulatedNao/LocalConsole.cpp
 *
 * Implementation of LocalConsole.
 *
 * @author Thomas Röfer
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 */

#include "LocalConsole.h"
#include "SimulatedNao/ConsoleRoboCupCtrl.h"
#include "SimulatedNao/SimulatedRobot2D.h"
#include "SimulatedNao/SimulatedRobot3D.h"
#include "Platform/Time.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Framework/Debug.h"

LocalConsole::LocalConsole(const Settings& settings, const std::string& robotName, ConsoleRoboCupCtrl* ctrl, const std::string& logFile, Debug* debug) :
  RobotConsole(settings, robotName, ctrl,
               logFile.empty() ? SystemCall::simulatedRobot : logFile == "remote" ? SystemCall::remoteRobot : SystemCall::logFileReplay,
               connectReceiverWithRobot(debug), connectSenderWithRobot(debug)),
  updatedSignal(1)
{
  addPerRobotViews();

  if(mode == SystemCall::remoteRobot)
  {
    SimRobot::Object* puppet = RoboCupCtrl::application->resolveObject("RoboCup.puppets." + QString::fromStdString(robotName), SimRobotCore3::body);
    if(puppet)
    {
      simulatedRobot = std::make_unique<SimulatedRobot3D>(puppet);
      simulatedRobot->enableGravity(false);
    }
  }
  else if(mode == SystemCall::logFileReplay)
  {
    this->logFile = logFile;
    if(logPlayer.open(logFile))
    {
      updateAnnotationsFromLog();
      logPlayer.state = LogPlayer::playing;
      if(!ctrl->is2D)
      {
        SimRobot::Object* puppet = RoboCupCtrl::application->resolveObject("RoboCup.puppets." + QString::fromStdString(robotName), SimRobotCore3::body);
        if(puppet)
        {
          simulatedRobot = std::make_unique<SimulatedRobot3D>(puppet);
          simulatedRobot->enableGravity(false);
        }
      }
    }
    else
    {
      ctrl->printLn("Error: Cannot open log file " + logFile);
    }
  }
  else if(mode == SystemCall::simulatedRobot)
  {
    SimRobot::Object* robot = RoboCupCtrl::application->resolveObject("RoboCup.robots." + QString::fromStdString(robotName), ctrl->is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body));
    ASSERT(robot);
    if(ctrl->is2D)
      simulatedRobot = std::make_unique<SimulatedRobot2D>(robot);
    else
      simulatedRobot = std::make_unique<SimulatedRobot3D>(robot);
    ctrl->gameController.registerSimulatedRobot(SimulatedRobot::getNumber(robot) - 1, *simulatedRobot);
    if(!ctrl->is2D)
    {
      SimRobot::Object* referee = RoboCupCtrl::application->resolveObject("RoboCup.referee", static_cast<int>(SimRobotCore3::body));
      ctrl->gameController.registerReferee(referee);
    }
  }
}

void LocalConsole::init()
{
  RobotConsole::init();
#ifdef MACOS
  setPriority(2);
#endif
}

bool LocalConsole::main()
{
  if(updateSignal.tryWait())
  {
    {
      // Only one thread can access *this now.
      SYNC;

      if(mode == SystemCall::simulatedRobot)
      {
        if(!ctrl->is2D)
        {
          if(jointLastTimestampSent != jointSensorData.timestamp)
          {
            debugSender->bin(idFrameBegin) << "Motion";
            debugSender->bin(idJointSensorData) << jointSensorData;
            debugSender->bin(idFsrSensorData) << fsrSensorData;
            debugSender->bin(idRawInertialSensorData) << rawInertialSensorData;
            debugSender->bin(idGroundTruthOdometryData) << odometryData;
            debugSender->bin(idTorsoMatrix) << torsoMatrix;
            debugSender->bin(idFrameFinished) << "Motion";
            jointLastTimestampSent = jointSensorData.timestamp;
          }

          if(imageLastTimestampSent != cameraImage.timestamp)
          {
            std::string perception = cameraInfo.getThreadName();
            debugSender->bin(idFrameBegin) << perception;
            if(imageCalculated)
              debugSender->bin(idCameraImage) << cameraImage;
            else
            {
              FrameInfo frameInfo;
              frameInfo.time = cameraImage.timestamp;
              debugSender->bin(idFrameInfo) << frameInfo;
            }
            debugSender->bin(idCameraInfo) << cameraInfo;
            debugSender->bin(idGroundTruthWorldState) << worldState;
            debugSender->bin(idFrameFinished) << perception;

            debugSender->bin(idFrameBegin) << "Cognition";
            debugSender->bin(idGameControllerData) << gameControllerData;
            debugSender->bin(idGroundTruthWorldState) << worldState;
            debugSender->bin(idFrameFinished) << "Cognition";
            debugSender->bin(idFrameBegin) << "Audio";
            debugSender->bin(idWhistle) << whistle;
            debugSender->bin(idFrameFinished) << "Audio";
            imageLastTimestampSent = cameraImage.timestamp;
          }
        }
        else
        {
          debugSender->bin(idFrameBegin) << "Cognition";
          FrameInfo frameInfo;
          frameInfo.time = cameraImage.timestamp;
          debugSender->bin(idFrameInfo) << frameInfo;
          debugSender->bin(idCameraInfo) << cameraInfo;
          debugSender->bin(idGroundTruthOdometryData) << odometryData;
          FallDownState fallDownState;
          fallDownState.state = FallDownState::upright;
          debugSender->bin(idFallDownState) << fallDownState;
          GroundContactState groundContactState;
          groundContactState.contact = true;
          debugSender->bin(idGroundContactState) << groundContactState;
          CameraMatrix cameraMatrix;
          cameraMatrix.isValid = false;
          debugSender->bin(idCameraMatrix) << cameraMatrix;
          debugSender->bin(idMotionInfo) << motionInfo;
          debugSender->bin(idGameControllerData) << gameControllerData;
          debugSender->bin(idGroundTruthWorldState) << worldState;
          debugSender->bin(idFrameFinished) << "Cognition";
        }
      }
      debugSender->send(true);
    }

    updatedSignal.post();
  }
  return true;
}

void LocalConsole::update()
{
  RobotConsole::update();
  updatedSignal.wait();

  QString statusText;
  {
    // Only one thread can access *this now.
    SYNC;

    if(mode == SystemCall::remoteRobot && simulatedRobot)
    {
      simulatedRobot->setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData), true);
      const Angle headYaw = RobotConsole::jointSensorData.angles[Joints::headYaw];
      Vector3f orientation = RobotConsole::rawInertialSensorData.angle.cast<float>();
      if(std::abs(headYaw) > 30_deg)
        orientation.z() = -headYaw + sgn(headYaw) * 30_deg;
      simulatedRobot->moveRobot({ 0, 0, -1000.f }, orientation, true, true);
    }
    else if(mode == SystemCall::logFileReplay)
    {
      if(logPlayer.state == LogPlayer::playing && (logPlayer.cycle || logPlayer.frame() + 1 < logPlayer.frames()))
      {
        const std::string threadName = logPlayer.threadOf(logPlayer.frame() +  1);
        if(threadName != "" && threadData[threadName].logAcknowledged)
        {
          logPlayer.playBack(logPlayer.frame() + 1);
          threadData[threadName].currentFrame = logPlayer.frame();
          threadData[threadName].logAcknowledged = false;
        }
      }
      if(simulatedRobot)
      {
        if(RobotConsole::jointSensorData.timestamp)
          simulatedRobot->setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData), true);
        else
          simulatedRobot->getAndSetJointData(jointRequest, jointSensorData);
        simulatedRobot->moveRobot({0, 0, 1000.f}, RobotConsole::rawInertialSensorData.angle.cast<float>(), true, true);
      }
    }
    if(mode == SystemCall::simulatedRobot)
    {
      unsigned now = Time::getCurrentSystemTime();
      if(now >= nextImageTimestamp)
      {
        unsigned newNextImageTimestamp = ctrl->globalNextImageTimestamp;
        if(newNextImageTimestamp == nextImageTimestamp)
        {
          int imageDelay = (2000 / ctrl->calculateImageFps + 1) >> 1;
          int duration = now - ctrl->globalNextImageTimestamp;
          ctrl->globalNextImageTimestamp = (duration >= imageDelay ? now : ctrl->globalNextImageTimestamp) + imageDelay;
          newNextImageTimestamp = ctrl->globalNextImageTimestamp;
        }
        nextImageTimestamp = newNextImageTimestamp;

        if((imageCalculated = ctrl->calculateImage))
          simulatedRobot->getImage(cameraImage, cameraInfo);
        else
        {
          simulatedRobot->getCameraInfo(cameraInfo);
          cameraImage.timestamp = now;
        }
        simulatedRobot->getRobotPose(robotPose);
        simulatedRobot->getWorldState(worldState);
        simulatedRobot->toggleCamera();
      }
      else
        simulatedRobot->getRobotPose(robotPose);

      if(jointCalibrationChanged)
      {
        simulatedRobot->setJointCalibration(jointCalibration);
        jointCalibrationChanged = false;
      }
      simulatedRobot->getOdometryData(robotPose, odometryData);
      simulatedRobot->getSensorData(fsrSensorData, rawInertialSensorData);
      simulatedRobot->getAndSetJointData(jointRequest, jointSensorData);
      simulatedRobot->getAndSetMotionData(motionRequest, motionInfo);
      simulatedRobot->getTorsoMatrix(torsoMatrix);

      ctrl->gameController.getGameControllerData(gameControllerData);
      ctrl->gameController.getWhistle(whistle);
    }

    if(mode == SystemCall::logFileReplay)
    {
      statusText = QString("replaying ") +
#ifdef WINDOWS
                   QString::fromLatin1(logFile.c_str())
#else
                   logFile.c_str()
#endif
                   + " ";
      if(logPlayer.frame() + 1 < logPlayer.frames())
        statusText += QString("%1").arg(static_cast<int>(logPlayer.frame()));
      else
        statusText += "finished";
    }
    else if(!logPlayer.empty())
      statusText += QString("recorded %1 mb").arg(static_cast<float>(logPlayer.size()) / 1000000.f, 0, 'f', 1);
  }

  updateSignal.post();
  trigger(); // invoke a call of main()

  if(pollingFor)
  {
    statusText += statusText != "" ? ", polling for " : "polling for ";
    statusText += pollingFor;
  }

  if(statusText.size() > 0)
    ctrl->printStatusText((QString::fromStdString(robotName) + ": " + statusText).toUtf8());
}

DebugReceiver<MessageQueue>* LocalConsole::connectReceiverWithRobot(Debug* debug)
{
  DebugReceiver<MessageQueue>* receiver = new DebugReceiver<MessageQueue>(this, debug->getName());
  debug->debugSender->setReceiver(*receiver);
  return receiver;
}

DebugSender<MessageQueue>* LocalConsole::connectSenderWithRobot(Debug* debug) const
{
  return new DebugSender<MessageQueue>(*debug->debugReceiver, debug->getName());
}
