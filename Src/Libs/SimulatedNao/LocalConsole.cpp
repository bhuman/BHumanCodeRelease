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
  RobotConsole(settings, robotName, ctrl, logFile.empty() ? SystemCall::simulatedRobot : SystemCall::logFileReplay, connectReceiverWithRobot(debug), connectSenderWithRobot(debug)),
  updatedSignal(1)
{
  addPerRobotViews();

  if(mode == SystemCall::logFileReplay)
  {
    this->logFile = logFile;
    if(logPlayer.open(logFile))
    {
      updateAnnotationsFromLog();
      logPlayer.play();
      if(!ctrl->is2D)
      {
        SimRobot::Object* puppet = RoboCupCtrl::application->resolveObject("RoboCup.puppets." + QString::fromStdString(robotName), SimRobotCore2::body);
        if(puppet)
          simulatedRobot = std::make_unique<SimulatedRobot3D>(puppet);
      }
    }
    else
    {
      ctrl->printLn("Error: Cannot open log file " + logFile);
    }
  }
  else if(mode == SystemCall::simulatedRobot)
  {
    SimRobot::Object* robot = RoboCupCtrl::application->resolveObject("RoboCup.robots." + QString::fromStdString(robotName), ctrl->is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore2::body));
    ASSERT(robot);
    if(ctrl->is2D)
      simulatedRobot = std::make_unique<SimulatedRobot2D>(robot);
    else
      simulatedRobot = std::make_unique<SimulatedRobot3D>(robot);
    ctrl->gameController.registerSimulatedRobot(SimulatedRobot::getNumber(robot) - 1, *simulatedRobot);
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
            debugSender->out.bin << "Motion";
            debugSender->out.finishMessage(idFrameBegin);
            debugSender->out.bin << jointSensorData;
            debugSender->out.finishMessage(idJointSensorData);
            debugSender->out.bin << fsrSensorData;
            debugSender->out.finishMessage(idFsrSensorData);
            debugSender->out.bin << inertialSensorData;
            debugSender->out.finishMessage(idInertialSensorData);
            debugSender->out.bin << odometryData;
            debugSender->out.finishMessage(idGroundTruthOdometryData);
            debugSender->out.bin << "Motion";
            debugSender->out.finishMessage(idFrameFinished);
            jointLastTimestampSent = jointSensorData.timestamp;
          }

          if(imageLastTimestampSent != cameraImage.timestamp)
          {
            std::string perception = TypeRegistry::getEnumName(cameraInfo.camera);
            perception[0] &= ~0x20;
            debugSender->out.bin << perception;
            debugSender->out.finishMessage(idFrameBegin);
            if(imageCalculated)
            {
              debugSender->out.bin << cameraImage;
              debugSender->out.finishMessage(idCameraImage);
            }
            else
            {
              FrameInfo frameInfo;
              frameInfo.time = cameraImage.timestamp;
              debugSender->out.bin << frameInfo;
              debugSender->out.finishMessage(idFrameInfo);
            }
            debugSender->out.bin << cameraInfo;
            debugSender->out.finishMessage(idCameraInfo);
            debugSender->out.bin << worldState;
            debugSender->out.finishMessage(idGroundTruthWorldState);
            debugSender->out.bin << perception;
            debugSender->out.finishMessage(idFrameFinished);

            debugSender->out.bin << "Cognition";
            debugSender->out.finishMessage(idFrameBegin);
            debugSender->out.bin << gameControllerData;
            debugSender->out.finishMessage(idGameControllerData);
            debugSender->out.bin << worldState;
            debugSender->out.finishMessage(idGroundTruthWorldState);
            debugSender->out.bin << "Cognition";
            debugSender->out.finishMessage(idFrameFinished);
            debugSender->out.bin << "Audio";
            debugSender->out.finishMessage(idFrameBegin);
            debugSender->out.bin << whistle;
            debugSender->out.finishMessage(idWhistle);
            debugSender->out.bin << "Audio";
            debugSender->out.finishMessage(idFrameFinished);
            imageLastTimestampSent = cameraImage.timestamp;
          }
        }
        else
        {
          debugSender->out.bin << "Cognition";
          debugSender->out.finishMessage(idFrameBegin);
          FrameInfo frameInfo;
          frameInfo.time = cameraImage.timestamp;
          debugSender->out.bin << frameInfo;
          debugSender->out.finishMessage(idFrameInfo);
          debugSender->out.bin << cameraInfo;
          debugSender->out.finishMessage(idCameraInfo);
          debugSender->out.bin << odometryData;
          debugSender->out.finishMessage(idGroundTruthOdometryData);
          {
            FallDownState fallDownState;
            fallDownState.state = FallDownState::upright;
            debugSender->out.bin << fallDownState;
            debugSender->out.finishMessage(idFallDownState);
          }
          {
            GroundContactState groundContactState;
            groundContactState.contact = true;
            debugSender->out.bin << groundContactState;
            debugSender->out.finishMessage(idGroundContactState);
          }
          {
            CameraMatrix cameraMatrix;
            cameraMatrix.isValid = false;
            debugSender->out.bin << cameraMatrix;
            debugSender->out.finishMessage(idCameraMatrix);
          }
          debugSender->out.bin << motionInfo;
          debugSender->out.finishMessage(idMotionInfo);
          debugSender->out.bin << gameControllerData;
          debugSender->out.finishMessage(idGameControllerData);
          debugSender->out.bin << worldState;
          debugSender->out.finishMessage(idGroundTruthWorldState);
          debugSender->out.bin << "Cognition";
          debugSender->out.finishMessage(idFrameFinished);
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

#ifdef MACOS
  pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
#endif

  updatedSignal.wait();

#ifdef MACOS
  pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif

  QString statusText;
  {
    // Only one thread can access *this now.
    SYNC;

    if(mode == SystemCall::logFileReplay)
    {
      std::string threadIdentifier = logPlayer.getThreadIdentifierOfNextFrame();
      if(threadIdentifier != "" && threadData[threadIdentifier].logAcknowledged && logPlayer.replay())
        threadData[threadIdentifier].logAcknowledged = false;
      if(simulatedRobot)
      {
        if(RobotConsole::jointSensorData.timestamp)
          simulatedRobot->setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData));
        else
          simulatedRobot->getAndSetJointData(jointRequest, jointSensorData);
        simulatedRobot->moveRobot({0, 0, 1000.f}, RobotConsole::inertialSensorData.angle.cast<float>(), true);
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
      simulatedRobot->getSensorData(fsrSensorData, inertialSensorData);
      simulatedRobot->getAndSetJointData(jointRequest, jointSensorData);
      simulatedRobot->getAndSetMotionData(motionRequest, motionInfo);

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
      if(logPlayer.currentFrameNumber != -1)
      {
        char buf[33];
        sprintf(buf, "%u", logPlayer.currentFrameNumber);
        statusText += buf;
      }
      else
        statusText += "finished";
    }
    else if(logPlayer.numberOfFrames != 0)
    {
      statusText += QString("recorded ");
      char buf[33];
      sprintf(buf, "%u", logPlayer.numberOfFrames);
      statusText += buf;
    }
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
  ASSERT(!debug->debugSender);
  DebugReceiver<MessageQueue>* receiver = new DebugReceiver<MessageQueue>(this, debug->getName());
  debug->debugSender = new DebugSender<MessageQueue>(*receiver, "LocalConsole");
  return receiver;
}

DebugSender<MessageQueue>* LocalConsole::connectSenderWithRobot(Debug* debug) const
{
  ASSERT(!debug->debugReceiver);
  debug->debugReceiver = new DebugReceiver<MessageQueue>(debug, "LocalConsole");
  return new DebugSender<MessageQueue>(*debug->debugReceiver, debug->getName());
}
