/**
 * @file Controller/LocalRobot.cpp
 *
 * Implementation of LocalRobot.
 *
 * @author Thomas Röfer
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 */

#include "LocalRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/SimulatedRobot2D.h"
#include "Controller/SimulatedRobot3D.h"
#include "Platform/Time.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Threads/Debug.h"

LocalRobot::LocalRobot(const Settings& settings, const std::string& robotName, Debug* debug) :
  RobotConsole(settings, robotName, connectReceiverWithRobot(debug), connectSenderWithRobot(debug)),
  updatedSignal(1)
{
  mode = static_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller)->getMode();
  addPerRobotViews();

  if(mode == SystemCall::logFileReplay)
  {
    logFile = static_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller)->getLogFile();
    if(logPlayer.open(logFile))
    {
      updateAnnotationsFromLog();
      logPlayer.play();
      if(!RoboCupCtrl::controller->is2D)
      {
        puppet = static_cast<SimRobotCore2::Body*>(RoboCupCtrl::application->resolveObject("RoboCup.puppets." + QString::fromStdString(robotName), SimRobotCore2::body));
        if(puppet)
          simulatedRobot = std::make_unique<SimulatedRobot3D>(puppet);
      }
    }
    else
    {
      printLn("Error: Cannot open log file " + logFile);
    }
  }
  else if(mode == SystemCall::simulatedRobot)
  {
    SimRobot::Object* robot = RoboCupCtrl::application->resolveObject("RoboCup.robots." + QString::fromStdString(robotName), RoboCupCtrl::controller->is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore2::body));
    ASSERT(robot);
    if(RoboCupCtrl::controller->is2D)
      simulatedRobot = std::make_unique<SimulatedRobot2D>(robot);
    else
      simulatedRobot = std::make_unique<SimulatedRobot3D>(robot);
    ctrl->gameController.registerSimulatedRobot(SimulatedRobot::getNumber(robot) - 1, *simulatedRobot);
  }
}

bool LocalRobot::main()
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
            if(ctrl->calculateImage)
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
            ctrl->gameController.writeGameInfo(debugSender->out.bin);
            debugSender->out.finishMessage(idRawGameInfo);
            const int robot = std::atoi(robotName.substr(5).c_str()) - 1;
            ctrl->gameController.writeOwnTeamInfo(robot, debugSender->out.bin);
            debugSender->out.finishMessage(idOwnTeamInfo);
            ctrl->gameController.writeOpponentTeamInfo(robot, debugSender->out.bin);
            debugSender->out.finishMessage(idOpponentTeamInfo);
            ctrl->gameController.writeRobotInfo(robot, debugSender->out.bin);
            debugSender->out.finishMessage(idRobotInfo);
            debugSender->out.bin << worldState;
            debugSender->out.finishMessage(idGroundTruthWorldState);
            debugSender->out.bin << "Cognition";
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
          ctrl->gameController.writeGameInfo(debugSender->out.bin);
          debugSender->out.finishMessage(idGameInfo);
          const int robot = std::atoi(robotName.substr(5).c_str()) - 1;
          ctrl->gameController.writeOwnTeamInfo(robot, debugSender->out.bin);
          debugSender->out.finishMessage(idOwnTeamInfo);
          ctrl->gameController.writeOpponentTeamInfo(robot, debugSender->out.bin);
          debugSender->out.finishMessage(idOpponentTeamInfo);
          ctrl->gameController.writeRobotInfo(robot, debugSender->out.bin);
          debugSender->out.finishMessage(idRobotInfo);
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

void LocalRobot::update()
{
  RobotConsole::update();

  updatedSignal.wait();

  // Only one thread can access *this now.
  {
    SYNC;

    if(mode == SystemCall::logFileReplay)
    {
      std::string threadIdentifier = logPlayer.getThreadIdentifierOfNextFrame();
      if(threadIdentifier != "" && threadData[threadIdentifier].logAcknowledged && logPlayer.replay())
        threadData[threadIdentifier].logAcknowledged = false;
      if(puppet)
      {
        if(RobotConsole::jointSensorData.timestamp)
          simulatedRobot->setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData));
        else
          simulatedRobot->getAndSetJointData(jointRequest, jointSensorData);
      }
    }
    if(mode == SystemCall::simulatedRobot || puppet)
    {
      if(moveOp != noMove)
      {
        if(moveOp == moveBoth)
          simulatedRobot->moveRobot(movePos, moveRot * 1_deg, true);
        else if(moveOp == movePosition)
          simulatedRobot->moveRobot(movePos, Vector3f::Zero(), false);
        else if(moveOp == moveBallPosition)
          simulatedRobot->moveBall(movePos);
        moveOp = noMove;
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

        if(ctrl->calculateImage)
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
    }

    QString statusText;
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

    if(mode != SystemCall::logFileReplay && logPlayer.numberOfFrames != 0)
    {
      if(statusText != "")
        statusText += ", ";
      statusText += QString("recorded ");
      char buf[33];
      sprintf(buf, "%u", logPlayer.numberOfFrames);
      statusText += buf;
    }

    if(pollingFor)
    {
      statusText += statusText != "" ? ", polling for " : "polling for ";
      statusText += pollingFor;
    }

    if(statusText.size() > 0)
      ((ConsoleRoboCupCtrl*)ConsoleRoboCupCtrl::controller)->printStatusText((QString::fromStdString(robotName) + ": " + statusText).toUtf8());
  }

  updateSignal.post();
  trigger(); // invoke a call of main()
}

DebugReceiver<MessageQueue>* LocalRobot::connectReceiverWithRobot(Debug* debug)
{
  ASSERT(!debug->debugSender);
  DebugReceiver<MessageQueue>* receiver = new DebugReceiver<MessageQueue>(this, debug->getName());
  debug->debugSender = new DebugSender<MessageQueue>(*receiver, "LocalRobot");
  return receiver;
}

DebugSender<MessageQueue>* LocalRobot::connectSenderWithRobot(Debug* debug) const
{
  ASSERT(!debug->debugReceiver);
  debug->debugReceiver = new DebugReceiver<MessageQueue>(debug, "LocalRobot");
  return new DebugSender<MessageQueue>(*debug->debugReceiver, debug->getName());
}
