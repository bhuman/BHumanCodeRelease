/**
 * @file Controller/LocalRobot.cpp
 *
 * Implementation of LocalRobot.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 */

#include "LocalRobot.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Platform/Time.h"

LocalRobot::LocalRobot() :
  RobotConsole(theDebugReceiver, theDebugSender),
  theDebugReceiver(this),
  theDebugSender(this),
  image(false), updatedSignal(1)
{
  mode = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getMode();
  addViews();

  if(mode == SystemCall::logfileReplay)
  {
    logFile = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getLogFile();
    if(logPlayer.open(logFile))
    {
      logPlayer.handleAllMessages(annotationInfos);
      logPlayer.play();
      puppet = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject("RoboCup.puppets." + robotName, SimRobotCore2::body);
      if(puppet)
        simulatedRobot.init(puppet);
    }
    else
    {
      ctrl->printLn("Error: Cannot open log file " + logFile);
    }
  }
  else if(mode == SystemCall::simulatedRobot)
  {
    SimRobotCore2::Body* robot = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject(RoboCupCtrl::getRobotFullName(), SimRobotCore2::body);
    ASSERT(robot);
    simulatedRobot.init(robot);
    ctrl->gameController.registerSimulatedRobot(robotName.mid(5).toInt() - 1, simulatedRobot);
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
        if(jointLastTimeStampSent != jointSensorData.timestamp)
        {
          debugOut.out.bin << 'm';
          debugOut.out.finishMessage(idProcessBegin);
          debugOut.out.bin << jointSensorData;
          debugOut.out.finishMessage(idJointSensorData);
          debugOut.out.bin << fsrSensorData;
          debugOut.out.finishMessage(idFsrSensorData);
          debugOut.out.bin << inertialSensorData;
          debugOut.out.finishMessage(idInertialSensorData);
          debugOut.out.bin << odometryData;
          debugOut.out.finishMessage(idGroundTruthOdometryData);
          ctrl->gameController.writeGameInfo(debugOut.out.bin);
          debugOut.out.finishMessage(idGameInfo);
          int robot = robotName.mid(5).toInt() - 1;
          ctrl->gameController.writeOwnTeamInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idOwnTeamInfo);
          ctrl->gameController.writeOpponentTeamInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idOpponentTeamInfo);
          ctrl->gameController.writeRobotInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idRobotInfo);
          debugOut.out.bin << 'm';
          debugOut.out.finishMessage(idProcessFinished);
          jointLastTimeStampSent = jointSensorData.timestamp;
        }

        if(imageLastTimeStampSent != image.timeStamp)
        {
          debugOut.out.bin << 'c';
          debugOut.out.finishMessage(idProcessBegin);
          if(ctrl->calculateImage)
          {
            debugOut.out.bin << image;
            debugOut.out.finishMessage(idImage);
          }
          else
          {
            FrameInfo frameInfo;
            frameInfo.time = image.timeStamp;
            debugOut.out.bin << frameInfo;
            debugOut.out.finishMessage(idFrameInfo);
          }
          debugOut.out.bin << cameraInfo;
          debugOut.out.finishMessage(idCameraInfo);
          debugOut.out.bin << worldState;
          debugOut.out.finishMessage(idGroundTruthWorldState);
          debugOut.out.bin << 'c';
          debugOut.out.finishMessage(idProcessFinished);
          imageLastTimeStampSent = image.timeStamp;
        }
      }
      theDebugSender.send(true);
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

    if(mode == SystemCall::logfileReplay)
    {
      if(logAcknowledged && logPlayer.replay())
        logAcknowledged = false;
      if(puppet)
      {
        if(RobotConsole::jointSensorData.timestamp)
          simulatedRobot.setJointRequest(reinterpret_cast<JointRequest&>(RobotConsole::jointSensorData));
        else
          simulatedRobot.getAndSetJointData(jointRequest, jointSensorData);
      }
    }
    if(mode == SystemCall::simulatedRobot || puppet)
    {
      if(moveOp != noMove)
      {
        if(moveOp == moveBoth)
          simulatedRobot.moveRobot(movePos, moveRot * 1_deg, true);
        else if(moveOp == movePosition)
          simulatedRobot.moveRobot(movePos, Vector3f::Zero(), false);
        else if(moveOp == moveBallPosition)
          simulatedRobot.moveBall(movePos);
        moveOp = noMove;
      }
    }
    if(mode == SystemCall::simulatedRobot)
    {
      unsigned now = Time::getCurrentSystemTime();
      if(now >= nextImageTimeStamp)
      {
        unsigned newNextImageTimeStamp = ctrl->globalNextImageTimeStamp;
        if(newNextImageTimeStamp == nextImageTimeStamp)
        {
          int imageDelay = (2000 / ctrl->calculateImageFps + 1) >> 1;
          int duration = now - ctrl->globalNextImageTimeStamp;
          ctrl->globalNextImageTimeStamp = (duration >= imageDelay ? now : ctrl->globalNextImageTimeStamp) + imageDelay;
          newNextImageTimeStamp = ctrl->globalNextImageTimeStamp;
        }
        nextImageTimeStamp = newNextImageTimeStamp;

        if(ctrl->calculateImage)
          simulatedRobot.getImage(image, cameraInfo);
        else
        {
          simulatedRobot.getCameraInfo(cameraInfo);
          image.timeStamp = now;
        }
        simulatedRobot.getRobotPose(robotPose);
        simulatedRobot.getWorldState(worldState);
        simulatedRobot.toggleCamera();
      }
      else
        simulatedRobot.getRobotPose(robotPose);

      if(jointCalibrationChanged)
      {
        simulatedRobot.setJointCalibration(jointCalibration);
        jointCalibrationChanged = false;
      }
      simulatedRobot.getOdometryData(robotPose, odometryData);
      simulatedRobot.getSensorData(fsrSensorData, inertialSensorData);
      simulatedRobot.getAndSetJointData(jointRequest, jointSensorData);
    }

    QString statusText;
    if(mode == SystemCall::logfileReplay)
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

    if(mode != SystemCall::logfileReplay && logPlayer.numberOfFrames != 0)
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
      ((ConsoleRoboCupCtrl*)ConsoleRoboCupCtrl::controller)->printStatusText((robotName + ": " + statusText).toUtf8());
  }

  updateSignal.post();
  trigger(); // invoke a call of main()
}
