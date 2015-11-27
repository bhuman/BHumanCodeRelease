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

LocalRobot::LocalRobot()
  : RobotConsole(theDebugReceiver, theDebugSender),
    theDebugReceiver(this, "Receiver.MessageQueue.O"),
    theDebugSender(this, "Sender.MessageQueue.S"),
    image(false),
    nextImageTimeStamp(0),
    imageLastTimeStampSent(0),
    jointLastTimeStampSent(0),
    updatedSignal(1),
    puppet(0)
{
  mode = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getMode();
  addViews();

  if(mode == SystemCall::logfileReplay)
  {
    logFile = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getLogFile();
    logPlayer.open(logFile.c_str());
    logPlayer.handleAllMessages(annotationInfos['c']);
    logPlayer.play();
    puppet = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject("RoboCup.puppets." + robotName, SimRobotCore2::body);
    if(puppet)
      simulatedRobot.init(puppet);
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
          debugOut.out.bin << inertialSensorData;
          debugOut.out.finishMessage(idInertialSensorData);
          debugOut.out.bin << usSensorData;
          debugOut.out.finishMessage(idUsSensorData);
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
            frameInfo.cycleTime = 1.f / (float) ctrl->calculateImageFps;
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
        simulatedRobot.getAndSetJointData(jointRequest, jointSensorData);
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
      unsigned now = SystemCall::getCurrentSystemTime();
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
        {
          simulatedRobot.getImage(image, cameraInfo);
        }
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

      simulatedRobot.getOdometryData(robotPose, odometryData);
      simulatedRobot.getSensorData(inertialSensorData, usSensorData, usRequest);
      simulatedRobot.getAndSetJointData(jointRequest, jointSensorData);
    }

    std::string statusText;
    if(mode == SystemCall::logfileReplay)
    {
      statusText = std::string("replaying ") + logFile + " ";
      if(logPlayer.currentFrameNumber != -1)
      {
        char buf[33];
        sprintf(buf, "%u", logPlayer.currentFrameNumber + 1);
        statusText += buf;
      }
      else
        statusText += "finished";
    }

    if(mode != SystemCall::logfileReplay && logPlayer.numberOfFrames != 0)
    {
      if(statusText != "")
        statusText += ", ";
      statusText += std::string("recorded ");
      char buf[33];
      sprintf(buf, "%u", logPlayer.numberOfFrames);
      statusText += buf;
    }

    if(pollingFor)
    {
      statusText += statusText != "" ? ", polling for " : "polling for ";
      statusText += pollingFor;
    }

    if(!statusText.empty())
      ((ConsoleRoboCupCtrl*)ConsoleRoboCupCtrl::controller)->printStatusText((robotName + ": " + statusText.c_str()).toUtf8().constData());
  }

  updateSignal.post();
  trigger(); // invoke a call of main()
}
