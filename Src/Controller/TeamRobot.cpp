/**
* @file Controller/TeamRobot.cpp
* Implementation of a class representing a process that communicates with a remote robot via team communication.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "TeamRobot.h"
#define Drawings ::Drawings // Base class also defines Drawings, but need the global class

bool TeamRobot::main()
{
  {
    SYNC;
    OUTPUT(idProcessBegin, bin, 't');

    DECLARE_DEBUG_DRAWING("representation:RobotPose", "drawingOnField"); // The robot pose
    DECLARE_DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField"); // The robot pose
    DECLARE_DEBUG_DRAWING("origin:RobotPose", "drawingOnField"); // Set the origin to the robot's current position
    DECLARE_DEBUG_DRAWING("representation:BallModel", "drawingOnField"); // drawing of the ball model
    DECLARE_DEBUG_DRAWING("representation:CombinedWorldModel", "drawingOnField"); // drawing of the combined world model
    DECLARE_DEBUG_DRAWING("representation:RobotsModel:robots", "drawingOnField"); // drawing of the robots model a
    DECLARE_DEBUG_DRAWING("representation:RobotsModel:covariance", "drawingOnField"); // drawing of the robots model b
    DECLARE_DEBUG_DRAWING("representation:GoalPercept:Field", "drawingOnField"); // drawing of the goal percept
    DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField"); // drawing of a request walk vector
    DECLARE_DEBUG_DRAWING("representation:FreePartOfOpponentGoalModel", "drawingOnField"); // drawing of free part
    DECLARE_DEBUG_DRAWING("representation:ObstacleModelReduced", "drawingOnField"); // drawing of obstacle model
    DECLARE_DEBUG_DRAWING("representation:LinePercept:Field", "drawingOnField");
    DECLARE_PLOT("tgb");
    DECLARE_PLOT("tob");

    int teamColor = 0,
        swapSides = 0;
    MODIFY("teamColor", teamColor);
    MODIFY("swapSides", swapSides);

    if(SystemCall::getTimeSince(robotPoseReceived) < 1000)
      robotPose.draw(!teamColor);
    if(SystemCall::getTimeSince(robotsModelReceived) < 1000)
      robotsModel.draw();
    if(SystemCall::getTimeSince(ballModelReceived) < 1000)
      ballModel.draw();
    if(SystemCall::getTimeSince(combinedWorldModelReceived) < 1000)
      combinedWorldModel.draw();
    if(SystemCall::getTimeSince(goalPerceptReceived) < 1000)
      goalPercept.draw();
    if(SystemCall::getTimeSince(motionRequestReceived) < 1000)
      motionRequest.draw();
    if(SystemCall::getTimeSince(obstacleModelReceived) < 1000)
      obstacleModel.draw();
    if(SystemCall::getTimeSince(freePartOfOpponentGoalModelReceived) < 1000)
      freePartOfOpponentGoalModel.draw();
    if(SystemCall::getTimeSince(linePerceptReceived) < 1000)
      linePercept.drawOnField(fieldDimensions, 0);

    if(swapSides ^ teamColor)
    {
      ORIGIN("field polygons", 0, 0, pi2); // hack: swap sides!
    }
    fieldDimensions.draw();
    fieldDimensions.drawPolygons(teamColor);

    DECLARE_DEBUG_DRAWING("robotState", "drawingOnField"); // text decribing the state of the robot
    int lineY = 3550;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorClasses::white, "batteryLevel: " << robotHealth.batteryLevel << " %");
    DRAWTEXT("robotState", 3700, lineY, 150, ColorClasses::white, "role: " << BehaviorStatus::getName(behaviorStatus.role));
    lineY -= 180;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorClasses::white, "temperatures: joint: " << robotHealth.maxJointTemperature << " C, cpu: " << robotHealth.cpuTemperature << " C, board: " << robotHealth.boardTemperature << " C");
    lineY -= 180;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorClasses::white, "rates: cognition: " << roundNumberToInt(robotHealth.cognitionFrameRate) << " fps, motion: " << roundNumberToInt(robotHealth.motionFrameRate) << " fps");
    if(ballModel.timeWhenLastSeen)
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorClasses::white, "ballLastSeen: " << SystemCall::getRealTimeSince(ballModel.timeWhenLastSeen) << " ms");
    }
    else
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorClasses::white, "ballLastSeen: never");
    }
    //DRAWTEXT("robotState", -5100, lineY, 150, ColorClasses::white, "ballPercept: " << ballModel.lastPerception.position.x << ", " << ballModel.lastPerception.position.y);
    lineY -= 180;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorClasses::white, "memory usage: " << robotHealth.memoryUsage << " %");
    if(goalPercept.timeWhenGoalPostLastSeen)
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorClasses::white, "goalLastSeen: " << SystemCall::getRealTimeSince(goalPercept.timeWhenGoalPostLastSeen) << " ms");
    }
    else
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorClasses::white, "goalLastSeen: never");
    }

    lineY -= 180;

    DRAWTEXT("robotState", -5100, lineY, 150, ColorClasses::white, "load average: " << (float(robotHealth.load[0]) / 10.f) << " " << (float(robotHealth.load[1]) / 10.f) << " " << (float(robotHealth.load[2]) / 10.f));
    DRAWTEXT("robotState", -4100, 0, 150, ColorRGBA(255, 255, 255, 50), robotHealth.robotName);

    DECLARE_DEBUG_DRAWING("robotOffline", "drawingOnField"); // A huge X to display the online/offline state of the robot
    if(SystemCall::getTimeSince(robotPoseReceived) > 500
       || (SystemCall::getTimeSince(isPenalizedReceived) < 1000 && isPenalized)
       || (SystemCall::getTimeSince(hasGroundContactReceived) < 1000 && !hasGroundContact)
       || (SystemCall::getTimeSince(isUprightReceived) < 1000 && !isUpright))
    {
      LINE("robotOffline", -5100, 3600, 5100, -3600, 50, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
      LINE("robotOffline", 5100, 3600, -5100, -3600, 50, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    }
    if(SystemCall::getTimeSince(isPenalizedReceived) < 1000 && isPenalized)
    {
      // Draw a text in red letters to tell that the robot is penalized
      DRAWTEXT("robotState", -2000, 0, 200, ColorClasses::red, "PENALIZED");
    }
    if(SystemCall::getTimeSince(hasGroundContactReceived) < 1000 && !hasGroundContact)
    {
      // Draw a text in red letters to tell that the robot doesn't have ground contact
      DRAWTEXT("robotState", 300, 0, 200, ColorClasses::red, "NO GROUND CONTACT");
    }
    if(SystemCall::getTimeSince(isUprightReceived) < 1000 && !isUpright)
    {
      // Draw a text in red letters to tell that the robot is fallen down
      DRAWTEXT("robotState", 300, 0, 200, ColorClasses::red, "NOT UPRIGHT");
    }

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager", OUTPUT(idDrawingManager, bin, Global::getDrawingManager()););
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D", OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D()););
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification", OUTPUT(idStreamSpecification, bin, Global::getStreamHandler()););

    OUTPUT(idProcessFinished, bin, 't');
    teamOut.moveAllMessages(teamIn);
  }
  SystemCall::sleep(50);
  return false;
}

TeamRobot::TeamRobot(const char* name, int number) :
  RobotConsole(teamIn, teamOut), number(number)
{
  strcpy(this->name, name);
  mode = SystemCall::teamRobot;
  fieldDimensions.load();
}
