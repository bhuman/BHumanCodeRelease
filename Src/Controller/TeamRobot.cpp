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
    DECLARE_DEBUG_DRAWING("representation:TeamBallModel", "drawingOnField"); // drawing of the team ball model
    DECLARE_DEBUG_DRAWING("representation:TeamPlayersModel", "drawingOnField"); // drawing of the team players model
    DECLARE_DEBUG_DRAWING("representation:GoalPercept:Field", "drawingOnField"); // drawing of the goal percept
    DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField"); // drawing of a request walk vector
    DECLARE_DEBUG_DRAWING("representation:ObstacleModelCompressed:CenterCross", "drawingOnField"); //drawing center of obstacles from obstacle model
    DECLARE_DEBUG_DRAWING("representation:LinePercept:Field", "drawingOnField");

    uint8_t teamColor = 0,
            swapSides = 0;
    MODIFY("teamColor", ownTeamInfo.teamColor);
    MODIFY("swapSides", swapSides);

    if(SystemCall::getTimeSince(robotPoseReceived) < 1000)
      robotPose.draw();
    if(SystemCall::getTimeSince(ballModelReceived) < 1000)
      ballModel.draw();
    if(SystemCall::getTimeSince(teamBallModelReceived) < 1000)
      teamBallModel.draw();
    if(SystemCall::getTimeSince(teamPlayersModelReceived) < 1000)
      teamPlayersModel.draw();
    if(SystemCall::getTimeSince(goalPerceptReceived) < 1000)
      goalPercept.draw();
    if(SystemCall::getTimeSince(motionRequestReceived) < 1000)
      motionRequest.draw();
    if(SystemCall::getTimeSince(obstacleModelCompressedReceived) < 1000)
      obstacleModelCompressed.draw();
    if(SystemCall::getTimeSince(linePerceptReceived) < 1000)
      linePercept.draw();

    if(swapSides ^ teamColor)
    {
      ORIGIN("field polygons", 0, 0, pi2); // hack: swap sides!
    }
    fieldDimensions.draw();
    fieldDimensions.drawPolygons(teamColor);

    DECLARE_DEBUG_DRAWING("robotState", "drawingOnField"); // text decribing the state of the robot
    int lineY = 3550;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorRGBA::white, "batteryLevel: " << robotHealth.batteryLevel << " %");
    DRAWTEXT("robotState", 3700, lineY, 150, ColorRGBA::white, "role: " << Role::getName(behaviorStatus.role));
    lineY -= 180;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorRGBA::white, "temperatures: joint " << Joints::getName(robotHealth.jointWithMaxTemperature)<< ":" << robotHealth.maxJointTemperature << " C, cpu: " << robotHealth.cpuTemperature << " C");
    lineY -= 180;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorRGBA::white, "rates: cognition: " << (int) std::floor(robotHealth.cognitionFrameRate + 0.5f) << " fps, motion: " << (int) std::floor(robotHealth.motionFrameRate + 0.5f) << " fps");
    if(ballModel.timeWhenLastSeen)
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorRGBA::white, "ballLastSeen: " << SystemCall::getRealTimeSince(ballModel.timeWhenLastSeen) << " ms");
    }
    else
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorRGBA::white, "ballLastSeen: never");
    }
    //DRAWTEXT("robotState", -5100, lineY, 150, ColorRGBA::white, "ballPercept: " << ballModel.lastPerception.position.x << ", " << ballModel.lastPerception.position.y);
    lineY -= 180;
    DRAWTEXT("robotState", -5100, lineY, 150, ColorRGBA::white, "memory usage: " << robotHealth.memoryUsage << " %");
    if(goalPercept.timeWhenGoalPostLastSeen)
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorRGBA::white, "goalLastSeen: " << SystemCall::getRealTimeSince(goalPercept.timeWhenGoalPostLastSeen) << " ms");
    }
    else
    {
      DRAWTEXT("robotState", 3700, lineY, 150, ColorRGBA::white, "goalLastSeen: never");
    }

    lineY -= 180;

    DRAWTEXT("robotState", -5100, lineY, 150, ColorRGBA::white, "load average: " << (float(robotHealth.load[0]) / 10.f) << " " << (float(robotHealth.load[1]) / 10.f) << " " << (float(robotHealth.load[2]) / 10.f));
    DRAWTEXT("robotState", -4100, 0, 150, ColorRGBA(255, 255, 255, 50), robotHealth.robotName);

    DECLARE_DEBUG_DRAWING("robotOffline", "drawingOnField"); // A huge X to display the online/offline state of the robot
    if(SystemCall::getTimeSince(robotPoseReceived) > 500
       || (SystemCall::getTimeSince(isPenalizedReceived) < 1000 && isPenalized)
       || (SystemCall::getTimeSince(hasGroundContactReceived) < 1000 && !hasGroundContact)
       || (SystemCall::getTimeSince(isUprightReceived) < 1000 && !isUpright))
    {
      LINE("robotOffline", -5100, 3600, 5100, -3600, 50, Drawings::solidPen, ColorRGBA(0xff, 0, 0));
      LINE("robotOffline", 5100, 3600, -5100, -3600, 50, Drawings::solidPen, ColorRGBA(0xff, 0, 0));
    }
    if(SystemCall::getTimeSince(isPenalizedReceived) < 1000 && isPenalized)
    {
      // Draw a text in red letters to tell that the robot is penalized
      DRAWTEXT("robotState", -2000, 0, 200, ColorRGBA::red, "PENALIZED");
    }
    if(SystemCall::getTimeSince(hasGroundContactReceived) < 1000 && !hasGroundContact)
    {
      // Draw a text in red letters to tell that the robot doesn't have ground contact
      DRAWTEXT("robotState", 300, 0, 200, ColorRGBA::red, "NO GROUND CONTACT");
    }
    if(SystemCall::getTimeSince(isUprightReceived) < 1000 && !isUpright)
    {
      // Draw a text in red letters to tell that the robot is fallen down
      DRAWTEXT("robotState", 300, 0, 200, ColorRGBA::red, "NOT UPRIGHT");
    }

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

    OUTPUT(idProcessFinished, bin, 't');
    teamOut.moveAllMessages(teamIn);
  }
  SystemCall::sleep(50);
  return false;
}

TeamRobot::TeamRobot(const char* name, int number) :
  RobotConsole((setGlobals(), teamIn), teamOut), number(number),
  fieldDimensions(Blackboard::getInstance().alloc<FieldDimensions>("FieldDimensions")),
  ownTeamInfo(Blackboard::getInstance().alloc<OwnTeamInfo>("OwnTeamInfo"))
{
  strcpy(this->name, name);
  mode = SystemCall::teamRobot;
  fieldDimensions.load();
}

TeamRobot::~TeamRobot()
{
  Thread<TeamRobot>::stop();
  setGlobals();
  Blackboard::getInstance().free("FieldDimensions");
  Blackboard::getInstance().free("OwnTeamInfo");
}
