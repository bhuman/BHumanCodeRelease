/**
 * @file Demo.h
 *
 * This file declares complex skills related to demonstrations.
 *
 * @author Thomas Röfer
 */

/** This skill makes the robot go to a ball and kick it for public demos. */
option(DemoGoToBallAndKick);

/** This skill makes the robot pose for public demos. */
option(DemoPose);

/** This skill makes the robot search for the ball for public demos. */
option(DemoSearchForBall);

/** This skill makes the robot talk for public demos. */
option(DemoTalk);
option(DemoTalkWaitForKey);
option(DemoWaitAndPlay, args((const std::string&) soundName));

/** This skill makes the robot wave for public demos. */
option(DemoWave);
