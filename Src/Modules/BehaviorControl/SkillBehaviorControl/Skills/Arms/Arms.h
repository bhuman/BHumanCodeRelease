/**
 * @file Arms.h
 *
 * This file declares complex skills related to arm motion.
 *
 * @author Thomas Röfer
 */

/**
 * This skill reacts to arm contact by taking the arm away.
 * @param arm The arm for which the contact reaction shall be done.
 *            By default, it is both arms.
 */
option(ArmContact, args((Arms::Arm)(Arms::numOfArms) arm));

/** This skill reacts to contact to the left arm by taking the arm away. */
option(ArmContactLeftArm);

/** This skill reacts to contact to the right arm by taking the arm away. */
option(ArmContactRightArm);

/**
 * This skill takes arms back to avoid obstacles.
 * @param arm The arm for which the obstacle avoidance should be done.
 *            By default, it is both arms.
 */
option(ArmObstacleAvoidance, args((Arms::Arm)(Arms::numOfArms) arm));

/** This skill takes the left arm back to avoid obstacles. */
option(ArmObstacleAvoidanceLeftArm);

/** This skill takes the right arm back to avoid obstacles. */
option(ArmObstacleAvoidanceRightArm);
