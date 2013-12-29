/**
* @file Modules/MotionControl/SpecialActions.h
* This file declares a module that creates the motions of special actions.
* @author <A href="mailto:dueffert@informatik.hu-berlin.de">Uwe Düffert</A>
* @author Martin Lötzsch
* @author Max Risler
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/MessageQueue/InMessage.h"

MODULE(SpecialActions)
  REQUIRES(FilteredJointData)
  REQUIRES(FrameInfo)
  REQUIRES(MotionSelection)
  REQUIRES(HardnessSettings)
  PROVIDES_WITH_MODIFY(SpecialActionsOutput)
END_MODULE

class SpecialActions : public SpecialActionsBase
{
private:
  /**
  * Represents a node of the motion net.
  * The motion net is organised in an array of nodes (MotionNetNode).
  */
  class MotionNetNode
  {
  public:
    ENUM(NodeType,
      typeConditionalTransition, /**< The current node is a conditional transition. */
      typeTransition, /**< The current node is a transition. */
      typeData, /**< The current node is a motor data vector. */
      typeHardness /**< The current node is a motor hardness tuple. */
    );
    float d[JointRequest::numOfJoints + 4]; /**< Represent a set of values from a data line. */
    /*
    //possible content:
    {typeData, d[0]..d[21], interpolationMode, dataRepetitionCounter, execMotionRequest}
    {typeConditionalTransition, to_motion, via_label, 17*0, execMotionRequest}
    {typeTransition, to_label, 18*0, execMotionRequest}
    */

    void toJointRequest(JointRequest& jointRequest, int& dataRepetitionCounter, bool& interpolationMode, bool& deShakeMode) const
    {
      for(int i = 0; i < JointRequest::numOfJoints; ++i)
        jointRequest.angles[i] = (float) d[i + 1];
      interpolationMode = (int(d[JointRequest::numOfJoints + 1]) & 1) != 0;
      deShakeMode = (int(d[JointRequest::numOfJoints + 1]) & 2) != 0;
      dataRepetitionCounter = int(d[JointRequest::numOfJoints + 2]);
    }

    void toHardnessRequest(HardnessData& hardnessRequest, int& hardnessInterpolationTime)
    {
      for(int i = 0; i < JointRequest::numOfJoints; i++)
        hardnessRequest.hardness[i] = (int) d[i + 1];
      hardnessInterpolationTime = int(d[JointRequest::numOfJoints + 1]);
    }

    SpecialActionRequest::SpecialActionID getSpecialActionID() const
    {
      return SpecialActionRequest::SpecialActionID(short(d[JointRequest::numOfJoints + 3]));
    }
  };

  /**
  * MotionNetData encapsulates all the motion data in the motion net.
  */
  class MotionNetData
  {
  public:
    /** Default constructor. */
    MotionNetData() : nodeArray(0) {}

    /** Destructor. */
    ~MotionNetData() {if(nodeArray) delete[] nodeArray;}

    /** Loads the motion net from a file or another stream. */
    void load(In& stream);

    /** jump table from extern.mof: get start index from request */
    short label_extern_start[SpecialActionRequest::numOfSpecialActionIDs + 1];

    /** The motion net */
    MotionNetNode* nodeArray;
  };

  /**
  * Odometry table entry.
  */
  class SpecialActionInfo: public Streamable
  {
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(id, SpecialActionRequest);
      STREAM(type);
      if(type != none)
        STREAM(odometryOffset);
      STREAM(isMotionStable);
      STREAM_REGISTER_FINISH;
    }

  public:
    /**
    * Enum for odometry types
    */
    ENUM(OdometryType,
      none, /**< No odometry, means no movement. */
      once, /**< Odometry pose is used once the motion is executed. */
      homogeneous /**< Odometry pose describes speed and is used each tick. */
    );

    SpecialActionRequest::SpecialActionID id; /**< The id to which belongs this SpecialActionInfo. */
    OdometryType type; /**< The type of this odometry entry. */
    Pose2D odometryOffset; /**< The displacement performed by the special action. */
    bool isMotionStable; /**< Is the position of the camera directly related to the kinematic chain of joint angles? */

    /**
    * Default constructor.
    */
    SpecialActionInfo() : type(none), isMotionStable(false) {}
  };

  STREAMABLE(OdometryParams,
  {,
    (std::vector<SpecialActionInfo>) specialActionInfos,
  });

  HardnessData currentHardnessRequest,/**< The current hardness of the joints */
               lastHardnessRequest;/**< The last hardness data*/
  bool wasEndOfSpecialAction; /**< Was the SpecialAction at the end in the last frame? */
  int hardnessInterpolationCounter, /**< Cycle counter for current hardness interpolation */
      hardnessInterpolationLength; /**< Length of the current hardness interpolation */

  static PROCESS_WIDE_STORAGE(SpecialActions) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool wasActive; /**< Was this module active in the previous frame? */
  MotionNetData motionNetData; /**< The motion data array. */
  short currentNode; /**< Current motion net node */
  JointRequest currentRequest, /**< Current joint data. */
               lastRequest; /**< Last data for interpolation. */
  bool interpolationMode, /**< True if values should be interpolated. */
       deShakeMode; /**< True if shaking of arms should be prevented. */
  int dataRepetitionLength, /**< Length of current data line in cycles. */
      dataRepetitionCounter; /**< Cycle counter for current data line. */
  SpecialActionInfo infoTable[SpecialActionRequest::numOfSpecialActionIDs + 1], /**< Odometry offset table. */
                    currentInfo; /**< Information about the special action currently executed. */
  SpecialActionRequest::SpecialActionID lastSpecialAction; /**< type of last executed special action. */
  bool mirror; /**< Mirror current special actions? */

  /**
  * Called from a MessageQueue to distribute messages.
  * @param message The message that can be read.
  * @return True if the message was handled.
  */
  virtual bool handleMessage2(InMessage& message);

  /** Get next motion node from motion net */
  bool getNextData(const SpecialActionRequest& specialActionRequest, SpecialActionsOutput& specialActionsOutput);

  /** Calculates the next joint data vector by interpolating if necessary */
  void calculateJointRequest(JointRequest& jointRequest);

  void update(SpecialActionsOutput& specialActionsOutput);

public:
  /*
  * Default constructor.
  */
  SpecialActions();

  /*
  * Destructor.
  */
  ~SpecialActions() {theInstance = 0;}

  /**
  * The method is called for every incoming debug message.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  static bool handleMessage(InMessage& message);
};
