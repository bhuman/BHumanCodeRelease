/**
 * @file RobotTrackerUtils.h
 *
 * This file declares some utility classes to represent tracked robots
 *
 * TODO: make it to STREAMABLEs so it can part of the Team communication
 *
 * @author Yannik Meinken
 */

#pragma once

#include "Math/UnscentedKalmanFilter.h"
#include "Math/Covariance.h"
#include "Debugging/ColorRGBA.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/TeamData.h"

namespace GlobalRobotTracker
{
  // Todo: find a better name
  class Estimate
  {
    UKF<4> kf;              // filter for position and velocity

    bool isPercept;         // only differentiate here between percepts and estimates that are modeled over time allows us to threat own percepts and estimates send by out teammates the same way further up

    bool wasSeen;           // was there a new measurement in the last frame

    Vector4f measurementModelPercept(const Vector4f&); // just one percept contains no information about velocity so it should not be incorporated.

    Vector4f measurementModel(const Vector4f&);

  public:
    Estimate(Vector2f& position, Matrix2f& positionCovariance);

    // not necessarily euclidean, can be used to map measurements to robots
    float getDistance(const Estimate& other) const
    {
      return (getPosition() - other.getPosition()).norm();
    }

    Vector2f getPosition() const
    {
      return kf.mean.head(2);
    }

    Vector2f getVelocity() const
    {
      return kf.mean.tail(2);
    }

    Vector2f getCovariance() const
    {
      return kf.cov.topLeftCorner(2, 2);
    }

    /**
     * threats the given Estimate as a percept to merge it with this one.
     * after calling this method isPercept should be false as it now contains data from more than one percept.
     * wasSeen can here be set to true since we had a measurement
     * @param z new measurement
     */
    void measurementUpdate(const Estimate& z);

    /**
     * calls the predict method of the UKF
     * resets wasSeen
     */
    void predict(const std::function<void(Vector4f&)>& dynamicModel, const Matrix4f& noise);

    // can in combination with the expected observations and the game controller information be used to determine if this robot is penalized
    bool hasNewMeasurement() const
    {
      return wasSeen;
    }

    /** Draws the opponents with the given color
     * @param color The color
     */
    void draw(const ColorRGBA& color) const;
  };

  class Robot
  {
  protected:
    std::vector<Estimate> estimates;    // list of possible states of this robot
    float penalizedProbability;         // how likely is it that this robot is penalized?

  public:
    Robot(std::vector<Estimate> estimates);

    /**
     * disposes no longer valid estimates
     * @return returns true if after the cleanup no valid estimate corresponds to this robot
     */
    bool cleanup();

    std::vector<Estimate>& getEstimations()
    {
      return estimates;
    }

    /**
     * calls the predict function of the estimates
     * maybe also updates the penalizedProbability or calls a method to do so
     */
    void update();

    // Maybe only for Teammate to not awake utopic expectations
    Vector2f getFuturePosition(unsigned time) const;

    /** Draws the opponents with the given color
     * @param color The color
     */
    void draw(const ColorRGBA& color) const;
  };

  class Teammate : Robot
  {
    unsigned number;                  // jersey number of the Teammate. Zero if not known
    Vector2f goalPosition;             // position to which the Teammate wants to go according to his last communicated message
    unsigned lastCommunication;       // time when the last message from this Teammate was received. Zero if no new message was received since the Teammate returned from a penalty/the game started.

  public:
    // for seeing a new Teammate not known before or one returning from penalty
    Teammate(std::vector<Estimate> estimates, unsigned num = 0);

    // for a new teammate based on a communicated message of this
    Teammate(Estimate estimate, unsigned num, Vector2f goal, unsigned time);

    /**
     * merge the communicated estimation of the Teammate as well as update the goal position and set the Number if unknown before.
     */
    void includeCommunication(const Teammate& other);

    /**
     * overrides the method of Robot to include acceleration towards the communicated end position
     * also include dampening/friction to stay inside the maximal velocity
     * Todo: think about how to address overshot/oscillations
     *   hack: massive dampening near the goal position
     */
    void update();

    /**
     * switches the numbers of the Teammates
     * for example if we confused two teammates walking past each other and now one of them communicated his position.
     */
    void switchNumber(Teammate& other);

    //get the number
    unsigned getNumber() const
    {
      return number;
    }

    /**
     * returns the estimated position of this Teammate at the given time
     */
    Vector2f getFuturePosition(unsigned time) const;

    /** Draws the opponents with the given color
     * @param color The color
     */
    void draw(const ColorRGBA& color) const;
  };
}
