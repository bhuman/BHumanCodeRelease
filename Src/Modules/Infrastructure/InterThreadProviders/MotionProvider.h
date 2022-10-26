/**
 * @file MotionProvider.h
 *
 * This file declares a module that provides representations from the motion
 * thread for the current frame.
 *
 * @author Arne Hasselbring
 */

#include "Framework/Module.h"
#include "Representations/MotionControl/OdometryData.h"

MODULE(MotionProvider,
{,
  REQUIRES(MotionOdometryData),
  PROVIDES(OdometryData),
});

class MotionProvider : public MotionProviderBase
{
  void update(OdometryData& odometryData);
};
