/**
 * @file InternalStateProvider.h
 *
 * This file declares a module that writes the internal state
 * to a file, which is read by the DeployDialog.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/InternalState.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"

MODULE(InternalStateProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(SystemSensorData),
  USES(InternalState),
  PROVIDES(InternalState),
  DEFINES_PARAMETERS(
  {,
    (int)(-2) writerPriority, /**< The priority of the writer thread. */
    (int)(5000) writeDelay, /**< The delay between two writes in ms. */
  }),
});

class InternalStateProvider : public InternalStateProviderBase, public Thread
{
  DECLARE_SYNC;
  unsigned timeWhenLastWritten = 0; /**< The last time when the internal state was written. */
  Semaphore writeNow; /**< The semaphore is triggered when the background thread should write the data. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theInternalState The representation updated.
   */
  void update(InternalState& theInternalState) override;

  /** Waits for the semaphore and then writes the data. */
  void writer();

public:
  /** Starts the writer thread. */
  InternalStateProvider();

  /** Stops the writer thread. */
  ~InternalStateProvider();
};
