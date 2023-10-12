/**
 * @file BHExecutionUnit.h
 *
 * This file declares the class \c BHExecutionUnit.
 *
 * @author Arne Hasselbring
 */

#include "Representations/Infrastructure/GameState.h"
#include "Framework/FrameExecutionUnit.h"
#include <string>

class LoggingController;

class BHExecutionUnit : public FrameExecutionUnit
{
protected:
  /**
   * This method creates an annotation if the representation \c GameState is present in this thread and the state changed.
   */
  void beforeModules() override;

private:
  /**
   * This function determines if this thread is the first thread in the config which provides the representation \c GameState.
   * In that case, this thread should control the logger and an instance of \c BHLoggingController is returned.
   * @param config The initial configuration of all threads.
   * @param index The index of this thread in the config.
   * @return A \c BHLoggingController if this thread should control the logger.
   */
  const LoggingController* initLogging(const Configuration& config, const std::size_t index) override;

  GameState::State lastGameState = GameState::beforeHalf; /**< Game state in the last frame (for annotation). */
};
