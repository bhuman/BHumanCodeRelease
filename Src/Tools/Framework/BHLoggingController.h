/**
 * @file BHLoggingController.h
 *
 * This file declares the class \c BHLoggingController.
 *
 * @author Arne Hasselbring
 */

#include "Framework/Logger.h"
#include "Streaming/AutoStreamable.h"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct GameState;

class BHLoggingController : public LoggingController
{
public:
  /**
   * Constructor.
   * - Reads the file \c teamList.cfg that defines a mapping from team number to team name.
   * - Allocates a \c GameState in the \c Blackboard.
   */
  BHLoggingController();

private:
  /** A team number and the corresponding team name. */
  STREAMABLE(Team,
  {,
    (std::uint8_t)(0) number,
    (std::string) name,
  });

  /** The map from team numbers to team names. */
  STREAMABLE(TeamList,
  {,
    (std::vector<Team>) teams,
  });

  /**
   * Destructor.
   * - Frees the \c GameState in the \c Blackboard.
   */
  ~BHLoggingController() override;

  /**
   * Returns if the current game state is neither INITIAL nor FINISHED(*).
   * (*) See the implementation for an exception.
   * @param wasLogging Whether the logger was logging in the previous frame.
   * @return If the current game state is ...
   */
  bool shouldLog(bool wasLogging) const override;

  /**
   * Builds a log file description matching the current game state.
   * - If a GameController is running and the opponent team is known:
   *   <opponent team name> "_" ( "1stHalf" | "2ndHalf" | "ShootOut" )
   * - Otherwise: "Testing"
   * @return The description for the log file.
   */
  std::string getDescription() const override;

  std::unordered_map<std::uint8_t, std::string> teams; /**< The map from team numbers to team names for naming the log file after the opponent. */
  const GameState& gameState; /**< The game state in the current thread's blackboard. */
};
