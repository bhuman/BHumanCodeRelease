/**
 * @file LibGameProvider.h
 *
 * Collection of functions that provide information about the course of the game states.
 *
 * @author Tim Laue
 * @author Andreas Stolpmann
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibGame.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"

MODULE(LibGameProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(CognitionStateChanges),
  REQUIRES(GameInfo),
  REQUIRES(FrameInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotInfo),
  PROVIDES(LibGame),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(15000) timeBetweenWhistleAndGCPlaying,
  }),
});

class LibGameProvider : public LibGameProviderBase
{
private:
  unsigned char gameStateLastFrame = STATE_INITIAL;  /*< The game state that was active in the last frame */
  unsigned char previousGameState = STATE_INITIAL;  /*< The game state that was active prior to the current game state */
  unsigned timeWhenLastReadyStarted = 0;
  unsigned timeWhenLastSetStarted = 0;
  unsigned timeWhenLastPlayingStarted = 0;
  unsigned timeWhenLastPenaltyEnded = 0;
  unsigned timeWhenBallWentOut = 0;
  unsigned char rawGameStateLastFrame;  /*< The raw game state that was active in the last frame */
  unsigned short dropInTimeLastFrame;   /*< The time since the last ball drop in in the last frame */
  short lastRawSecondsRemaining;

public:
  void update(LibGame& libGame);
};