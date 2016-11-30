/**
* @file LibDemo.h
*
* Manages usefull things for demos
*
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
*/

class LibDemo : public LibraryBase
{
public:
  STREAMABLE(Parameters,
  {,
    (bool) isDemoActive,
  });
  Parameters parameters;                        /**< The parameters */
  LibDemo();

  void preProcess() override;

  ENUM(DemoGameState,
  { ,
    wavingPlinking,
    normal,
  });

  DemoGameState demoGameState;
  unsigned lastSwtich;
  bool lastBumperState;

  unsigned lastWaveStart = 0;
  Arms::Arm armToWing = Arms::left;
};
