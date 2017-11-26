/**
* @file LibCodeRelease.h
*/

class LibCodeRelease : public LibraryBase
{
public:
  /** Constructor for initializing all members*/
  LibCodeRelease();

  void preProcess() override;

  void postProcess() override;
  
  
  bool between(float value, float min, float max);
  float clamp(float value, float min, float max);
    
  int timeSinceBallWasSeen();
  
 
  float angleToOppGoal;
  float angleToOwnGoal;
  Vector2f KeeperDesiredPos; 
};