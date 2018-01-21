/**
* @file LibInfo.h
*/

class LibInfo : public LibraryBase
{
public:
  /** Constructor for initializing all members*/
  LibInfo();

  void preProcess() override;

  void postProcess() override;
  
  bool between(float value, float min, float max);
  float clamp(float value, float min, float max);
    
  int timeSinceBallWasSeen();
  
  int howManyDef();
  
  bool isCloserToTheBall();
  
  float angleToOppGoal;
  float angleToOwnGoal;
  Vector2f KeeperDesiredPos; 
  Vector2f SupporterDesiredPos; 
  float distanceToBall;
  bool closerToTheBall;
  int nbOfDef;
};




    