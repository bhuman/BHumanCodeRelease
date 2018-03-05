/**
* @file LibTactic.h
*/

class LibTactic : public LibraryBase
{
public:
  /** Constructor for initializing all members*/
  LibTactic();

  void preProcess() override;

  void postProcess() override;
  
  bool between(float value, float min, float max);
  float clamp(float value, float min, float max);
    
  int timeSinceBallWasSeen();
  
  int howManyDef();
  
  bool isCloserToTheBall();
  
  float angleToOppGoal;
  float angleToOwnGoal;
  Vector2f keeperDesiredPos; 
  Vector2f supporterDesiredPos; 
  double distanceToBall;
  bool closerToTheBall;
  int nbOfDef;
};




    