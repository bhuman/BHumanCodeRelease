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
  
  void countRoles();
  
  bool isCloserToTheBall();

  float xPos;
  float yPos;
  Vector2f midPointBallGoal;
  float angleToOppGoal;
  float angleToOwnGoal;
  Vector2f DesiredPos;
  double distanceToBall;
  bool closerToTheBall;
  int nbOfDefender;
  int nbOfKeeper;
  int nbOfStriker;
  int nbOfSupporter;
};




    