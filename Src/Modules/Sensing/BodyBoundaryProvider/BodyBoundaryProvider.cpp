#include "BodyBoundaryProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MAKE_MODULE(BodyBoundaryProvider, sensing)

void BodyBoundaryProvider::update(BodyBoundary& bodyBoundary)
{
  if(first || parametersChanging)
  {
    bbp.initBodyBoundary(bodyBoundary);
    first = false;
  }

  bodyBoundary.update(theRobotModel.limbs.data());

  Geometry::LineSegment3D notUsed;
  if(debugMode)
    for(int i = 0; i < 8; i++)
    {
      bodyBoundary.fAL[i] = bodyBoundary.objects[bodyBoundary.foreArmLeft]->isIntersectingWith(*bodyBoundary.objects[i + 4], notUsed);
      bodyBoundary.fAR[i] = bodyBoundary.objects[bodyBoundary.foreArmRight]->isIntersectingWith(*bodyBoundary.objects[i + 4], notUsed);
      bodyBoundary.hL[i] = bodyBoundary.objects[bodyBoundary.handLeft]->isIntersectingWith(*bodyBoundary.objects[i + 4], notUsed);
      bodyBoundary.hR[i] = bodyBoundary.objects[bodyBoundary.handRight]->isIntersectingWith(*bodyBoundary.objects[i + 4], notUsed);
    }
}
