/**
* @file BIKEParameters.cpp
* @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
*/

#include "BIKEParameters.h"
#include "Representations/MotionControl/BikeRequest.h"
#include "Tools/Debugging/Modify.h"

void Phase::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;

  STREAM(duration);

  Vector3<>& leftFootTra1(controlPoints[leftFootTra][1]);
  Vector3<>& leftFootTra2(controlPoints[leftFootTra][2]);
  Vector3<>& leftFootRot1(controlPoints[leftFootRot][1]);
  Vector3<>& leftFootRot2(controlPoints[leftFootRot][2]);
  Vector3<>& rightFootTra1(controlPoints[rightFootTra][1]);
  Vector3<>& rightFootTra2(controlPoints[rightFootTra][2]);
  Vector3<>& rightFootRot1(controlPoints[rightFootRot][1]);
  Vector3<>& rightFootRot2(controlPoints[rightFootRot][2]);
  Vector3<>& leftArmTra1(controlPoints[leftArmTra][1]);
  Vector3<>& leftArmTra2(controlPoints[leftArmTra][2]);
  Vector3<>& leftHandRot1(controlPoints[leftHandRot][1]);
  Vector3<>& leftHandRot2(controlPoints[leftHandRot][2]);
  Vector3<>& rightArmTra1(controlPoints[rightArmTra][1]);
  Vector3<>& rightArmTra2(controlPoints[rightArmTra][2]);
  Vector3<>& rightHandRot1(controlPoints[rightHandRot][1]);
  Vector3<>& rightHandRot2(controlPoints[rightHandRot][2]);
  Vector2<>& comTra1(comTra[1]);
  Vector2<>& comTra2(comTra[2]);
  Vector2<>& headTra1(headTra[1]);
  Vector2<>& headTra2(headTra[2]);

  STREAM(leftFootTra1)
  STREAM(leftFootTra2)
  STREAM(leftFootRot1)
  STREAM(leftFootRot2)
  STREAM(rightFootTra1)
  STREAM(rightFootTra2)
  STREAM(rightFootRot1)
  STREAM(rightFootRot2)
  STREAM(leftArmTra1)
  STREAM(leftArmTra2)
  STREAM(leftHandRot1)
  STREAM(leftHandRot2)
  STREAM(rightArmTra1)
  STREAM(rightArmTra2)
  STREAM(rightHandRot1)
  STREAM(rightHandRot2)
  STREAM(comTra1)
  STREAM(comTra2)
  STREAM(headTra1)
  STREAM(headTra2)

  STREAM(odometryOffset)

  STREAM_REGISTER_FINISH;
}


void BIKEParameters::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;

  STREAM(footOrigin)
  STREAM(footRotOrigin)
  STREAM(armOrigin)
  STREAM(handRotOrigin)
  STREAM(comOrigin)
  STREAM(headOrigin)

  STREAM(kpx)
  STREAM(kix)
  STREAM(kdx)

  STREAM(kpy)
  STREAM(kiy)
  STREAM(kdy)

  STREAM(preview)

  STREAM(loop)

  STREAM(autoComTra)

  STREAM(ignoreHead)

  if(in)
  {
    phaseParameters.clear();
  }
  STREAM(phaseParameters);
  numberOfPhases = phaseParameters.size();

  if(in)
  {
    calcControlPoints();
  }

  STREAM_REGISTER_FINISH;
}


void BIKEParameters::calcControlPoints()
{
  for(int phaseNumber = 0; phaseNumber < numberOfPhases - 1; phaseNumber++)
  {
    float factor = (float)phaseParameters[phaseNumber].duration /
                   (float)phaseParameters[phaseNumber + 1].duration;

    phaseParameters[phaseNumber + 1].comTra[0] =
      phaseParameters[phaseNumber].comTra[2] -
      phaseParameters[phaseNumber].comTra[1];

    phaseParameters[phaseNumber + 1].comTra[0] *= factor;

    phaseParameters[phaseNumber + 1].comTra[0] +=
      phaseParameters[phaseNumber].comTra[2];


	    phaseParameters[phaseNumber + 1].headTra[0] =
      phaseParameters[phaseNumber].headTra[2] -
      phaseParameters[phaseNumber].headTra[1];

    phaseParameters[phaseNumber + 1].headTra[0] *= factor;

    phaseParameters[phaseNumber + 1].headTra[0] +=
      phaseParameters[phaseNumber].headTra[2];

    for(int limb = 0; limb < Phase::numOfLimbs; limb++)
    {
      phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
        phaseParameters[phaseNumber].controlPoints[limb][2] -
        phaseParameters[phaseNumber].controlPoints[limb][1];

      phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

      phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
        phaseParameters[phaseNumber].controlPoints[limb][2];
    }
  }
}

Vector3<> BIKEParameters::getPositionBlame(const float& phase, const int& phaseNumber, const int& limb)
{
  Vector3<> p0, p1;
  if(phaseNumber == 0)
  {
    p0 = phaseParameters[phaseNumber].originPos[limb];
    p1 = p0;
  }
  else
  {
    p0 = phaseParameters[phaseNumber - 1].controlPoints[limb][2];
    p1 = phaseParameters[phaseNumber].controlPoints[limb][0];
  }

  Vector3<> p2 = phaseParameters[phaseNumber].controlPoints[limb][1];
  Vector3<> p3 = phaseParameters[phaseNumber].controlPoints[limb][2];

  return (-p0 + p1 * 3 - p2 * 3 + p3) * phase * phase * phase + (p0 * 3 - p1 * 6 + p2 * 3) * phase * phase + (p0 * -3 + p1 * 3) * phase + p0;
}



Vector2<>BIKEParameters::getComRefPositionBlame(const float& phase, const int& phaseNumber)
{
  Vector2<> p0, p1;
  if(phaseNumber == 0)
  {
    p0 = phaseParameters[phaseNumber].comOriginPos;
    p1 = p0;
  }
  else
  {
    p0 = phaseParameters[phaseNumber - 1].comTra[2];
    p1 = phaseParameters[phaseNumber].comTra[0];
  }

  Vector2<> p2 = phaseParameters[phaseNumber].comTra[1];
  Vector2<> p3 = phaseParameters[phaseNumber].comTra[2];

  return (-p0 + p1 * 3 - p2 * 3 + p3) * phase * phase * phase + (p0 * 3 - p1 * 6 + p2 * 3) * phase * phase + (p0 * -3 + p1 * 3) * phase + p0;
}

Vector2<>BIKEParameters::getHeadRefPositionBlame(const float& phase, const int& phaseNumber)
{
  Vector2<> p0, p1;
  if(phaseNumber == 0)
  {
    p0 = phaseParameters[phaseNumber].headOrigin;
    p1 = p0;
  }
  else
  {
    p0 = phaseParameters[phaseNumber - 1].headTra[2];
    p1 = phaseParameters[phaseNumber].headTra[0];
  }

  Vector2<> p2 = phaseParameters[phaseNumber].headTra[1];
  Vector2<> p3 = phaseParameters[phaseNumber].headTra[2];

  return (-p0 + p1 * 3 - p2 * 3 + p3) * phase * phase * phase + (p0 * 3 - p1 * 6 + p2 * 3) * phase * phase + (p0 * -3 + p1 * 3) * phase + p0;
}

void BIKEParameters::initFirstPhase()
{

  //this function is only called by bikeView
  if(numberOfPhases > 0)
  {
    phaseParameters[0].originPos[Phase::leftFootTra] = footOrigin;
    phaseParameters[0].originPos[Phase::rightFootTra] = Vector3<>(footOrigin.x, -footOrigin.y, footOrigin.z);

    phaseParameters[0].originPos[Phase::leftFootRot] = footRotOrigin;
    phaseParameters[0].originPos[Phase::rightFootRot] = Vector3<>(-footRotOrigin.x, footRotOrigin.y, -footRotOrigin.z);

    phaseParameters[0].originPos[Phase::leftArmTra] = armOrigin;
    phaseParameters[0].originPos[Phase::rightArmTra] = Vector3<>(armOrigin.x, -armOrigin.y, armOrigin.z);

    phaseParameters[0].originPos[Phase::leftHandRot] = handRotOrigin;
    phaseParameters[0].originPos[Phase::rightHandRot] = Vector3<>(-handRotOrigin.x, handRotOrigin.y, -handRotOrigin.z);

    //set the Offset for the first Phase to zero, because all calculations based on the startOrigin

    phaseParameters[0].comOriginPos = comOrigin;
    phaseParameters[0].comOriginOffset = Vector2<>(0.f, 0.f);

	phaseParameters[0].headOrigin = headOrigin;

    phaseParameters[0].controlPoints[Phase::leftFootTra][0] = footOrigin;
    phaseParameters[0].controlPoints[Phase::rightFootTra][0] = Vector3<>(footOrigin.x, -footOrigin.y, footOrigin.z);

    phaseParameters[0].controlPoints[Phase::leftFootRot][0] = footRotOrigin;
    phaseParameters[0].controlPoints[Phase::rightFootRot][0] = Vector3<>(-footRotOrigin.x, footRotOrigin.y, -footRotOrigin.z);

    phaseParameters[0].controlPoints[Phase::leftArmTra][0] = armOrigin;
    phaseParameters[0].controlPoints[Phase::rightArmTra][0] = Vector3<>(armOrigin.x, -armOrigin.y, armOrigin.z);

    phaseParameters[0].controlPoints[Phase::leftHandRot][0] = handRotOrigin;
    phaseParameters[0].controlPoints[Phase::rightHandRot][0] = Vector3<>(-handRotOrigin.x, handRotOrigin.y, -handRotOrigin.z);

    phaseParameters[0].comTra[0] = comOrigin;

  }
}


void BIKEParameters::initFirstPhase(Vector3<> *origins, Vector2<> head)
{

  for(int i = 0; i < Phase::numOfLimbs; ++i)
  {
    phaseParameters[0].originPos[i] = origins[i];
  }
  phaseParameters[0].comOriginPos = Vector2<>(0.f, 0.f);
  phaseParameters[0].comOriginOffset = Vector2<>(0.f, 0.f);
  phaseParameters[0].headOrigin = head;
}

