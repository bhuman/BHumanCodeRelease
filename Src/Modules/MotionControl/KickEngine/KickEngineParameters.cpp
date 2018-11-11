/**
 * @file KickEngineParameters.cpp
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include "KickEngineParameters.h"

void Phase::serialize(In* in, Out* out)
{
  STREAM(duration);

  Vector3f& leftFootTra1(controlPoints[leftFootTra][1]);
  Vector3f& leftFootTra2(controlPoints[leftFootTra][2]);
  Vector3f& leftFootRot1(controlPoints[leftFootRot][1]);
  Vector3f& leftFootRot2(controlPoints[leftFootRot][2]);
  Vector3f& rightFootTra1(controlPoints[rightFootTra][1]);
  Vector3f& rightFootTra2(controlPoints[rightFootTra][2]);
  Vector3f& rightFootRot1(controlPoints[rightFootRot][1]);
  Vector3f& rightFootRot2(controlPoints[rightFootRot][2]);
  Vector3f& leftArmTra1(controlPoints[leftArmTra][1]);
  Vector3f& leftArmTra2(controlPoints[leftArmTra][2]);
  Vector3f& leftHandRot1(controlPoints[leftHandRot][1]);
  Vector3f& leftHandRot2(controlPoints[leftHandRot][2]);
  Vector3f& rightArmTra1(controlPoints[rightArmTra][1]);
  Vector3f& rightArmTra2(controlPoints[rightArmTra][2]);
  Vector3f& rightHandRot1(controlPoints[rightHandRot][1]);
  Vector3f& rightHandRot2(controlPoints[rightHandRot][2]);
  Vector2f& comTra1(comTra[1]);
  Vector2f& comTra2(comTra[2]);
  Vector2f& headTra1(headTra[1]);
  Vector2f& headTra2(headTra[2]);

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
}

void Phase::reg()
{
  PUBLISH(reg);
  REG_CLASS(Phase);
  REG(duration);
  REG(Vector3f, leftFootTra1);
  REG(Vector3f, leftFootTra2);
  REG(Vector3f, leftFootRot1);
  REG(Vector3f, leftFootRot2);
  REG(Vector3f, rightFootTra1);
  REG(Vector3f, rightFootTra2);
  REG(Vector3f, rightFootRot1);
  REG(Vector3f, rightFootRot2);
  REG(Vector3f, leftArmTra1);
  REG(Vector3f, leftArmTra2);
  REG(Vector3f, leftHandRot1);
  REG(Vector3f, leftHandRot2);
  REG(Vector3f, rightArmTra1);
  REG(Vector3f, rightArmTra2);
  REG(Vector3f, rightHandRot1);
  REG(Vector3f, rightHandRot2);
  REG(Vector2f, comTra1);
  REG(Vector2f, comTra2);
  REG(Vector2f, headTra1);
  REG(Vector2f, headTra2);
  REG(odometryOffset);
}

void KickEngineParameters::onRead()
{
  numberOfPhases = static_cast<int>(phaseParameters.size());
  calcControlPoints();
}

void KickEngineParameters::calcControlPoints()
{
  for(int phaseNumber = 0; phaseNumber < numberOfPhases - 1; phaseNumber++)
  {
    float factor = static_cast<float>(phaseParameters[phaseNumber].duration) /
                   static_cast<float>(phaseParameters[phaseNumber + 1].duration);

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

Vector3f KickEngineParameters::getPosition(const float& phase, const int& phaseNumber, const int& limb)
{
  Vector3f p0, p1;
  if(phaseNumber == 0)
    p0 = p1 = phaseParameters[phaseNumber].originPos[limb];
  else
  {
    p0 = phaseParameters[phaseNumber - 1].controlPoints[limb][2];
    p1 = phaseParameters[phaseNumber].controlPoints[limb][0];
  }

  const Vector3f p2 = phaseParameters[phaseNumber].controlPoints[limb][1];
  const Vector3f p3 = phaseParameters[phaseNumber].controlPoints[limb][2];

  return (-p0 + p1 * 3 - p2 * 3 + p3) * phase * phase * phase + (p0 * 3 - p1 * 6 + p2 * 3) * phase * phase + (p0 * -3 + p1 * 3) * phase + p0;
}

Vector2f KickEngineParameters::getComRefPosition(const float& phase, const int& phaseNumber)
{
  Vector2f p0, p1;
  if(phaseNumber == 0)
    p0 = p1 = phaseParameters[phaseNumber].comOriginPos;
  else
  {
    p0 = phaseParameters[phaseNumber - 1].comTra[2];
    p1 = phaseParameters[phaseNumber].comTra[0];
  }

  const Vector2f p2 = phaseParameters[phaseNumber].comTra[1];
  const Vector2f p3 = phaseParameters[phaseNumber].comTra[2];

  //bezier
  return (-p0 + p1 * 3 - p2 * 3 + p3) * phase * phase * phase + (p0 * 3 - p1 * 6 + p2 * 3) * phase * phase + (p0 * -3 + p1 * 3) * phase + p0;
}

Vector2f KickEngineParameters::getHeadRefPosition(const float& phase, const int& phaseNumber)
{
  Vector2f p0, p1;
  if(phaseNumber == 0)
  {
    p0 = p1 = phaseParameters[phaseNumber].headOrigin;
  }
  else
  {
    p0 = phaseParameters[phaseNumber - 1].headTra[2];
    p1 = phaseParameters[phaseNumber].headTra[0];
  }

  const Vector2f p2 = phaseParameters[phaseNumber].headTra[1];
  const Vector2f p3 = phaseParameters[phaseNumber].headTra[2];

  return (-p0 + p1 * 3 - p2 * 3 + p3) * phase * phase * phase + (p0 * 3 - p1 * 6 + p2 * 3) * phase * phase + (p0 * -3 + p1 * 3) * phase + p0;
}

void KickEngineParameters::initFirstPhase()
{
  //this function is only called by kickView
  if(numberOfPhases > 0)
  {
    phaseParameters[0].originPos[Phase::leftFootTra] = footOrigin;
    phaseParameters[0].originPos[Phase::rightFootTra] = Vector3f(footOrigin.x(), -footOrigin.y(), footOrigin.z());

    phaseParameters[0].originPos[Phase::leftFootRot] = footRotOrigin;
    phaseParameters[0].originPos[Phase::rightFootRot] = Vector3f(-footRotOrigin.x(), footRotOrigin.y(), -footRotOrigin.z());

    phaseParameters[0].originPos[Phase::leftArmTra] = armOrigin;
    phaseParameters[0].originPos[Phase::rightArmTra] = Vector3f(armOrigin.x(), -armOrigin.y(), armOrigin.z());

    phaseParameters[0].originPos[Phase::leftHandRot] = handRotOrigin;
    phaseParameters[0].originPos[Phase::rightHandRot] = Vector3f(-handRotOrigin.x(), handRotOrigin.y(), -handRotOrigin.z());

    //set the Offset for the first Phase to zero, because all calculations based on the startOrigin

    phaseParameters[0].comOriginPos = comOrigin;
    phaseParameters[0].comOriginOffset = Vector2f::Zero();

    phaseParameters[0].headOrigin = headOrigin;

    phaseParameters[0].controlPoints[Phase::leftFootTra][0] = footOrigin;
    phaseParameters[0].controlPoints[Phase::rightFootTra][0] = Vector3f(footOrigin.x(), -footOrigin.y(), footOrigin.z());

    phaseParameters[0].controlPoints[Phase::leftFootRot][0] = footRotOrigin;
    phaseParameters[0].controlPoints[Phase::rightFootRot][0] = Vector3f(-footRotOrigin.x(), footRotOrigin.y(), -footRotOrigin.z());

    phaseParameters[0].controlPoints[Phase::leftArmTra][0] = armOrigin;
    phaseParameters[0].controlPoints[Phase::rightArmTra][0] = Vector3f(armOrigin.x(), -armOrigin.y(), armOrigin.z());

    phaseParameters[0].controlPoints[Phase::leftHandRot][0] = handRotOrigin;
    phaseParameters[0].controlPoints[Phase::rightHandRot][0] = Vector3f(-handRotOrigin.x(), handRotOrigin.y(), -handRotOrigin.z());

    phaseParameters[0].comTra[0] = comOrigin;
  }
}

void KickEngineParameters::initFirstPhase(const Vector3f* origins, const Vector2f& head)
{
  for(int i = 0; i < Phase::numOfLimbs; ++i)
    phaseParameters[0].originPos[i] = origins[i];
  phaseParameters[0].comOriginPos = Vector2f::Zero();
  phaseParameters[0].comOriginOffset = Vector2f::Zero();
  phaseParameters[0].headOrigin = head;
}

void KickEngineParameters::initFirstPhaseLoop(const Vector3f* origins, const Vector2f& lastCom, const Vector2f& head)
{
  for(int i = 0; i < Phase::numOfLimbs; ++i)
    phaseParameters[0].originPos[i] = origins[i];
  phaseParameters[0].comOriginPos = lastCom;
  phaseParameters[0].comOriginOffset = Vector2f::Zero();
  phaseParameters[0].headOrigin = head;
}
