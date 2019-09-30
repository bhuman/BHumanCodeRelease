/**
 * @file OuterCornerPerceptor.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "OuterCornerPerceptor.h"

void OuterCornerPerceptor::update(OuterCorner& outerCorner)
{
  outerCorner.clear();

  if(searchForLAndPA(outerCorner)
     || searchForBigLAndT(outerCorner)
     || searchForBigLAndTL(outerCorner)
     || searchForBigLAndSmallL(outerCorner))
    outerCorner.isValid = true;
  else
    outerCorner.isValid = false;
}

bool OuterCornerPerceptor::searchForBigLAndT(OuterCorner& outerCorner) const
{
  std::vector<const FieldLineIntersections::Intersection*> useBigLIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::L && intersection.additionalType == FieldLineIntersections::Intersection::big)
      useBigLIntersections.push_back(&intersection);

  if(useBigLIntersections.empty())
    return false;

  std::vector<const FieldLineIntersections::Intersection*> useNormalTIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::T && intersection.additionalType == FieldLineIntersections::Intersection::none)
      useNormalTIntersections.push_back(&intersection);

  const float fieldDistance = theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosLeftPenaltyArea;

  for(const FieldLineIntersections::Intersection* intersectionL : useBigLIntersections)
    for(const FieldLineIntersections::Intersection* intersectionT : useNormalTIntersections)
    {
      if(std::abs((intersectionL->pos - intersectionT->pos).norm() - fieldDistance) < thresholdLTIntersections)
      {
        if(std::abs(Angle(intersectionT->dir2.angle() - intersectionL->dir1.angle()).normalize()) < thesholdAngleDisForLTIntersections
           || std::abs(Angle(intersectionT->dir2.angle() + pi - intersectionL->dir1.angle()).normalize()) < thesholdAngleDisForLTIntersections)
        {
          outerCorner.translation = intersectionL->pos;
          outerCorner.rotation = intersectionL->dir1.angle();
          outerCorner.isRightCorner = Angle(intersectionL->dir2.angle() - intersectionL->dir1.angle()).normalize() > 0;

          outerCorner.clear();
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionL->ownIndex, outerCorner.isRightCorner ? MarkedIntersection::BLR : MarkedIntersection::BLL),
              theFieldLineIntersections, theFieldLines, outerCorner);

          if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
            continue;

          return true;
        }
        else if(std::abs(Angle(intersectionT->dir2.angle() - intersectionL->dir2.angle()).normalize()) < thesholdAngleDisForLTIntersections
                || std::abs(Angle(intersectionT->dir2.angle() + pi - intersectionL->dir2.angle()).normalize()) < thesholdAngleDisForLTIntersections)
        {
          outerCorner.translation = intersectionL->pos;
          outerCorner.rotation = intersectionL->dir2.angle();
          outerCorner.isRightCorner = Angle(intersectionL->dir1.angle() - intersectionL->dir2.angle()).normalize() > 0;

          outerCorner.clear();
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionL->ownIndex, outerCorner.isRightCorner ? MarkedIntersection::BLR : MarkedIntersection::BLL),
              theFieldLineIntersections, theFieldLines, outerCorner);

          if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
            continue;

          return true;
        }
      }
    }
  return false;
}

bool OuterCornerPerceptor::searchForLAndPA(OuterCorner& outerCorner) const
{
  if(!thePenaltyArea.isValid)
    return false;

  std::vector<const FieldLineIntersections::Intersection*> useLIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::L)
      useLIntersections.push_back(&intersection);

  if(useLIntersections.empty())
    return false;

  for(const FieldLineIntersections::Intersection* lIntersection : useLIntersections)
  {
    const Vector2f intersectionInPA = thePenaltyArea.inverse() * lIntersection->pos;
    if(intersectionInPA.x() < 0.f)
      continue;

    static const float xOffsetPA = (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;
    if(intersectionInPA.y() < 0) //check for right corner
    {
      static const Vector2f rightCornerInPa = Vector2f(xOffsetPA, theFieldDimensions.yPosRightSideline);
      const Vector2f displacement = intersectionInPA - rightCornerInPa;

      if(displacement.squaredNorm() > squaredThresholdDisOffsetToPA ||
         std::abs(displacement.x()) > allowedDisplacement || std::abs(displacement.y()) > allowedDisplacement)
        continue;

      const Angle postulatedCornerAngle = thePenaltyArea.rotation + 90_deg;
      Angle rotation;
      if(std::abs((rotation = lIntersection->dir1.angle()) - postulatedCornerAngle) < thresholdPostulatedCornerAngleOffset)
      {
        if(Angle(lIntersection->dir2.angle() - rotation).normalize() < 0.f)
          continue;

        outerCorner.translation = lIntersection->pos;
        outerCorner.rotation = rotation;
        outerCorner.isRightCorner = true;

        outerCorner.clear();
        theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(lIntersection->ownIndex, MarkedIntersection::BLR),
            theFieldLineIntersections, theFieldLines, outerCorner);

        if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
          continue;

        return true;
      }
      else if(std::abs((rotation = lIntersection->dir2.angle()) - postulatedCornerAngle) < thresholdPostulatedCornerAngleOffset)
      {
        if(Angle(lIntersection->dir1.angle() - rotation).normalize() < 0.f)
          continue;

        outerCorner.translation = lIntersection->pos;
        outerCorner.rotation = rotation;
        outerCorner.isRightCorner = true;

        outerCorner.clear();
        theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(lIntersection->ownIndex, MarkedIntersection::BLR),
            theFieldLineIntersections, theFieldLines, outerCorner);

        if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
          continue;

        return true;
      }
    }
    else //check for left corner
    {
      static const Vector2f leftCornerInPa = Vector2f(xOffsetPA, theFieldDimensions.yPosLeftSideline);
      const Vector2f displacement = intersectionInPA - leftCornerInPa;

      if(displacement.squaredNorm() > squaredThresholdDisOffsetToPA ||
         std::abs(displacement.x()) > allowedDisplacement || std::abs(displacement.y()) > allowedDisplacement)
        continue;

      const float postulatedCornerAngle = thePenaltyArea.rotation - 90_deg;
      Angle rotation;
      if(std::abs((rotation = lIntersection->dir1.angle()) - postulatedCornerAngle) < thresholdPostulatedCornerAngleOffset)
      {
        if(Angle(lIntersection->dir2.angle() - rotation).normalize() > 0.f)
          continue;

        outerCorner.translation = lIntersection->pos;
        outerCorner.rotation = rotation;
        outerCorner.isRightCorner = false;

        outerCorner.clear();
        theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(lIntersection->ownIndex, MarkedIntersection::BLL),
            theFieldLineIntersections, theFieldLines, outerCorner);

        if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
          continue;

        return true;
      }
      else if(std::abs((rotation = lIntersection->dir2.angle()) - postulatedCornerAngle) < thresholdPostulatedCornerAngleOffset)
      {
        if(Angle(lIntersection->dir1.angle() - rotation).normalize() > 0.f)
          continue;

        outerCorner.translation = lIntersection->pos;
        outerCorner.rotation = rotation;
        outerCorner.isRightCorner = false;

        outerCorner.clear();
        theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(lIntersection->ownIndex, MarkedIntersection::BLL),
            theFieldLineIntersections, theFieldLines, outerCorner);

        if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
          continue;

        return true;
      }
    }
  }
  return false;
}

bool OuterCornerPerceptor::searchForBigLAndTL(OuterCorner& outerCorner) const
{
  std::vector<const FieldLineIntersections::Intersection*> useBigLIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::L && intersection.additionalType == FieldLineIntersections::Intersection::big)
      useBigLIntersections.push_back(&intersection);

  if(useBigLIntersections.empty())
    return false;

  std::vector<const FieldLineIntersections::Intersection*> useNormalLIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::L && intersection.additionalType == FieldLineIntersections::Intersection::none)
      useNormalLIntersections.push_back(&intersection);

  static const float fieldDistance = theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosLeftPenaltyArea;

  for(const FieldLineIntersections::Intersection* intersectionBigL : useBigLIntersections)
  {
    const Pose2f poseBigL(intersectionBigL->dir1.angle(), intersectionBigL->pos);
    const Pose2f inversePoseBigL = poseBigL.inverse();
    for(const FieldLineIntersections::Intersection* intersectionSmallL : useNormalLIntersections)
    {
      if(std::abs((intersectionBigL->pos - intersectionSmallL->pos).norm() - fieldDistance) < thresholdLTIntersections)
      {
        const Pose2f smallLInBigL = inversePoseBigL * intersectionSmallL->pos;
        if(std::abs(smallLInBigL.translation.x()) < allowedDisplacement && smallLInBigL.translation.y() > 0.f
           && std::abs(Angle(intersectionBigL->dir1.angle() - intersectionSmallL->dir2.angle()).normalize()) < allowedAngleDisplacement
           && std::abs(Angle(pi + intersectionBigL->dir2.angle() - intersectionSmallL->dir1.angle()).normalize()) < allowedAngleDisplacement)
        {
          outerCorner.translation = intersectionBigL->pos;
          outerCorner.rotation = intersectionBigL->dir2.angle();
          outerCorner.isRightCorner = false;

          outerCorner.clear();
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionBigL->ownIndex, MarkedIntersection::BLL),
              theFieldLineIntersections, theFieldLines, outerCorner);

          if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
            continue;

          return true;
        }
        else if(std::abs(smallLInBigL.translation.y()) < allowedDisplacement && smallLInBigL.translation.x() > 0.f
                && std::abs(Angle(pi + intersectionBigL->dir1.angle() - intersectionSmallL->dir2.angle()).normalize()) < allowedAngleDisplacement
                && std::abs(Angle(intersectionBigL->dir2.angle() - intersectionSmallL->dir1.angle()).normalize()) < allowedAngleDisplacement)
        {
          outerCorner.translation = intersectionBigL->pos;
          outerCorner.rotation = intersectionBigL->dir1.angle();
          outerCorner.isRightCorner = true;

          outerCorner.clear();
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionBigL->ownIndex, MarkedIntersection::BLR),
              theFieldLineIntersections, theFieldLines, outerCorner);

          if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
            continue;

          return true;
        }
      }
    }
  }
  return false;
}

bool OuterCornerPerceptor::searchForBigLAndSmallL(OuterCorner& outerCorner) const
{
  std::vector<const FieldLineIntersections::Intersection*> useBigLIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::L && intersection.additionalType == FieldLineIntersections::Intersection::big)
      useBigLIntersections.push_back(&intersection);

  if(useBigLIntersections.empty())
    return false;

  std::vector<const FieldLineIntersections::Intersection*> useNormalLIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::L && intersection.additionalType == FieldLineIntersections::Intersection::none)
      useNormalLIntersections.push_back(&intersection);

  static const float xFieldDistance = theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea;
  static const float fieldDistance = theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosLeftPenaltyArea + xFieldDistance;

  for(const FieldLineIntersections::Intersection* intersectionBigL : useBigLIntersections)
  {
    const Pose2f poseBigL(intersectionBigL->dir1.angle(), intersectionBigL->pos);
    const Pose2f inversePoseBigL = poseBigL.inverse();
    for(const FieldLineIntersections::Intersection* intersectionSmallL : useNormalLIntersections)
    {
      if(std::abs((intersectionBigL->pos - intersectionSmallL->pos).norm() - fieldDistance) < thresholdLLIntersections)
      {
        const Pose2f smallLInBigL = inversePoseBigL * intersectionSmallL->pos;
        if(std::abs(smallLInBigL.translation.x() - xFieldDistance) < allowedDisplacement && smallLInBigL.translation.y() > 0.f
           && std::abs(Angle(pi + intersectionBigL->dir1.angle() - intersectionSmallL->dir2.angle()).normalize()) < allowedAngleDisplacement
           && std::abs(Angle(intersectionBigL->dir2.angle() - intersectionSmallL->dir1.angle()).normalize()) < allowedAngleDisplacement)
        {
          outerCorner.translation = intersectionBigL->pos;
          outerCorner.rotation = intersectionBigL->dir2.angle();
          outerCorner.isRightCorner = false;

          outerCorner.clear();
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionBigL->ownIndex, MarkedIntersection::BLL),
              theFieldLineIntersections, theFieldLines, outerCorner);

          if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
            continue;

          return true;
        }
        else if(std::abs(smallLInBigL.translation.y() - xFieldDistance) < allowedDisplacement && smallLInBigL.translation.x() > 0.f
                && std::abs(Angle(intersectionBigL->dir1.angle() - intersectionSmallL->dir2.angle()).normalize()) < allowedAngleDisplacement
                && std::abs(Angle(pi + intersectionBigL->dir2.angle() - intersectionSmallL->dir1.angle()).normalize()) < allowedAngleDisplacement)
        {
          outerCorner.translation = intersectionBigL->pos;
          outerCorner.rotation = intersectionBigL->dir1.angle();
          outerCorner.isRightCorner = true;

          outerCorner.clear();
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionBigL->ownIndex, MarkedIntersection::BLR),
              theFieldLineIntersections, theFieldLines, outerCorner);

          if(!outerCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
            continue;

          return true;
        }
      }
    }
  }
  return false;
}

MAKE_MODULE(OuterCornerPerceptor, perception)
