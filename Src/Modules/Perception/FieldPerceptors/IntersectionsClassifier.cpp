/**
 * @file IntersectionsClassifier.cpp
 *
 * This file implements a module that ...
 *
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 */

#include "IntersectionsClassifier.h"
#include "Platform/File.h"

MAKE_MODULE(IntersectionsClassifier);

IntersectionsClassifier::IntersectionsClassifier() : network(&Global::getAsmjitRuntime())
{
  // Initialize model for the neural net
  model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir()) + "/Config/NeuralNets/IntersectionsClassifier/distanceUpdatedModel.h5");
  network.compile(*model);
}

void IntersectionsClassifier::update(IntersectionsPercept& theIntersectionsPercept)
{
  // check whether network has been successfully compiled
  if(!network.valid())
    return;

  DECLARE_DEBUG_DRAWING("module:IntersectionsClassifier:field", "drawingOnField");
  theIntersectionsPercept.intersections.clear();

  for(IntersectionCandidates::IntersectionCandidate intersection : theIntersectionCandidates.intersections)
  {
    if(!classifyIntersection(intersection))
      continue;

    // change attributes dependent on intersection type
    switch(intersection.type)
    {
      case IntersectionsPercept::Intersection::L:
        intersection.dir1 = intersection.line1FurtherEnd - intersection.absPos;
        intersection.dir2 = intersection.line2FurtherEnd - intersection.absPos;
        break;
      case IntersectionsPercept::Intersection::X:
        break;
      case IntersectionsPercept::Intersection::T:
        // The 4 was a 2 before the RoboCup German Open 2018.
        const bool lineIsEnd = (intersection.line1CloserEnd - intersection.absPos).squaredNorm() < sqr(4 * theFieldDimensions.fieldLinesWidth);
        const bool line2IsEnd = (intersection.line2CloserEnd - intersection.absPos).squaredNorm() < sqr(4 * theFieldDimensions.fieldLinesWidth);

        // which line ends in intersection
        if(lineIsEnd && !line2IsEnd) //line is vertical
        {
          Vector2f vertical = intersection.line1FurtherEnd - intersection.absPos;
          Vector2f horizontal = intersection.line2FurtherEnd - intersection.absPos;
          enforceTIntersectionDirections(vertical, horizontal);
          intersection.dir1 = vertical;
          intersection.dir2 = horizontal;
        }
        else //line is horizontal
        {
          Vector2f vertical = intersection.line2FurtherEnd - intersection.absPos;
          Vector2f horizontal = intersection.line1FurtherEnd - intersection.absPos;
          enforceTIntersectionDirections(vertical, horizontal);
          intersection.dir1 = vertical;
          intersection.dir2 = horizontal;

          const unsigned new2Index = intersection.line1Index;
          intersection.line1Index = intersection.line2Index;
          intersection.line2Index = new2Index;
        }
    }
    addIntersection(theIntersectionsPercept, intersection);
  }
}

void IntersectionsClassifier::enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const
{
  Vector2f vertical90 = vertical;
  vertical90.rotate(pi_2); //vertical rotated by +90°
  if(vertical90.angleTo(horizontal) > pi_2)
    horizontal.mirror();
}

void IntersectionsClassifier::addIntersection(IntersectionsPercept& theIntersectionsPercept, IntersectionCandidates::IntersectionCandidate& intersection)
{
  theIntersectionsPercept.intersections.emplace_back(intersection.type, intersection.pos, intersection.cov, intersection.dir1, intersection.dir2, intersection.line1Index, intersection.line2Index);

  Vector2f fieldCoords = theRobotPose * intersection.pos;
  COMPLEX_DRAWING("module:IntersectionsClassifier:field")
  {
    DRAW_TEXT("module:IntersectionsClassifier:field", fieldCoords.x() + 5, fieldCoords.y() + 5, 10, ColorRGBA::black, std::string(1, static_cast<char>(*(TypeRegistry::getEnumName(intersection.type)))));
    CROSS("module:IntersectionsClassifier:field", fieldCoords.x(), fieldCoords.y(), 60, 30, Drawings::solidPen, ColorRGBA::blue);
  }
}

bool IntersectionsClassifier::classifyIntersection(IntersectionCandidates::IntersectionCandidate& intersection)
{
  STOPWATCH("module:IntersectionsClassifier:network") {
    const unsigned patchSize = intersection.imagePatch.height;
    PatchUtilities::extractPatch(Vector2i(patchSize/2, patchSize/2), Vector2i(patchSize, patchSize), Vector2i(patchSize, patchSize), intersection.imagePatch, network.input(0).data());
    *(network.input(1).data()) = intersection.distance;
    ASSERT(network.input(1).rank() == 1);

    network.apply();
    float l = network.output(0)[0];
    float none = network.output(0)[1];
    float t = network.output(0)[2];
    float x = network.output(0)[3];

    if(none >= threshold + 0.1f)
      return false;
    if(l >= threshold)
    {
      intersection.type = IntersectionsPercept::Intersection::L;
      return true;
    }
    if(t >= threshold)
    {
      intersection.type = IntersectionsPercept::Intersection::T;
      return true;
    }
    // We want to be especially sure before we classify an intersection as x.
    if(x >= threshold + 0.1f)
    {
      intersection.type = IntersectionsPercept::Intersection::X;
      return true;
    }
  }
  // If no prediction passes the threshold, take the original prediction by the IntersectionsCandidatesProvider.
  return true;
}
