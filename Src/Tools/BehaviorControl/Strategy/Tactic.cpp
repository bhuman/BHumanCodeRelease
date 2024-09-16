/**
 * @file Tactic.cpp
 *
 * This file implements the representation of a tactic.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "Tactic.h"
#include "Subset.h"
#include "Framework/Blackboard.h"
#include "Platform/BHAssert.h"
#include "Representations/Configuration/FieldDimensions.h"
#define JC_VORONOI_IMPLEMENTATION
#include <voronoi/voronoi.h>
#include <cstring>

Tactic::Position::Type Tactic::Position::mirror(Type type)
{
  switch(type)
  {
    case defenderL:
      return defenderR;
    case defenderR:
      return defenderL;
    case midfielderL:
      return midfielderR;
    case midfielderR:
      return midfielderL;
    case forwardL:
      return forwardR;
    case forwardR:
      return forwardL;
    default:
      return type;
  }
}

Tactic::Position::Position(Type type, Pose2f pose): type(type), pose(pose) {}

bool Tactic::Position::isGoalkeeper(Tactic::Position::Type type)
{
  return type == Tactic::Position::attackingGoalkeeper || type == Tactic::Position::goalkeeper;
}

Tactic::Position::Type Tactic::Position::mirrorIf(Type type, bool doIt)
{
  return doIt ? mirror(type) : type;
}

void Tactic::onRead()
{
  positionSubsetsPerNumOfAgents = compilePriorityGroups(priorityGroups);
  voronoiRegionSubsetsPerNumOfAgents = generateVoronoiRegionSubsets(positions, positionSubsetsPerNumOfAgents);
}

void Tactic::verify([[maybe_unused]] Type tactic) const
{
  std::array<int, Position::numOfTypes> existingPositions{0};
  for(const Position& position : positions)
    if(existingPositions[position.type])
      FAIL("The position " << TypeRegistry::getEnumName(position.type) << " is defined multiple times within tactic " << TypeRegistry::getEnumName(tactic) << ".");
    else
      ++existingPositions[position.type];

  if(existingPositions[Tactic::Position::none])
    FAIL("The tactic " << TypeRegistry::getEnumName(tactic) << " tries to define the position none.");

  for(const PriorityGroup& priorityGroup : priorityGroups)
    for(const Position::Type position : priorityGroup.positions)
      if(Tactic::Position::isGoalkeeper(position))
        FAIL("The goalkeeper position appears in a priority group within tactic " << TypeRegistry::getEnumName(tactic) << ".");
      else if(!existingPositions[position])
        FAIL("The position " << TypeRegistry::getEnumName(position) << " appears in a priority group but is not defined within tactic " << TypeRegistry::getEnumName(tactic) << ".");
      else
        ++existingPositions[position];

  FOREACH_ENUM(Position::Type, position)
    if(!Tactic::Position::isGoalkeeper(position))
    {
      if(existingPositions[position] == 1)
        FAIL("The position " << TypeRegistry::getEnumName(position) << " does not appear in a priority group within tactic " << TypeRegistry::getEnumName(tactic) << ".");
      else if(existingPositions[position] > 2)
        FAIL("The position " << TypeRegistry::getEnumName(position) << " appears multiple times in a priority group within tactic " << TypeRegistry::getEnumName(tactic) << ".");
    }
}

std::vector<std::vector<std::vector<Tactic::Position::Type>>> Tactic::compilePriorityGroups(const std::vector<PriorityGroup>& priorityGroups)
{
  struct PriorityGroupHandle
  {
    explicit PriorityGroupHandle(const PriorityGroup* group) :
      group(group)
    {}
    const PriorityGroup* const group = nullptr;
    std::size_t count = 0;
    std::vector<std::size_t> subset;
  };
  struct PriorityLevel
  {
    decltype(PriorityGroup::priorities)::value_type priority = 0;
    std::size_t groupIndex = 0;
  };
  std::vector<PriorityGroupHandle> priorityGroupHandles;
  std::vector<PriorityLevel> priorityLevels;
  priorityGroupHandles.reserve(priorityGroups.size());
  for(const PriorityGroup& priorityGroup : priorityGroups)
  {
    ASSERT(!priorityGroup.positions.empty());
    ASSERT(priorityGroup.positions.size() == priorityGroup.priorities.size());
    for(auto priority : priorityGroup.priorities)
      priorityLevels.push_back({priority, priorityGroupHandles.size()});
    priorityGroupHandles.emplace_back(&priorityGroup);
  }

  std::stable_sort(priorityLevels.begin(), priorityLevels.end(), [](const auto& a, const auto& b){return a.priority < b.priority;});

  std::vector<std::size_t> multisetOfGroupsAtMarginalPriority;
  std::size_t elementsOfSubsetOfMarginalPriority = 0;

  std::vector<std::vector<std::vector<Position::Type>>> result(priorityLevels.size());
  for(std::size_t numOfAgents = 1; numOfAgents <= priorityLevels.size(); ++numOfAgents)
  {
    // Is the marginal priority level exhausted?
    if(++elementsOfSubsetOfMarginalPriority > multisetOfGroupsAtMarginalPriority.size())
    {
      // The counts can be made permanent.
      for(std::size_t index : multisetOfGroupsAtMarginalPriority)
        ++priorityGroupHandles[index].count;
      // Construct the new multiset.
      multisetOfGroupsAtMarginalPriority.clear();
      for(std::size_t index = numOfAgents - 1; index < priorityLevels.size() && priorityLevels[index].priority == priorityLevels[numOfAgents - 1].priority; ++index)
        multisetOfGroupsAtMarginalPriority.push_back(priorityLevels[index].groupIndex);
      // Start with one element.
      elementsOfSubsetOfMarginalPriority = 1;
    }

    std::vector<std::size_t> subsetOfMarginalPriority(elementsOfSubsetOfMarginalPriority);
    Subset::first(multisetOfGroupsAtMarginalPriority, subsetOfMarginalPriority);
    do
    {
      // Count in the priority groups from the current subset.
      for(std::size_t index : subsetOfMarginalPriority)
        ++priorityGroupHandles[index].count;

      // The following code builds the "cartesian product" of (priorityGroupHandles[i].group->positions, priorityGroupHandles[i].count).
      // ("cartesian product" in quotes because the order of the combinations does not matter)
      for(auto& priorityGroupHandle : priorityGroupHandles)
      {
        ASSERT(priorityGroupHandle.count <= priorityGroupHandle.group->positions.size());
        priorityGroupHandle.subset.resize(priorityGroupHandle.count);
        Subset::first(priorityGroupHandle.subset);
      }
      auto nextPositionSet = [&]
      {
        for(auto& priorityGroupHandle : priorityGroupHandles)
          if(Subset::next(priorityGroupHandle.group->positions.size(), priorityGroupHandle.subset))
            return true;
        return false;
      };
      do
      {
        std::vector<Position::Type> thisSubset;
        thisSubset.reserve(numOfAgents);
        for(const auto& priorityGroupHandle : priorityGroupHandles)
          for(std::size_t index : priorityGroupHandle.subset)
            thisSubset.push_back(priorityGroupHandle.group->positions[index]);
        result[numOfAgents - 1].push_back(thisSubset);
      }
      while(nextPositionSet());

      // Restore the counts.
      for(std::size_t index : subsetOfMarginalPriority)
        --priorityGroupHandles[index].count;
    }
    while(Subset::next(multisetOfGroupsAtMarginalPriority, subsetOfMarginalPriority));
  }

  return result;
}

std::vector<std::vector<std::vector<std::vector<Vector2f>>>> Tactic::generateVoronoiRegionSubsets(const std::vector<Tactic::Position>& positions, const std::vector<std::vector<std::vector<Tactic::Position::Type>>>& positionSubsets)
{
  std::array<const Position*, Position::numOfTypes> positionMap {nullptr};
  for(const Position& position : positions)
    positionMap[position.type] = &position;

  std::vector<std::vector<std::vector<std::vector<Vector2f>>>> voronoiRegionSubsets(positionSubsets.size());
  for(size_t i = 0; i < positionSubsets.size(); i++)
  {
    voronoiRegionSubsets[i].resize(positionSubsets[i].size());
    for(size_t j = 0; j < positionSubsets[i].size(); j++)
    {
      const std::vector<Position::Type>& subset = positionSubsets[i][j];
      std::vector<Position> selectedPositions;
      for(const Position::Type& position : subset)
      {
        const Position& selectedPosition = *positionMap[position];
        if(positionMap[position])
          selectedPositions.push_back(selectedPosition);
      }

      const std::vector<std::vector<Vector2f>> regions = generateVoronoiDiagram(selectedPositions);
      for(const std::vector<Vector2f>& region : regions)
        voronoiRegionSubsets[i][j].push_back(region);
    }
  }
  return voronoiRegionSubsets;
}

std::vector<std::vector<Vector2f>> Tactic::generateVoronoiDiagram(const std::vector<Tactic::Position>& positions)
{
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  FieldDimensions& theFieldDimensions = static_cast<FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
  const float xMax = theFieldDimensions.xPosOpponentGoalLine;
  const float yMax = theFieldDimensions.yPosLeftTouchline;

  const int numOfPositions = static_cast<int>(positions.size());
  std::vector<std::vector<Vector2f>> voronoiRegions(numOfPositions);

  jcv_diagram diagram;
  // Clip regions inside field
  jcv_rect boundingBox = {{-xMax, -yMax}, {xMax, yMax}};
  std::vector<jcv_point> points(numOfPositions);

  std::memset(&diagram, 0, sizeof(jcv_diagram));

  // Get base pose from each position in the tactic
  for(int i = 0; i < numOfPositions; i++)
  {
    points[i].x = positions[i].pose.translation.x();
    points[i].y = positions[i].pose.translation.y();
  }

  jcv_diagram_generate(numOfPositions, points.data(), &boundingBox, nullptr, &diagram);

  // Get Voronoi regions with edges from diagram
  const jcv_site* sites = jcv_diagram_get_sites(&diagram);
  for(int i = 0; i < diagram.numsites; i++)
  {
    for(const jcv_graphedge* graphEdge = sites[i].edges; graphEdge; graphEdge = graphEdge->next)
    {
      // Add points of the polygon to this position's region
      voronoiRegions[sites[i].index].emplace_back(static_cast<float>(graphEdge->pos[0].x),
                                                  static_cast<float>(graphEdge->pos[0].y));
    }
  }

  jcv_diagram_free(&diagram);

  return voronoiRegions;
}
