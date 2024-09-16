/**
 * @file Tactic.h
 *
 * This file declares the representation of a tactic.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <vector>

STREAMABLE(Tactic,
{
  ENUM(Type,
  {,
    none,
    t011,
    t020,
    t123,
    t100,
    t222,
    t033,
  });

  STREAMABLE(Position,
  {
    ENUM(Type,
    {,
      none,
      goalkeeper,
      attackingGoalkeeper,
      defender,
      defenderL,
      defenderR,
      midfielder,
      midfielderM,
      midfielderL,
      midfielderR,
      forward,
      forwardM,
      forwardL,
      forwardR,
      sacPasser,
    });

    Position() = default;
    Position(Type type, Pose2f pose);

    /**
     * Mirrors a position type.
     * @param type The input type.
     * @return The mirrored type.
     */
    static Type mirror(Type type);

    /**
     * Mirrors a position type if some condition is true.
     * @param type The input type.
     * @param doIt Whether to actually mirror the type.
     * @return The mirrored (or not) type.
     */
    static Type mirrorIf(Type type, bool doIt);

    /**
     * Checks whether the position type is a goalkeeper (attackingGoalkeeper or goalkeeper).
     * @param type The input type
     * @return True if its a goalkeeper of any kind (attackingGoalkeeper or goalkeeper) else False.
     */
    static bool isGoalkeeper(Tactic::Position::Type type),

    (Type)(none) type, /**< The position type which is defined here. */
    (Pose2f) pose, /**< The "home" pose of this position. */
    (float)(1000.f) stabilityOffset, /**< The cost for assigning the agent who had this position before to this position is decreased by this. */
  });

  STREAMABLE(PriorityGroup,
  {,
    (std::vector<Position::Type>) positions, /**< The positions in this priority group. */
    (std::vector<unsigned int>) priorities, /**< The priorities in this priority group. */
  });

  /** Compiles the priority groups and generates the Voronoi diagram. */
  void onRead();

  /**
   * Checks that the tactic is valid.
   * @param tactic The tactic that this is.
   */
  void verify(Type tactic) const;

  /**
   * Compiles a priority group representation into subsets per number of field players.
   * @param priorityGroups The set of priority groups.
   * @return A full description of all allowed subsets of positions per number of field players.
   */
  static std::vector<std::vector<std::vector<Tactic::Position::Type>>> compilePriorityGroups(const std::vector<Tactic::PriorityGroup>& priorityGroups);

  /**
   * Generates Voronoi regions for positions in subsets per number of field players.
   * @param positions The set of positions in the tactic.
   * @return A full description of all subsets of Voronoi regions per number of field players.
   */
  static std::vector<std::vector<std::vector<std::vector<Vector2f>>>> generateVoronoiRegionSubsets(const std::vector<Tactic::Position>& positions, const std::vector<std::vector<std::vector<Tactic::Position::Type>>>& positionSubsets);

  /**
   * Generates a Voronoi diagram of base poses for the positions in this tactic.
   * @param positions The set of positions in the tactic.
   * @return Voronoi Regions for the positions represented as points of a polygon.
   */
  static std::vector<std::vector<Vector2f>> generateVoronoiDiagram(const std::vector<Tactic::Position>& positions);

  /**
   * A map from # of remaining field players - 1 (outermost vector) to a list (central vector) of sets (inner vector) of positions which
   * are legal for that specific # of remaining field players. This member is precomputed using the priority group representation.
   */
  std::vector<std::vector<std::vector<Position::Type>>> positionSubsetsPerNumOfAgents;

  /**
   * A map from # of remaining field players - 1 (outermost vector) to a list (central vector) of sets (inner vector) of Voronoi regions which are legal for that specific # of remaining field players.
   * Voronoi regions for the positions are represented as points of a polygon.
   * This member is precomputed using the priority group representation.
   */
  std::vector<std::vector<std::vector<std::vector<Vector2f>>>> voronoiRegionSubsetsPerNumOfAgents,

  (std::vector<Position>) positions, /**< The positions in this tactic. */
  (std::vector<PriorityGroup>) priorityGroups, /**< The priority groups in this tactic. */
});
