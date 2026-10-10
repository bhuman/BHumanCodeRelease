/**
 * @file SetPlay.h
 *
 * This file declares a base class for set plays.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Role.h"
#include "Tactic.h"
#include "Math/Boundary.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <limits>

STREAMABLE(SetPlay,
{
  ENUM(GameState,
  {,
    noSetPlay,
    ownKickOff,
    opponentKickOff,
    ownPenaltyKick,
    opponentPenaltyKick,
    ownFreeKick,
    opponentFreeKick,
  });

  enum Type : unsigned char
  {
    none
  };

  static unsigned ownKickOffBegin; /**< The index in the enum at which own kick-offs begin. */
  static unsigned opponentKickOffBegin; /**< The index in the enum at which opponent kick-offs begin. */
  static unsigned ownPenaltyKickBegin; /**< The index in the enum at which own penalty kicks begin. */
  static unsigned opponentPenaltyKickBegin; /**< The index in the enum at which opponent penalty kicks begin. */
  static unsigned ownFreeKickBegin; /**< The index in the enum at which own free kicks begin. */
  static unsigned opponentFreeKickBegin; /**< The index in the enum at which opponent free kicks begin. */

  struct Type_Info
  {
    static Type numOfElements; /**< The number of elements in the enum. */

    /** Registers the enumeration in the type registry. */
    static void reg();
  };

  static bool isOwnKickOff(Type type)
  {
    return type >= ownKickOffBegin && type < opponentKickOffBegin;
  }

  static bool isOpponentKickOff(Type type)
  {
    return type >= opponentKickOffBegin && type < ownPenaltyKickBegin;
  }

  static bool isKickOff(Type type)
  {
    return type >= ownKickOffBegin && type < ownPenaltyKickBegin;
  }

  static bool isOwnPenaltyKick(Type type)
  {
    return type >= ownPenaltyKickBegin && type < opponentPenaltyKickBegin;
  }

  static bool isOpponentPenaltyKick(Type type)
  {
    return type >= opponentPenaltyKickBegin && type < ownFreeKickBegin;
  }

  static bool isPenaltyKick(Type type)
  {
    return type >= ownPenaltyKickBegin && type < ownFreeKickBegin;
  }

  static bool isOwnFreeKick(Type type)
  {
    return type >= ownFreeKickBegin && type < opponentFreeKickBegin;
  }

  static bool isOpponentFreeKick(Type type)
  {
    return type >= opponentFreeKickBegin && type < Type_Info::numOfElements;
  }

  static bool isFreeKick(Type type)
  {
    return type >= ownFreeKickBegin && type < Type_Info::numOfElements;
  }

  /**
   * Checks whether a set play can be played in a specific game state.
   * @param state The game state.
   * @param type The set play to check.
   * @return Whether the set play \c type can be played in state \c state.
   */
  static bool isCompatible(GameState state, Type type);

  /** Compiles the priority groups. */
  void onRead();

  /** Merges the positions of the tactic with the ones defined in this SetPlay and generates Voronoi region subsets */
  void compileVoronoiRegions(const std::array<Tactic, Tactic::numOfTypes>& tactics);

  /**
   * Checks that the set play is valid.
   * @param setPlay The set play that this is.
   * @param tactics The available tactics (which are needed to check some stuff).
   */
  void verify(Type setPlay, const std::array<Tactic, Tactic::numOfTypes>& tactics) const;

  STREAMABLE(Position,
  {,
    (Tactic::Position::Type)(Tactic::Position::none) position, /**< The position that is overridden by this entry. */
    (Vector2f) translation, /**< The pose that is used instead. */
    (bool)(false) relativeToTeamBall, /**< Whether the pose is relative to the team ball. */
    (bool)(false) intendedStriker, /**< Whether this position is supposed to be the intended striker. */
  });

  /**
   * A map from # of remaining field players - 1 (outermost vector) to a list (central vector) of sets (inner vector) of positions which
   * are legal for that specific # of remaining field players. This member is precomputed using the priority group representation.
   */
  std::vector<std::vector<std::vector<Tactic::Position::Type>>> positionSubsetsPerNumOfAgents;

  /**
   * A map from # of remaining field players - 1 (outermost vector) to a list (central vector) of sets (inner vector) of Voronoi regions which are legal for that specific # of remaining field players.
   * Voronoi regions for the positions are represented as points of a polygon.
   * This member is precomputed using the priority group representation.
   */
  std::vector<std::vector<std::vector<std::vector<Vector2f>>>> voronoiRegionSubsetsPerNumOfAgents,

  (Tactic::Type)(Tactic::none) tactic, /**< The tactic which should be used during the set play. */
  (std::vector<Position>) positions, /**< The overridden positions in this set play. */
  (std::vector<Tactic::PriorityGroup>) priorityGroups, /**< The priority groups in this set play. */
  (Tactic::Position::Type)(Tactic::Position::none) startPosition, /**< The position which should start the set play. */
  (decltype(Tactic::PriorityGroup::priorities)::value_type)(0) lowestRequiredPriority, /**< The lowest position priority that must be available for this set play to be executed. */
});
