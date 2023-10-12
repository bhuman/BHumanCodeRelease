/**
 * @file Teams.h
 *
 * This file defines a representation that contains the mapping between team names
 * and team numbers.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(Teams,
{
  STREAMABLE(Team,
  {,
    (int) number, /**< The number of the team. */
    (std::string) name, /**< The name of the team. */
  }),

  (std::vector<Team>) teams, /**< The list of all teams. */
});
