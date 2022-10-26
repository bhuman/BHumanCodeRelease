/**
 * @file ObstaclesPolygonPercept.h
 *
 *
 *
 * @author Nele Matschull
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(ObstaclesPolygon,
{,
  (std::vector<Vector2f>) polygon, /**< A list of points of the polygon */
});

STREAMABLE(ObstaclesPolygonPercept,
{,
  (std::vector<ObstaclesPolygon>) obstacles, /**< The list of obstacles */
});
