/**
 * @file ObstacleBoundingBox.h
 *
 * This file declares a simpel inter representation for storing
 * information about an obstacle detected in an image.
 *
 * @author Tim Laue
 */

#pragma once

class ObstacleBoundingBox
{
public:
  int top;              /**< The guessed top border of the obstacle in the image. */
  int bottom;           /**< The lower border of the obstacle in the image. */
  int left;             /**< The left border of the obstacle in the image. This usually only includes the width at the lower end. */
  int right;            /**< The right border of the obstacle in the image. This usually only includes the width at the lower end. */
  int bottomFound;      /**< Was the lower end of the obstacle found? Otherwise, it was hidden by the lower image border. */
  bool fallen;          /**< Is the obstacle a player lying on the field? */
  float distance = -1;  /**< Computed distance to the obstacle in field coordinates */
};
