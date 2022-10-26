/**
 * @file BodyContour.h
 * The file declares a struct that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct BodyContour
 * A struct that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 */
STREAMABLE(BodyContour,
{
  /** A struct representing a line in 2-D space. */
  STREAMABLE(Line,
  {
    /** Default constructor. */
    Line() = default;

    /**
     * Constructor.
     * @param p1 The first endpoint of the line.
     * @param p2 The second endpoint of the line.
     */
    Line(const Vector2i& p1, const Vector2i& p2);

    /**
     * The method determines the x coordinate of the line for a certain y coordinate
     * if there is any.
     * @param y The y coordinate.
     * @param x The x coordinate is returned here if it exists.
     * @return Does such an x coordinate exist?
     */
    bool xAt(int y, int& x) const
    {
      if((p1.y() <= y && p2.y() > y) || (p1.y() > y && p2.y() <= y))
      {
        x = p1.x() + (p2.x() - p1.x()) * (y - p1.y()) / (p2.y() - p1.y());
        return true;
      }
      else
        return false;
    }

    /**
     * The method determines the y coordinate of the line for a certain x coordinate
     * if the is any.
     * @param x The x coordinate.
     * @param y The y coordinate is returned here if it exists.
     * @return Does such a y coordinate exist?
     */
    bool yAt(int x, int& y) const
    {
      if(p1.x() <= x && p2.x() > x)
      {
        y = p1.y() + (p2.y() - p1.y()) * (x - p1.x()) / (p2.x() - p1.x());
        return true;
      }
      else
        return false;
    },

    (Vector2i) p1, /**< The left point of the line. */
    (Vector2i) p2, /**< The right point of the line. */
  });

  /** Default constructor. */
  BodyContour()
  {
    lines.reserve(50);
  }

  /**
   * The method returns the bottom y coordinate of a vertical line.
   * @param x The x coordinate of the vertical line.
   * @param yMax The maximum y coordinate that can be returned.
   * @return The maximum y coordinate for the vertical line at x.
   */
  int getBottom(int x, int yMax) const {clipBottom(x, yMax); return yMax;}

  /**
   * The method clips the bottom y coordinate of a vertical line.
   * @param x The x coordinate of the vertical line.
   * @param y The original y coordinate of the bottom of the vertical line.
   *          It will be replaced if necessary. Note that the resulting point
   *          can be outside the image!
   */
  void clipBottom(int x, int& y) const;

  /**
   * The method clips the bottom y coordinate of a vertical line.
   * @param x The x coordinate of the vertical line.
   * @param y The original y coordinate of the bottom of the vertical line.
   *          It will be replaced with a point inside the image.
   * @param imageHeight Is used to determine if the clipped y coordinate is outside
   *        the image.
   */
  void clipBottom(int x, int& y, int imageHeight) const;

  /**
   * The method clips the left x coordinate of a horizontal line.
   * It only considers descending clipping lines.
   * @param x The original x coordinate of the left end of the horizontal line.
   *          It will be replaced if necessary. Note that the resulting point
   *          can be outside the image!
   * @param y The y coordinate of the horizontal line.
   */
  void clipLeft(int& x, int y) const;

  /**
   * The method clips the right x coordinate of a horizontal line.
   * It only considers ascending clipping lines.
   * @param x The original x coordinate of the right end of the horizontal line.
   *          It will be replaced if necessary. Note that the resulting point
   *          can be outside the image!
   * @param y The y coordinate of the horizontal line.
   */
  void clipRight(int& x, int y) const;

  /**
   * @param y height of interest
   * @param imageWidth width of the current image
   * @return the x-position of the body contour's left edge in the image on height y
   */
  int getLeftEdge(int y, int imageWidth) const;

  /**
   * @param y height of interest
   * @param imageWidth width of the current image
   * @return the x-position of the body contour's right edge in the image on height y
   */
  int getRightEdge(int y, int imageWidth) const;

  /**
   * Returns the y coordinate of the highest visible point.
   * @return
   */
  int getMaxY() const;

  /**
   * Returns false if the given point is inside the body contour, true if not
   * @return
   */
  bool isValidPoint(const Vector2i& point) const;

  /** Creates drawings of the contour. */
  void draw() const,

  (std::vector<Line>) lines, /**< The clipping lines. */
  (Vector2i)(Vector2i::Zero()) cameraResolution, /**< For getMaxY() and drawing. */
});
