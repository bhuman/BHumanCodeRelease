/**
 * @file Controller/Visualization/PaintMethods.h
 * Declaration of class PaintMethods.
 *
 * @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
 * @author Colin Graf
 */

#pragma once

class DebugDrawing;
class QPainter;

/**
* @class PaintMethods
*
* Defines static methods to paint debug drawings to QPainters.
*/
class PaintMethods
{
public:
  /**
  * Paints a DebugDrawings to a QPainter.
  * @param painter The graphics context the DebugDrawing is painted to.
  * @param debugDrawing The DebugDrawing to paint.
  * @param baseTrans A basic transformation.
  */
  static void paintDebugDrawing(QPainter& painter, const DebugDrawing& debugDrawing, const QTransform& baseTrans);
};
