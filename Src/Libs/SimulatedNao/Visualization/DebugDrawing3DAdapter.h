/**
 * @file SimulatedNao/Visualization/DebugDrawing3DAdapter.h
 *
 * Declaration of class DebugDrawing3DAdapter.
 *
 * @author Philippe Schober
 * @author Arne Hasselbring
 */

#pragma once

#include "SimulatedNao/Visualization/DebugDrawing3D.h"
#include <SimRobotCore3.h>

class RobotConsole;

class DebugDrawing3DAdapter : private DebugDrawing3D, public SimRobotCore3::Controller3DDrawing
{
  /** Helper for temporarily changing the alpha value for drawing. */
  struct Alpha
  {
    float blendColor[4]; /**< The original blend color currently set. */

    /**
     * The constructor integrates the alpha value into the current blend color.
     * @param alpha The alpha value in the range of 0 ... 255.
     */
    Alpha(unsigned char alpha);

    /** The destructor resets the blend color to its previous value. */
    ~Alpha();
  };

public:
  RobotConsole* robotConsole = nullptr; /**< The robot console to which this drawing belongs. */
  QVector<QString> parts; /**< The path to the SimRobotCore3 object that this drawing is attached to. */
  bool drawn = false; /**< Is this drawing already registered? */
  bool flip = false; /**< Rotate drawings 180 degrees around the z-axis. */

  /**
   * Copies the contents of another drawing into this one.
   * @param other The other drawing.
   */
  void copyFrom(const DebugDrawing3D& other);

  /** The function empties the drawing. */
  void reset();

private:
  /**
   * Allocates resources before drawing.
   * @param projection The column-major 4x4 projection matrix.
   * @param view The column-major 4x4 view matrix.
   * @param model The column-major 4x4 model matrix.
   */
  void beforeFrame(const float* projection, const float* view, const float* model) override;

  /** Issues draw calls. */
  void draw() override;

  /** Deallocates resources after drawing. */
  void afterFrame() override;
};
