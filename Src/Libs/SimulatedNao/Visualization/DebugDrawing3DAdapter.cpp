/**
 * @file SimulatedNao/Visualization/DebugDrawing3D.cpp
 *
 * Implementation of class DebugDrawing3DAdapter.
 *
 * @author Philippe Schober
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Arne Hasselbring
 */

#include "DebugDrawing3DAdapter.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include "SimulatedNao/RobotConsole.h"
#include "SimulatedNao/Visualization/PaintMethods3DOpenGL.h"

void DebugDrawing3DAdapter::copyFrom(const DebugDrawing3D& other)
{
  scale = other.scale;
  rotate = other.rotate;
  trans = other.trans;
  renderOptions = other.renderOptions;

  lines = other.lines;
  dots = other.dots;
  quads = other.quads;
  spheres = other.spheres;
  ellipsoids = other.ellipsoids;
  cylinders = other.cylinders;
  images = other.images;
}

void DebugDrawing3DAdapter::reset()
{
  lines.clear();
  dots.clear();
  quads.clear();
  spheres.clear();
  ellipsoids.clear();
  cylinders.clear();
  images.clear();
}

void DebugDrawing3DAdapter::beforeFrame(const float* projection, const float* view, const float* model)
{
  // The lock is released in \c afterFrame because \c draw and \c afterFrame must use the same state of the drawing.
  robotConsole->_mutex.lock();
  RoboCupCtrl::controller->paintMethods3D->beforeFrame(*this, Eigen::Map<const Matrix4f>(projection), Eigen::Map<const Matrix4f>(view), Eigen::Map<const Matrix4f>(model), flip);
}

void DebugDrawing3DAdapter::draw()
{
  RoboCupCtrl::controller->paintMethods3D->draw(*this);
}

void DebugDrawing3DAdapter::afterFrame()
{
  robotConsole->_mutex.unlock();
}
