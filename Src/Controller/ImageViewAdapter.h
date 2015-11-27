/**
 * @file Controller/ImageViewAdapter.h
 *
 * ...
 *
 * @author <a href="mailto:ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include <string>
#include <map>
#include <memory>

class PointListener
{
public:
  virtual void deliverPoint(const Vector2i& point, bool upper, bool deletionRequired) = 0;
  virtual ~PointListener();
};

class ImageViewAdapter
{
private:
  static std::multimap<const std::string, PointListener*> listeners;
public:
  static void fireClick(const std::string view, const Vector2i& point, bool upper, bool deletionRequired);
  static bool addListener(PointListener* listener, const std::string view);
  static void removeListener(PointListener* listener, const std::string view);
  static void removeListener(PointListener* listener);
};
