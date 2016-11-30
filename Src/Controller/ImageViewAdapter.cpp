/**
 * @file Controller/ImageViewAdapter.cpp
 *
 * ...
 *
 * @author <a href="mailto:ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#include "ImageViewAdapter.h"

std::multimap<const std::string, PointListener*> ImageViewAdapter::listeners;

PointListener::~PointListener()
{
  ImageViewAdapter::removeListener(this);
}

void ImageViewAdapter::fireClick(const std::string view, const Vector2i& point, bool upper, bool deletionRequired)
{
  for(auto iter = listeners.find(view); iter != listeners.end(); ++iter)
    iter->second->deliverPoint(point, upper, deletionRequired);
}

bool ImageViewAdapter::addListener(PointListener* listener, const std::string view)
{
  // TODO: Check if there is a view named view and return false if not.
  for(auto iter = listeners.find(view); iter != listeners.end(); ++iter)
  {
    if(iter->second == listener)
      return false;
  }
  listeners.insert(std::pair<std::string, PointListener*>(view, listener));
  return true;
}

void ImageViewAdapter::removeListener(PointListener* listener, const std::string view)
{
  for(auto iter = listeners.find(view); iter != listeners.end(); ++iter)
  {
    if(iter->second == listener)
    {
      listeners.erase(iter);
      return;
    }
  }
}

void ImageViewAdapter::removeListener(PointListener* listener)
{
  for(auto iter = listeners.begin(); iter != listeners.end();)
  {
    if(iter->second == listener)
    {
      auto temp = iter;
      ++iter;
      listeners.erase(temp);
    }
    else
      ++iter;
  }
}
