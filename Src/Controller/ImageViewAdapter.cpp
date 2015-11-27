/**
 * @file Controller/ImageViewAdapter.cpp
 *
 * ...
 *
 * @author <a href="mailto:ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#include "ImageViewAdapter.h"

using namespace std;

multimap<const string, PointListener*> ImageViewAdapter::listeners;

PointListener::~PointListener()
{
  ImageViewAdapter::removeListener(this);
}

void ImageViewAdapter::fireClick(const string view, const Vector2i& point, bool upper, bool deletionRequired)
{
  for(multimap<const string, PointListener*>::iterator iter = listeners.find(view);
      iter != listeners.end();
      ++iter)
  {
    iter->second->deliverPoint(point, upper, deletionRequired);
  }
}

bool ImageViewAdapter::addListener(PointListener* listener, const string view)
{
  // TODO: Check if there is a view named view and return false if not.
  for(multimap<const string, PointListener*>::iterator iter = listeners.find(view);
      iter != listeners.end();
      ++iter)
  {
    if(iter->second == listener)
      return false;
  }
  listeners.insert(pair<string, PointListener*>(view, listener));
  return true;
}

void ImageViewAdapter::removeListener(PointListener* listener, const string view)
{
  for(multimap<const string, PointListener*>::iterator iter = listeners.find(view);
      iter != listeners.end();
      ++iter)
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
  for(multimap<const string, PointListener*>::iterator iter = listeners.begin();
      iter != listeners.end(); )
  {
    if(iter->second == listener)
    {
      multimap<const string, PointListener*>::iterator temp = iter;
      ++iter;
      listeners.erase(temp);
    }
    else
    {
      ++iter;
    }
  }
}
