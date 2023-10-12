/**
 * DataView.cpp
 *
 *  Created on: Mar 22, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#include "DataWidget.h"
#include "PropertyTreeCreator.h"
#include "PropertyTreeWriter.h"
#include "Platform/Time.h"
#include "Debugging/DebugDataStreamer.h"
#include "Streaming/MessageQueue.h"

#include <QEvent>

DataView::DataView(const QString& fullName, const std::string& repName, const std::string& threadName,
                   RobotConsole& console, const TypeInfo& typeInfo) :
  theFullName(fullName), theIcon(":/Icons/icons8-view-50.png"),
  theConsole(console), theName(repName), theThread(threadName), typeInfo(typeInfo)
{
  theIcon.setIsMask(true);
}

SimRobot::Widget* DataView::createWidget()
{
  ignoreUpdates = 0;
  pTheWidget = new DataWidget(*this, theVariantManager);
  theConsole.requestDebugData(theThread, theName);
  return pTheWidget;
}

QtVariantProperty* DataView::getProperty(const std::string& fqn, int propertyType, const QString& name, QtProperty* pParent)
{
  QtVariantProperty* pProp = nullptr;
  auto propIt = pathsToProperties.find(fqn);

  if(propIt == pathsToProperties.end())
  {
    //This is a new property.
    pProp = theVariantManager.addProperty(propertyType, name);
    pathsToProperties[fqn] = pProp;
    propertiesToPaths[pProp] = fqn;
  }
  else
  {
    //property already exists, load it.
    pProp = pathsToProperties[fqn];
  }

  //add to parent if there is one.
  if(nullptr != pParent)
    pParent->addSubProperty(pProp);

  return pProp;
}

void DataView::removeWidget()
{
  pTheWidget = nullptr;
  pathsToProperties.clear();
  propertiesToPaths.clear();
  forceUpdate = true;
}

void DataView::setMessageReceived(MessageQueue* data)
{
  // This is SYNC_WITH(theConsole)
  this->data = data;
}

void DataView::updateTree()
{
  if(!shouldIgnoreUpdates())
  {
    SYNC_WITH(theConsole);
    if(data)
    {
      auto stream = (*data->begin()).bin();
      std::string type;
      std::string representation;
      stream >> representation >> this->type;
      PropertyTreeCreator creator(*this);
      DebugDataStreamer streamer(typeInfo, stream, this->type , "value");
      creator << streamer;
      pTheCurrentRootNode = creator.root;
      pTheWidget->setRootProperty(pTheCurrentRootNode);
      data = nullptr;
      forceUpdate = false;
    }
  }
}

void DataView::updateIgnoreUpdates(int change)
{
  ignoreUpdates += change;
  if(!shouldIgnoreUpdates())
    theConsole.requestDebugData(theThread, theName);
}

bool DataView::shouldIgnoreUpdates() const
{
  return !pTheWidget || (!forceUpdate && (setWasCalled || pTheWidget->isUnchangedEnabled() || ignoreUpdates));
}

void DataView::repoll()
{
  if(!shouldIgnoreUpdates())
    theConsole.requestDebugData(theThread, theName);
}

void DataView::setUnchanged()
{
  if(setWasCalled)
  {
    theConsole.sendDebugData(theThread, theName);
    setWasCalled = false;
  }

  pTheWidget->setSetEnabled(true);
  setUnchangedEnabled(false);
  updateIgnoreUpdates(0);
  updateWindowTitle();
}

void DataView::notifyAboutSetStatus(bool set)
{
  setWasCalled = set;

  // Force one last update of the view
  if(set)
    forceUpdate = true;

  if(pTheWidget)
  {
    pTheWidget->setSetEnabled(!set);
    setUnchangedEnabled(set);
    updateIgnoreUpdates(0);
    updateWindowTitle();
  }
}

void DataView::setUnchangedEnabled(bool value)
{
  pTheWidget->setUnchangedEnabled(value);
}

void DataView::set()
{
  if(pTheCurrentRootNode)
  {
    PropertyTreeWriter writer(theVariantManager, pTheCurrentRootNode);
    OutBinaryMemory stream;
    DebugDataStreamer streamer(typeInfo, stream, type);
    writer >> streamer;
    theConsole.sendDebugData(theThread, theName, &stream);
  }

  setWasCalled = true;
  pTheWidget->setSetEnabled(false);
  setUnchangedEnabled(true);
  updateIgnoreUpdates(0);
  updateWindowTitle();
}

void DataView::setAutoSet(bool value)
{
  theAutoSetModeIsEnabled = value;

  // If auto-set is enabled and there are unset changes, set them
  if(value && shouldIgnoreUpdates())
    set();
  else
    setUnchangedEnabled(pTheWidget->isUnchangedEnabled());
}

void DataView::valueChanged()
{
  if(ignoreUpdates)
  {
    if(theAutoSetModeIsEnabled)
      set(); //set current values
    else
    {
      pTheWidget->setSetEnabled(true); //enable "Set" menu item
      pTheWidget->setUnchangedEnabled(true); // enable "Reset" menu item
      updateIgnoreUpdates(0);
      updateWindowTitle();
    }
  }
}

bool DataView::shouldExpand(QtProperty* property) const
{
  const auto i = propertiesToPaths.find(property);
  ASSERT(i != propertiesToPaths.cend());
  const bool rootLevel = QString(i->second.c_str()).count('.') < 2;
  const bool contained = expandedPaths.find(i->second.c_str()) != expandedPaths.cend();
  return rootLevel != contained;
}

void DataView::updateExpansion(QtProperty* property, bool expanded)
{
  if(updateExpandedPaths)
  {
    const auto i = propertiesToPaths.find(property);
    ASSERT(i != propertiesToPaths.cend());
    const bool rootLevel = QString(i->second.c_str()).count('.') < 2;
    if(rootLevel != expanded)
      expandedPaths.insert(i->second);
    else
      expandedPaths.erase(i->second);
  }
}

bool DataView::shouldUseRadians(QtProperty* property) const
{
  const auto i = propertiesToPaths.find(property);
  ASSERT(i != propertiesToPaths.cend());
  return radiansPaths.find(i->second.c_str()) != radiansPaths.cend();
}

void DataView::updateUseRadians(QtProperty* property, bool radians)
{
  const auto i = propertiesToPaths.find(property);
  ASSERT(i != propertiesToPaths.cend());
  if(radians)
    radiansPaths.insert(i->second);
  else
    radiansPaths.erase(i->second);
}

void DataView::updateWindowTitle()
{
  QWidget* dockWidget = qobject_cast<QWidget*>(pTheWidget->parent());
  QString windowTitle = dockWidget->windowTitle();
  if(windowTitle.endsWith("*") || windowTitle.endsWith("↑"))
    windowTitle = windowTitle.left(windowTitle.size() - 1);
  if(setWasCalled)
    windowTitle += "↑";
  else if(pTheWidget->isUnchangedEnabled())
    windowTitle += "*";
  dockWidget->setWindowTitle(windowTitle);
}
