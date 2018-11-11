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
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/MessageQueue/InMessage.h"

#include <QEvent>

DataView::DataView(const QString& fullName, const std::string& repName,
                   RobotConsole& console, const TypeInfo& typeInfo) :
  theFullName(fullName), theIcon(":/Icons/tag_green.png"),
  theConsole(console), theName(repName), typeInfo(typeInfo)
{}

bool DataView::handleMessage(InMessage& msg, const std::string& type, const std::string& repName)
{
  SYNC;

  if(nullptr != pTheWidget && //do nothing if no widget is associated with this view.
     !theIgnoreUpdatesFlag && //Or updates should be ignored.
     Time::getRealTimeSince(lastUpdated) > theUpdateTime)
  {
    lastUpdated = Time::getRealSystemTime();
    PropertyTreeCreator creator(*this);
    DebugDataStreamer streamer(typeInfo, msg.bin, type, "value");
    creator << streamer;
    pTheCurrentRootNode = creator.root;
    this->type = type;

    //Tell the widget to display the root property
    pTheWidget->setRootProperty(pTheCurrentRootNode);
    return true;
  }
  else
    return false; //this is not an error.
}

SimRobot::Widget* DataView::createWidget()
{
  theIgnoreUpdatesFlag = false;
  pTheWidget = new DataWidget(*this, theVariantManager);
  return pTheWidget;
}

QtVariantProperty* DataView::getProperty(const std::string& fqn, int propertyType, const QString& name, QtProperty* pParent)
{
  QtVariantProperty* pProp = nullptr;
  PropertiesMapType::iterator propIt = theProperties.find(fqn);

  if(propIt == theProperties.end())
  {
    //This is a new property.
    pProp = theVariantManager.addProperty(propertyType, name);
    theProperties[fqn] = pProp;
  }
  else
  {
    //property already exists, load it.
    pProp = theProperties[fqn];
  }

  //add to parent if there is one.
  if(nullptr != pParent)
    pParent->addSubProperty(pProp);

  return pProp;
}

void DataView::removeWidget()
{
  SYNC;
  pTheWidget = nullptr;
  theProperties.clear();
}

void DataView::setIgnoreUpdates(bool value)
{
  theConsole.requestDebugData(theName, !value);
  SYNC;
  theIgnoreUpdatesFlag = value;
}

void DataView::setUnchanged()
{
  MessageQueue tempQ; //temp queue used to build the message
  std::string debugRequest = theConsole.getDebugRequest(theName);
  tempQ.out.bin << debugRequest << char(0); //0 means unchanged
  tempQ.out.finishMessage(idDebugDataChangeRequest);

  theConsole.sendDebugMessage(tempQ.in);

  pTheWidget->setUnchangedButtonEnabled(false);
  setIgnoreUpdates(false);
}

void DataView::set()
{
  std::string debugRequest = theConsole.getDebugRequest(theName);
  MessageQueue tempQ; //temp queue used to build the message
  {
    SYNC;
    if(nullptr != pTheCurrentRootNode)
    {
      tempQ.out.bin << debugRequest << char(1);

      PropertyTreeWriter writer(theVariantManager, pTheCurrentRootNode);
      DebugDataStreamer streamer(typeInfo, tempQ.out.bin, type);
      writer >> streamer;
      tempQ.out.finishMessage(idDebugDataChangeRequest);
    }
  }
  if(tempQ.getNumberOfMessages() > 0)
    theConsole.sendDebugMessage(tempQ.in);

  pTheWidget->setSetButtonEnabled(false);
  pTheWidget->setUnchangedButtonEnabled(true);
  setIgnoreUpdates(false);
}

void DataView::setAutoSet(bool value)
{
  SYNC;
  theAutoSetModeIsEnabled = value;
}

bool DataView::handlePropertyEditorEvent(QWidget* pEditor, QtProperty* pProperty, QEvent* pEvent)
{
  /*
   * FocusIn/Out is used to detect whether the user is currently editing a value.
   * However, checkboxes do not send these events. They send Paint when the user selects them
   * and LayoutRequest when the user changed a value.
   * Property updating is interrupted until the user presses set (or until the user finishes his input in case of auto-set)
   */
  if(pEvent->type() == QEvent::FocusIn || pEvent->type() == QEvent::Paint)
    setIgnoreUpdates(true);
  else if(!pTheWidget->isSetButtonEnabled() && (pEvent->type() == QEvent::FocusOut || pEvent->type() == QEvent::LayoutRequest))
  {
    valueChanged();
    setIgnoreUpdates(false);
  }

  return false; //others might want to handle this event as well.
}

void DataView::valueChanged()
{
  if(theIgnoreUpdatesFlag)
  {
    if(theAutoSetModeIsEnabled)
      set();//set current values
    else
    {
      //enable the set button
      pTheWidget->setSetButtonEnabled(true);
    }
  }
}
