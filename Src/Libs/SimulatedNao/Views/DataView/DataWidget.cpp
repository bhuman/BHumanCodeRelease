/**
 * DataWidget.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: arne
 */

#include "DataWidget.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include <QSettings>
#include <QStringList>

DataWidget::DataWidget(DataView& view, QtVariantPropertyManager& manager)
  : theView(view), theEditorFactory(&view), theManager(manager)
{
  setFocusPolicy(Qt::StrongFocus);
  setResizeMode(QtTreePropertyBrowser::ResizeToContents);
  setHeaderVisible(false);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(theView.getFullName());
  theView.theAutoSetModeIsEnabled = settings.value("AutoSet", theView.theAutoSetModeIsEnabled).toBool();
  theView.expandedPaths.clear();
  for(const QString& path : settings.value("ExpandedPaths", QStringList()).toStringList())
    theView.expandedPaths.insert(path.toStdString());
  theView.radiansPaths.clear();
  for(const QString& path : settings.value("RadiansPaths", QStringList()).toStringList())
    theView.radiansPaths.insert(path.toStdString());
  settings.endGroup();

  QIcon setIcon(":/Icons/icons8-upload-50.png");
  setIcon.setIsMask(true);
  pSetAction = new QAction(setIcon, tr("&Set"), this);
  pSetAction->setText("Set");
  pSetAction->setToolTip("Overwrite data on robot");
  pSetAction->setEnabled(true);
  QIcon unchangedIcon(":/Icons/icons8-undo-50.png");
  unchangedIcon.setIsMask(true);
  pUnchangedAction = new QAction(unchangedIcon, tr("&Unchanged"), this);
  setUnchangedEnabled(theView.setWasCalled);
  QIcon autoSetIcon(":/Icons/icons8-synchronize-50.png");
  autoSetIcon.setIsMask(true);
  pAutoSetAction = new QAction(autoSetIcon, tr("&Auto-set"), this);
  pAutoSetAction->setCheckable(true);
  pAutoSetAction->setChecked(theView.theAutoSetModeIsEnabled);
  pAutoSetAction->setText("Auto-set");
  pAutoSetAction->setToolTip("Continuously overwrite data on robot");

  connect(pSetAction, &QAction::triggered, this, &DataWidget::setPressed);
  connect(pUnchangedAction, &QAction::triggered, this, &DataWidget::unchangedPressed);
  connect(pAutoSetAction, &QAction::toggled, this, &DataWidget::autoSetToggled);
  connect(&theManager, &QtVariantPropertyManager::valueChanged, this, &DataWidget::valueChanged);
  connect(this, &QtTreePropertyBrowser::collapsed, this, &DataWidget::collapsed);
  connect(this, &QtTreePropertyBrowser::expanded, this, &DataWidget::expanded);

  setFactoryForManager(&theManager, &theEditorFactory);
}

DataWidget::~DataWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(theView.getFullName());
  settings.setValue("AutoSet", theView.theAutoSetModeIsEnabled);
  QStringList expandedPaths;
  for(const std::string& path : theView.expandedPaths)
    expandedPaths.append(path.c_str());
  settings.setValue("ExpandedPaths", expandedPaths);
  QStringList radiansPaths;
  for(const std::string& path : theView.radiansPaths)
    radiansPaths.append(path.c_str());
  settings.setValue("RadiansPaths", radiansPaths);
  settings.endGroup();

  //Without a view the widget will stop updating the properties.
  theView.removeWidget(); //Remove the Widget from the view.
  clear();
}

void DataWidget::update()
{
  // Update properties of the tree.
  theView.updateTree();

  // Request new data if it is needed.
  theView.repoll();

  //Adding the property to the browser draws the property.
  //Therefore this is done in the gui thread.

  if(theRootPropertyHasChanged && nullptr != pTheCurrentProperty)
  {
    //If the property has children it is only a container.
    //Discard the container and add the children directly for a more compact view.
    QList<QtProperty*> subProps = pTheCurrentProperty->subProperties();
    if(subProps.size() > 0)
    {
      for(int i = 0; i < subProps.size(); i++)
        addProperty(subProps[i]);
    }
    else
      addProperty(pTheCurrentProperty);

    theRootPropertyHasChanged = false;
  }
}

QMenu* DataWidget::createUserMenu() const
{
  QMenu* pMenu = new QMenu(theView.getFullName());
  pMenu->addAction(pSetAction);
  pMenu->addAction(pUnchangedAction);
  pMenu->addAction(pAutoSetAction);
  return pMenu;
}

void DataWidget::setRootProperty(QtProperty* pRootProperty)
{
  if(pTheCurrentProperty != pRootProperty)
  {
    theRootPropertyHasChanged = true;
    pTheCurrentProperty = pRootProperty;
  }
}

void DataWidget::setSetEnabled(bool value)
{
  pSetAction->setEnabled(value);
}

void DataWidget::setUnchangedEnabled(bool value)
{
  pUnchangedAction->setEnabled(value);
  pUnchangedAction->setToolTip(theView.setWasCalled || theView.theAutoSetModeIsEnabled
                               ? "Switch back to using original data on robot"
                               : "Show data received from robot again");
  pUnchangedAction->setText(theView.setWasCalled || theView.theAutoSetModeIsEnabled ? "Unchanged" : "Reset");
}

bool DataWidget::isUnchangedEnabled() const
{
  return pUnchangedAction->isEnabled();
}

void DataWidget::setPressed()
{
  theView.set();
}

void DataWidget::autoSetToggled(bool checked)
{
  theView.setAutoSet(checked);
}

void DataWidget::unchangedPressed()
{
  theView.setUnchanged();
}

void DataWidget::itemInserted(QtBrowserItem* insertedItem, QtBrowserItem* precedingItem)
{
  theView.updateExpandedPaths = false;
  QtTreePropertyBrowser::itemInserted(insertedItem, precedingItem);
  setExpanded(insertedItem, theView.shouldExpand(insertedItem->property()));
  theView.updateExpandedPaths = true;
}

void DataWidget::valueChanged(QtProperty*, const QVariant&)
{
  theView.valueChanged();
}

void DataWidget::collapsed(QtBrowserItem* item)
{
  theView.updateExpansion(item->property(), false);
}

void DataWidget::expanded(QtBrowserItem* item)
{
  theView.updateExpansion(item->property(), true);
}
