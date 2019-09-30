/**
 * DataWidget.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: arne
 */

#include "DataWidget.h"
#include "Controller/RoboCupCtrl.h"
#include <QSettings>

DataWidget::DataWidget(DataView& view, QtVariantPropertyManager& manager)
  : theView(view), theEditorFactory(&view), theManager(manager)
{
  setFocusPolicy(Qt::StrongFocus);

  pSetAction = new QAction(QIcon(":/Icons/upload.png"), tr("&Set"), this);
  pSetAction->setText("Set");
  pSetAction->setToolTip("Overwrite data on robot");
  pSetAction->setEnabled(false);
  pUnchangedAction = new QAction(QIcon(":/Icons/arrow_undo.png"), tr("&Unchanged"), this);
  pUnchangedAction->setToolTip("Switch back to using original data on robot");
  pUnchangedAction->setEnabled(false);
  pUnchangedAction->setText("Unchanged");
  pAutoSetAction = new QAction(QIcon(":/Icons/arrow_refresh.png"), tr("&Auto-set"), this);
  pAutoSetAction->setCheckable(true);
  pAutoSetAction->setChecked(true);
  pAutoSetAction->setText("Auto-set");
  pAutoSetAction->setToolTip("Continuously overwrite data on robot");

  connect(pSetAction, SIGNAL(triggered()), this, SLOT(setPressed()));
  connect(pUnchangedAction, SIGNAL(triggered()), this, SLOT(unchangedPressed()));
  connect(pAutoSetAction, SIGNAL(toggled(bool)), this, SLOT(autoSetToggled(bool)));
  connect(&theManager, SIGNAL(valueChanged(QtProperty*, const QVariant&)), this, SLOT(valueChanged(QtProperty*, const QVariant&)));

  setFactoryForManager(&theManager, &theEditorFactory);
  setResizeMode(QtTreePropertyBrowser::Interactive);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(theView.getFullName());
  setSplitterPosition(settings.value("HeaderState", 100).toInt());
  settings.endGroup();
}

DataWidget::~DataWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(theView.getFullName());
  settings.setValue("HeaderState", splitterPosition());
  settings.endGroup();

  //Without a view the widget will stop updating the properties.
  theView.removeWidget(); //Remove the Widget from the view.
  clear();
}

void DataWidget::update()
{
  //Adding the property to the browser draws the property.
  //Therefore this is done in the gui thread.
  SYNC_WITH(theView); //without this lock pTheCurrentProperty might be changed while adding it.

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
};

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

void DataWidget::setUnchangedButtonEnabled(bool value)
{
  pUnchangedAction->setEnabled(value);
}

void DataWidget::setSetButtonEnabled(bool value)
{
  pSetAction->setEnabled(value);
}

bool DataWidget::isSetButtonEnabled() const
{
  return pSetAction->isEnabled();
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
  setUnchangedButtonEnabled(false);
}

void DataWidget::itemInserted(QtBrowserItem* insertedItem, QtBrowserItem* precedingItem)
{
  QtTreePropertyBrowser::itemInserted(insertedItem, precedingItem);
  setExpanded(insertedItem, !insertedItem->parent());
}

void DataWidget::valueChanged(QtProperty*, const QVariant&)
{
  theView.valueChanged();
}
