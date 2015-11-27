
#include <QAction>

#include "StatusBar.h"

StatusBar::StatusBar(QWidget* parent) : QStatusBar(parent), toggleViewAct(0)
{
  userMessage = latestMessage = tr("Ready");
  showMessage(userMessage);
  connect(this, SIGNAL(messageChanged(const QString&)), this, SLOT(messageChanged(const QString&)));
}

void StatusBar::addLabel(const SimRobot::Module* module, SimRobot::StatusLabel* statusLabel)
{
  insertPermanentWidget(0, statusLabel->getWidget());
  registeredLables.append(RegisteredLabel(module, statusLabel));
}

void StatusBar::removeAllLabels()
{
  for(QList<RegisteredLabel>::iterator it = registeredLables.begin(), end = registeredLables.end(); it != end; ++it)
  {
    removeWidget(it->label->getWidget());
    delete it->label;
  }
  registeredLables.clear();
}

void StatusBar::removeLabelsFromModule(const SimRobot::Module* module)
{
  for(QList<RegisteredLabel>::iterator it = registeredLables.begin(); it != registeredLables.end();)
    if(it->module == module)
    {
      removeWidget(it->label->getWidget());
      delete it->label;
      it = registeredLables.erase(it);
    }
    else
      ++it;
}

void StatusBar::setUserMessage(const QString& userMessage)
{
  latestMessage = userMessage;
}

void StatusBar::update()
{
  for(QList<RegisteredLabel>::iterator it = registeredLables.begin(), end = registeredLables.end(); it != end; ++it)
    it->label->update();

  QString currentMessage = this->currentMessage();
  if(currentMessage.isEmpty() || currentMessage == userMessage)
  {
    userMessage = latestMessage;
    if(!currentMessage.isEmpty() || !userMessage.isEmpty())
      showMessage(userMessage);
  }
}

QAction* StatusBar::toggleViewAction()
{
  if(toggleViewAct)
    return toggleViewAct;
  toggleViewAct = new QAction(tr("&Status Bar"), this);
  toggleViewAct->setCheckable(true);
  toggleViewAct->setChecked(isVisible());
  connect(toggleViewAct, SIGNAL(triggered(bool)), this, SLOT(setVisible(bool)));
  return toggleViewAct;
}

void StatusBar::hideEvent(QHideEvent* event)
{
  QStatusBar::hideEvent(event);

  if(toggleViewAct)
    toggleViewAct->setChecked(false);
}

void StatusBar::showEvent(QShowEvent* event)
{
  QStatusBar::showEvent(event);

  if(toggleViewAct)
    toggleViewAct->setChecked(true);
}

void StatusBar::messageChanged(const QString& message)
{
  if(message.isEmpty() && (!userMessage.isEmpty() || latestMessage != userMessage))
  {
    userMessage = latestMessage;
    showMessage(userMessage);
  }
}
