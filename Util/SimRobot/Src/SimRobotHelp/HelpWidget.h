
#pragma once

#include <QSplitter>
#include <QIcon>
#include <QSize>

#include "../SimRobot/SimRobot.h"

class QHelpEngine;
class HelpBrowser;
class QTextBrowser;

class HelpObject : public SimRobot::Object
{
public:
  HelpObject() : name("Help"), icon(":/Icons/help.png") {}

protected:
  QString name;
  QIcon icon;

  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const {return name;}
  virtual const QIcon* getIcon(void) const {return &icon;}
};

class HelpWidget : public QSplitter, public SimRobot::Widget
{
  Q_OBJECT

public:
  HelpWidget(const SimRobot::Object& object);
  ~HelpWidget();

private:
  const SimRobot::Object& object;
  QHelpEngine* helpEngine;
  HelpBrowser* helpBrowser;

  QTextBrowser* searchResultsWidget;

  virtual QWidget* getWidget() {return this;}

  virtual QSize sizeHint() const;

private slots:
  void navigateHome();
  void locatePage();
  void navigateBackward();
  void navigateForward();

  void searchInvoked();

  void searchFinished(int hits);

signals:
  void tabChangeRequested(int tabIndex);
};
