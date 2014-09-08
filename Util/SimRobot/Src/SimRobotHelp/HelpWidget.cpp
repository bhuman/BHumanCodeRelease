#include <QVBoxLayout>
#include <QToolBar>
#include <QTabWidget>
#include <QAction>
#include <QTextBrowser>
#include <QSettings>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QTextStream>
#include <QHelpEngine>
#include <QHelpContentWidget>
#include <QHelpIndexWidget>
#include <QHelpSearchEngine>
#include <QHelpSearchQueryWidget>
#include <QHelpSearchResultWidget>

#include "HelpModule.h"
#include "HelpWidget.h"

static const QUrl& HOME_PAGE = QUrl("qthelp://simrobothelp/doc/index.html");

SimRobot::Widget* HelpObject::createWidget()
{
  return new HelpWidget(*this);
}

class HelpBrowser : public QTextBrowser
{
public:
  HelpBrowser(QHelpEngine* helpEngine, QWidget* parent) : QTextBrowser(parent), helpEngine(helpEngine)
  {
    QFile file(":/simrobothelp.css");

    if(!file.open(QIODevice::ReadOnly))
      return;

    QTextStream stream(&file);
    QString css;
    while(!stream.atEnd())
      css += stream.readLine();

    file.close();

    document()->addResource(QTextDocument::StyleSheetResource, QUrl("simrobothelp.css" ), css);
  }

  virtual QVariant loadResource(int type, const QUrl &url)
  {
    if(url.scheme() == "qthelp")
      return QVariant(helpEngine->fileData(url));
    return QTextBrowser::loadResource(type, url);
  }

private:
  QHelpEngine* helpEngine;
};

HelpWidget::HelpWidget(const SimRobot::Object& object) : QSplitter(Qt::Horizontal), object(object)
{
  QString qhc = QFileInfo(HelpModule::application->getAppPath()).dir().path() +
#ifdef OSX
  "/../Resources" +
#endif
  "/helpcollection.qhc";
  helpEngine = new QHelpEngine(qhc, this);
  helpBrowser = new HelpBrowser(helpEngine, this);
  helpBrowser->setFrameStyle(QFrame::NoFrame);

  if(!helpEngine->setupData())
    HelpModule::application->showWarning(tr("SimRobotHelp"), helpEngine->error());

  QHelpContentWidget* contentWidget = helpEngine->contentWidget();
  contentWidget->setFrameStyle(QFrame::NoFrame);

  QHelpIndexWidget* indexWidget = helpEngine->indexWidget();
  indexWidget->setFrameStyle(QFrame::NoFrame);

  QTabWidget* leftWidget = new QTabWidget(this);
  leftWidget->addTab(contentWidget, "Contents");
  leftWidget->addTab(indexWidget, "Index");

  QWidget* searchTabWidget = new QWidget(this);
  QVBoxLayout* searchTabLayout = new QVBoxLayout(this);
  searchTabLayout->addWidget(helpEngine->searchEngine()->queryWidget());
  searchTabLayout->addWidget(new QLabel("Search results:", this));
  searchResultsWidget = new QTextBrowser(this);
  searchTabLayout->addWidget(searchResultsWidget);
  searchTabWidget->setLayout(searchTabLayout);

  leftWidget->addTab(searchTabWidget, "Search");

  QToolBar* toolBar = new QToolBar(this);

  QAction* navigateHomeAction = new QAction(QIcon(":/Icons/home.png"), tr("Home"), this);
  navigateHomeAction->setStatusTip(tr("Navigate Home"));
  connect(navigateHomeAction, SIGNAL(triggered()), this, SLOT(navigateHome()));
  toolBar->addAction(navigateHomeAction);

  QAction* locatePageAction = new QAction(QIcon(":/Icons/locate.png"), tr("Locate"), this);
  locatePageAction->setStatusTip(tr("Locate current page in contents window"));
  connect(locatePageAction, SIGNAL(triggered()), this, SLOT(locatePage()));
  toolBar->addAction(locatePageAction);

  toolBar->addSeparator();

  QAction* navigateBackwardAction = new QAction(QIcon(":/Icons/back.png"), tr("Back"), this);
  navigateBackwardAction->setStatusTip(tr("Navigate Back"));
  connect(navigateBackwardAction, SIGNAL(triggered()), this, SLOT(navigateBackward()));
  toolBar->addAction(navigateBackwardAction);

  QAction* navigateForwardAction = new QAction(QIcon(":/Icons/forward.png"), tr("Forward"), this);
  navigateForwardAction->setStatusTip(tr("Navigate Forward"));
  connect(navigateForwardAction, SIGNAL(triggered()), this, SLOT(navigateForward()));
  toolBar->addAction(navigateForwardAction);

  QWidget* rightWidget = new QWidget(this);

  QVBoxLayout* rightWidgetLayout = new QVBoxLayout(this);
  rightWidgetLayout->addWidget(toolBar);
  rightWidgetLayout->addWidget(helpBrowser);
  rightWidget->setLayout(rightWidgetLayout);

  addWidget(leftWidget);
  addWidget(rightWidget);

  connect(helpBrowser, SIGNAL(forwardAvailable(bool)), navigateForwardAction, SLOT(setEnabled(bool)));
  connect(helpBrowser, SIGNAL(backwardAvailable(bool)), navigateBackwardAction, SLOT(setEnabled(bool)));

  connect(contentWidget, SIGNAL(linkActivated(const QUrl&)), helpBrowser, SLOT(setSource(const QUrl &)));
  connect(indexWidget, SIGNAL(linkActivated(const QUrl&, const QString&)), helpBrowser, SLOT(setSource(const QUrl &)));

  connect(this, SIGNAL(tabChangeRequested(int)), leftWidget, SLOT(setCurrentIndex(int)));
  connect(helpEngine->searchEngine()->queryWidget(), SIGNAL(search()), this, SLOT(searchInvoked()));
  connect(helpEngine->searchEngine(), SIGNAL(searchingFinished(int)), this, SLOT(searchFinished(int)));

  helpEngine->searchEngine()->reindexDocumentation();

  searchResultsWidget->setOpenLinks(false);
  searchResultsWidget->setOpenExternalLinks(false);

  connect(searchResultsWidget, SIGNAL(anchorClicked(const QUrl&)), helpBrowser, SLOT(setSource(const QUrl&)));

  navigateHome();

  setStretchFactor(0,1);
  setStretchFactor(1,2);

  setFrameStyle(QFrame::NoFrame);

  QSettings& settings = HelpModule::application->getLayoutSettings();
  settings.beginGroup(object.getFullName());
  restoreState(settings.value("State").toByteArray());
  settings.endGroup();
}

QSize HelpWidget::sizeHint() const
{
  QSplitter::sizeHint();
  return QSize(1024, 768);
}

void HelpWidget::navigateHome()
{
  helpEngine->contentWidget()->setCurrentIndex(helpEngine->contentWidget()->indexOf(HOME_PAGE));
  helpBrowser->setSource(HOME_PAGE);
}

void HelpWidget::locatePage()
{
  helpEngine->contentWidget()->setCurrentIndex(helpEngine->contentWidget()->indexOf(helpBrowser->source()));
  emit tabChangeRequested(0);
}

void HelpWidget::navigateForward()
{
  helpBrowser->forward();
}

void HelpWidget::navigateBackward()
{
  helpBrowser->backward();
}

void HelpWidget::searchInvoked()
{
  helpEngine->searchEngine()->search(helpEngine->searchEngine()->queryWidget()->query());
}

void HelpWidget::searchFinished(int hits)
{
  QList<QPair<QString, QString> > hitList = helpEngine->searchEngine()->hits(0, hits - 1);
  QString html = "<html><body>";
  for(int i = 0; i < hitList.size(); i++)
  {
    QString url = hitList[i].first;
    QString title = hitList[i].second;
    html += "<a href=\"" + url + "\">" + title + "</a><br>";
  }
  html += "</body></html>";
  searchResultsWidget->setHtml(html);
}

HelpWidget::~HelpWidget()
{
  QSettings& settings = HelpModule::application->getLayoutSettings();
  settings.beginGroup(object.getFullName());
  settings.setValue("State", saveState());
  settings.endGroup();
  delete searchResultsWidget;
  delete helpBrowser;
  delete helpEngine;
}
