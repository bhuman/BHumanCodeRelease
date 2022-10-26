#include "MainWindow.h"
#include "models/Team.h"
#include "ui/CommandBar.h"
#include "ui/Console.h"
#include "ui/RobotPool.h"
#include "ui/SizeManager.h"
#include "ui/TeamSelector.h"
#include "Platform/File.h"
#include <QBoxLayout>
#include <QGridLayout>
#include <QGuiApplication>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include <QSplitter>
#include <QSettings>
#include <QApplication>
#include <QMouseEvent>
#include <qstyleoption.h>

MainWindow::MainWindow()
{
  setWindowTitle("bush");
  SizeManager sizeManager;
  int widgetWidth = sizeManager.widgetWidth - frameGeometry().width();
  QSplitter* splitter(new QSplitter(Qt::Vertical));
  TeamSelector* teamSelector(new TeamSelector());
  splitter->addWidget(teamSelector);
  splitter->setStretchFactor(0, 0);

  QSplitter* hSplitter(new QSplitter(Qt::Horizontal));
  hSplitter->addWidget(splitter);
  RobotPool* robotPool = new RobotPool(teamSelector);
  connect(teamSelector, &QTabWidget::currentChanged, robotPool, &RobotPool::update);
  QFrame* rightSide = new QFrame(this);
  QGridLayout* rsLayout = new QGridLayout(rightSide);
  rsLayout->addWidget(new QLabel("<b>Robot Pool:</b>"), 0, 0);
  rsLayout->addWidget(robotPool, 1, 0);
  rightSide->setLayout(rsLayout);
  rightSide->setMinimumWidth(static_cast<int>(widgetWidth / 6));
  hSplitter->addWidget(rightSide);
  setCentralWidget(hSplitter);

  Console* console = new Console(teamSelector);
  this->setWindowIcon(QPixmap(":icons/bush.png"));
  console->setFocus(Qt::OtherFocusReason);
  console->resize(60, 500);

  CommandBar* commandBar = new CommandBar(console, teamSelector);
#ifdef MACOS
  setUnifiedTitleAndToolBarOnMac(true);
#endif
  addToolBar(Qt::TopToolBarArea, commandBar);
  splitter->addWidget(console);

  int widgetWidthResize = 1128;
  int widgetHeightResize = 695;
  teamSelector->loadTeams();
  splitter->setMinimumWidth(100);
  QWidget::resize(widgetWidthResize, widgetHeightResize);
  QRect desktop = QGuiApplication::primaryScreen()->virtualGeometry();
  QPoint position((desktop.width() - frameGeometry().width()) / 2, (desktop.height() - frameGeometry().height()) / 2);
  QWidget::move(position);
  readSettings();

  setAcceptDrops(true);
}

void MainWindow::dragEnterEvent(QDragEnterEvent* e)
{
  if(e->source() && (e->source()->inherits("RobotView") || e->source()->inherits("RobotPool")))
    e->acceptProposedAction();
}

void MainWindow::dragMoveEvent(QDragMoveEvent* e)
{
  if(e->source() && (e->source()->inherits("RobotView") || e->source()->inherits("RobotPool")))
    e->acceptProposedAction();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QSettings settings("B-Human", "bush");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  QMainWindow::closeEvent(event);
}

void MainWindow::readSettings()
{
  QSettings settings("B-Human", "bush");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}
