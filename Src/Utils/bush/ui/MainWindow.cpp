#include "Platform/File.h"
#include <QBoxLayout>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include <QSplitter>
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/ui/MainWindow.h"
#include "Utils/bush/ui/RobotPool.h"
#include "Utils/bush/ui/ShortcutBar.h"
#include "Utils/bush/ui/TeamSelector.h"

MainWindow::MainWindow()
{
  setWindowTitle("bush");

  QSplitter* splitter(new QSplitter(Qt::Vertical));
  TeamSelector* teamSelector(new TeamSelector());
  splitter->addWidget(teamSelector);

  Console* console = new Console(teamSelector);
  splitter->addWidget(console);
  splitter->setStretchFactor(0, 0);

  ShortcutBar* shortcutBar = new ShortcutBar(console);
  addToolBar(Qt::BottomToolBarArea, shortcutBar);
  shortcutBar->addShortcut("help", "help");
  shortcutBar->addShortcut("deploy", "deploy");
  shortcutBar->addShortcut("download logs", "downloadLogs");
  shortcutBar->addShortcut("delete logs", "deleteLogs");
  shortcutBar->addShortcut("simulator", "sim");
  shortcutBar->addShortcut("shutdown", "shutdown -s");

  QSplitter* hSplitter(new QSplitter(Qt::Horizontal));
  hSplitter->addWidget(splitter);
  RobotPool* robotPool = new RobotPool(teamSelector);
  connect(teamSelector, SIGNAL(currentChanged(int)), robotPool, SLOT(update()));
  QFrame* rightSide = new QFrame(this);
  QGridLayout* rsLayout = new QGridLayout(rightSide);
  rsLayout->addWidget(new QLabel("<b>Robot Pool:</b>"), 0, 0);
  rsLayout->addWidget(robotPool, 1, 0);
  rightSide->setLayout(rsLayout);
  rightSide->setMaximumWidth(190);
  hSplitter->addWidget(rightSide);
  setCentralWidget(hSplitter);

  this->setWindowIcon(QPixmap(":icons/bush.png"));
  console->setFocus(Qt::OtherFocusReason);
  console->resize(60, 500);

  teamSelector->loadTeams();
  int widgetWidth = 1128;
  int widgetHeight = 695;
  splitter->setMinimumWidth(200);
  QWidget::setMinimumHeight(widgetHeight);
  QWidget::setMinimumWidth(widgetWidth);
  QWidget::resize(widgetWidth, widgetHeight);
  QDesktopWidget desktop;
  QPoint position((desktop.width() - frameGeometry().width()) / 2, (desktop.height() - frameGeometry().height()) / 2);
  QWidget::move(position);
}
