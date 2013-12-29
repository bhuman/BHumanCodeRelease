#include "Platform/File.h"
#include <QBoxLayout>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/ui/MainWindow.h"
#include "Utils/bush/ui/RobotPool.h"
#include "Utils/bush/ui/ShortcutBar.h"
#include "Utils/bush/ui/TeamSelector.h"

MainWindow::MainWindow()
  : teamSelector(new TeamSelector()),
    shortcutBar(0),
    console(new Console(teamSelector)),
    robotPool(0),
    splitter(new QSplitter(Qt::Vertical)),
    hSplitter(new QSplitter(Qt::Horizontal))
{
  splitter->addWidget(teamSelector);
  splitter->addWidget(console);
  splitter->setStretchFactor(0, 0);

  shortcutBar = new ShortcutBar(console);
  addToolBar(Qt::BottomToolBarArea, shortcutBar);
  shortcutBar->addShortcut("help", "help", "help");
  shortcutBar->addShortcut("ping", "ping", "network");
  shortcutBar->addShortcut("deploy", "deploy", "browser-download");
  shortcutBar->addShortcut("download logs", "downloadLogs", "download logs");

  hSplitter->addWidget(splitter);
  robotPool = new RobotPool(teamSelector);
  connect(teamSelector, SIGNAL(currentChanged(int)), robotPool, SLOT(update()));
  QFrame *rightSide = new QFrame(this);
  QGridLayout *rsLayout = new QGridLayout(rightSide);
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
  int widgetWidth = 1008;
  int widgetHeight = 600;
  splitter->setMinimumWidth(200);
  QWidget::setMinimumHeight(widgetHeight);
  QWidget::setMinimumWidth(widgetWidth);
  QWidget::resize(widgetWidth,widgetHeight);
  desktop = new QDesktopWidget();
  QPoint position((desktop->width() - frameGeometry().width()) / 2, (desktop->height() - frameGeometry().height()) / 2);
  QWidget::move(position);
}

MainWindow::~MainWindow()
{
  delete console;
  delete splitter;
  delete desktop;
}
