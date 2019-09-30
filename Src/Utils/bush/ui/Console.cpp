#include <QApplication>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPalette>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollBar>
#include <QtConcurrent>
#include <QtCore>

#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/ui/CommandLineEdit.h"
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/ui/VisualContext.h"
#ifdef MACOS
#include "../Src/Controller/Visualization/Helper.h"
#endif

Icons Icons::theIcons;

void Icons::init()
{
  ICON_GRAY = QIcon(":icons/gray.png");
  ICON_GREEN = QIcon(":icons/green.png");
  ICON_ORANGE = QIcon(":icons/orange.png");
  ICON_RED = QIcon(":icons/red.png");
}

VisualContextDecoration::VisualContextDecoration(const QString& commandLine, VisualContext* parent, VisualContext* context)
  : QFrame(parent),
    button(new QPushButton(Icons::getInstance().ICON_GRAY, "", this)),
    header(new QLabel(commandLine)),
    visualContext(context),
    parentContext(parent)
{
  setAutoFillBackground(true);
  QPalette p = palette();
  p.setColor(QPalette::Background, p.color(QPalette::AlternateBase));
  setPalette(p);
  QFormLayout* layout = new QFormLayout();
  layout->setSpacing(3);
  header->setFrameStyle(QFrame::Box);
  layout->addRow(button, header);
  button->setMaximumWidth(25);
  button->setFlat(true);
  button->setCheckable(true);
  button->setChecked(true);
  button->setStyleSheet("background-color: rgba(255, 255, 255, 0);");
  layout->addRow(visualContext);
  setLayout(layout);

  connect(visualContext, SIGNAL(statusChanged(bool)), this, SLOT(updateStatus(bool)));
  connect(visualContext, SIGNAL(sCanceled()), this, SLOT(canceled()));
}

void VisualContextDecoration::updateStatus(bool status)
{
  if(button->icon().cacheKey() != Icons::getInstance().ICON_ORANGE.cacheKey())
  {
    if(status)
      button->setIcon(Icons::getInstance().ICON_GREEN);
    else
      button->setIcon(Icons::getInstance().ICON_RED);
  }
}

void VisualContextDecoration::canceled()
{
  button->setIcon(Icons::getInstance().ICON_ORANGE);
}

ScrollArea::ScrollArea(QWidget* parent)
  : QScrollArea(parent),
    scrollEnabled(true)
{
  connect(verticalScrollBar(), SIGNAL(valueChanged(int)), this, SLOT(updateScrollEnabled()));
}

bool ScrollArea::viewportEvent(QEvent* event)
{
  bool ret = QScrollArea::viewportEvent(event);
  if(event->type() == QEvent::LayoutRequest && widget() && scrollEnabled)
    ensureVisible(0, widget()->size().height());
  return ret;
}

void ScrollArea::updateScrollEnabled()
{
  scrollEnabled = verticalScrollBar()->value() == verticalScrollBar()->maximum();
}

Console::Console(TeamSelector* teamSelector)
  : visualContext(new VisualContext(this)),
    teamSelector(teamSelector),
    scrollArea(new ScrollArea(this)),
    cmdLine(0)
{
  cmdLine = new CommandLineEdit(this);
  QGridLayout* layout = new QGridLayout();
  layout->setHorizontalSpacing(0);
  scrollArea->setWidget(visualContext);
  scrollArea->setWidgetResizable(true);
  scrollArea->setBackgroundRole(QPalette::AlternateBase);
  layout->addWidget(scrollArea, 0, 0);
  layout->setRowStretch(0, 1);
  QFormLayout* fl = new QFormLayout();
  fl->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  fl->addRow(new QLabel("bush>", cmdLine), cmdLine);
  layout->addLayout(fl, 1, 0);
  setLayout(layout);

  connect(cmdLine, SIGNAL(returnPressed()), this, SLOT(returnPressed()));
}

void Console::returnPressed()
{
  fireCommand(cmdLine->text());
  cmdLine->setText("");
}

void Console::showEvent(QShowEvent* event)
{
  cmdLine->setFocus();
  QFrame::showEvent(event);
}

void Console::fireCommand(const QString& command)
{
  if(command.size() > 0)
  {
    // FixMe: This has to be done in the gui thread so it can't be inside of the DeleteLogsCmd, maybe there is another solution.
    if(command == "deleteLogs")
    {
      QMessageBox msgBox;
      msgBox.setWindowTitle("Delete Log Files");
      msgBox.setText("All log files on the selected robots will be lost.\nThis action cannot be undone!\n\nAre you sure?");
      msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
      msgBox.setDefaultButton(QMessageBox::Cancel);
      msgBox.setIcon(QMessageBox::Warning);

      if(msgBox.exec() != QMessageBox::Yes)
        return;
    }
    else if(command == "shutdown -s")
    {
      QMessageBox msgBox;
      msgBox.setWindowTitle("Shutdown");
      msgBox.setText("The selected robots will be shut down.\n\nAre you sure?");
      msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
      msgBox.setDefaultButton(QMessageBox::Cancel);
      msgBox.setIcon(QMessageBox::Warning);

      if(msgBox.exec() != QMessageBox::Yes)
        return;
    }

    QtConcurrent::run(visualContext, &VisualContext::executeInContext, this, teamSelector, command);
    cmdLine->setFocus();
  }
}

void Console::cancel()
{
  visualContext->cancel();
}

void Console::paintEvent(QPaintEvent* event)
{
#ifdef MACOS
  QPalette p = palette();
  p.setColor(QPalette::AlternateBase, getAlternateBase().color());
  setPalette(p);
#endif
  QFrame::paintEvent(event);
}
