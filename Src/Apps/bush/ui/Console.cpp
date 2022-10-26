#include "Console.h"
#include "ui/ScrollArea.h"
#include "ui/VisualContext.h"
#ifdef MACOS
#include "AppleHelper/Helper.h"
#endif
#include <QGridLayout>
#include <QPalette>
#include <QtConcurrent>

Console::Console(TeamSelector* teamSelector) :
  visualContext(new VisualContext(this, theIcons)),
  teamSelector(teamSelector),
  scrollArea(new ScrollArea(this))
{
  QGridLayout* layout = new QGridLayout();
  layout->setHorizontalSpacing(0);
  scrollArea->setWidget(visualContext);
  scrollArea->setWidgetResizable(true);
  scrollArea->setBackgroundRole(QPalette::AlternateBase);
  layout->addWidget(scrollArea, 0, 0);
  layout->setRowStretch(0, 1);
  setLayout(layout);
}

void Console::fireCommand(const CommandBase* cmd, const CommandArgs* args)
{
  static_cast<void>(QtConcurrent::run(&VisualContext::executeInContext, visualContext, teamSelector, cmd, args));
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
