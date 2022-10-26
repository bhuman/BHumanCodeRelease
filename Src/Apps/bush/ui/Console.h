#pragma once

#include "ui/Icons.h"
#include <QFrame>

struct CommandArgs;
class CommandBase;
class QPaintEvent;
class ScrollArea;
class TeamSelector;
class VisualContext;

class Console : public QFrame
{
  Q_OBJECT

  Icons theIcons;

  /** The root visualContext.
   * It does not really display output but contains all other visual
   * representations of further contexts.
   */
  VisualContext* visualContext;

  /** The teamSelector which knows which robots and which team are selected. */
  TeamSelector* teamSelector;

  ScrollArea* scrollArea;

public:
  Console(TeamSelector* teamSelector);
  QSize minimumSizeHint() const override { return QSize(100, 250); }

  void fireCommand(const CommandBase* cmd, const CommandArgs* args);

protected:
  void paintEvent(QPaintEvent* event) override;
};
