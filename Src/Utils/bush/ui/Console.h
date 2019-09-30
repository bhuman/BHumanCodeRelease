#pragma once

#include <QFrame>
#include <QListWidget>
#include <QScrollArea>
#include <QVariant>
#include "Utils/bush/cmdlib/AbstractConsole.h"

class TeamSelector;
class QLabel;
class Context;
class QFormLayout;

class CommandLineEdit;
class Console;
class VisualContext;

class Icons
{
  static Icons theIcons;
public:
  QIcon ICON_GRAY;
  QIcon ICON_GREEN;
  QIcon ICON_ORANGE;
  QIcon ICON_RED;
  static Icons& getInstance() { return theIcons; }
  void init();
};

class ScrollArea : public QScrollArea
{
  Q_OBJECT

  bool scrollEnabled;

public:
  ScrollArea(QWidget* parent);

  bool viewportEvent(QEvent* event);

public slots:
  void updateScrollEnabled();
};

class Console : public QFrame
{
  Q_OBJECT

  /** The root visualContext.
   * It does not really display output but contains all other visual
   * representations of further contexts.
   */
  VisualContext* visualContext;

  /** The teamSelector which knows which robots and which team are selected. */
  TeamSelector* teamSelector;

  ScrollArea* scrollArea;

  /** The thing where commands can be typed in. */
  CommandLineEdit* cmdLine;

public:
  Console(TeamSelector* teamSelector);
  QSize minimumSizeHint() const override { return QSize(100, 250); }

  /** Is executed when the console is shown and sets the focus to the
   * commandLineEdit so that the user can just start to type.
   */
  void showEvent(QShowEvent* event) override;

  void fireCommand(const QString& command);
  void cancel();

protected:
  void paintEvent(QPaintEvent* event) override;

protected slots:
  /** Starts a new thread which executes the entered command. */
  void returnPressed();
};
