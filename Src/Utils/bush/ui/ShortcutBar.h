#pragma once

#include <QToolBar>

class Console;
class QIcon;
class QAction;

class ShortcutBar : public QToolBar
{
  Q_OBJECT

  Console* console;

public:
  ShortcutBar(Console* console);
  QAction* addShortcut(const QString& name,
                       const QString& command);

private slots:
  void actionTriggered();
};
