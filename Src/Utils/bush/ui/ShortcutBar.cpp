#include "Utils/bush/ui/ShortcutBar.h"
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"

#include <QIcon>
#include <QAction>

ShortcutBar::ShortcutBar(Console *console)
  : QToolBar("Shortcuts"),
    console(console)
{ }

QAction* ShortcutBar::addShortcut(const QString &name,
                                  const QString &command,
                                  QIcon *icon)
{
  QIcon _icon;
  if (icon)
    _icon = *icon;
  else
    _icon = QIcon::fromTheme("player_play");
  QAction *a = new QAction(_icon, name, this);
  a->setToolTip(name);
  a->setWhatsThis(command);
  a->setData(command);
  addAction(a);
  connect(a, SIGNAL(triggered()), this, SLOT(actionTriggered()));
  return a;
}

QAction* ShortcutBar::addShortcut(const QString &name,
                                  const QString &command,
                                  const QString &icon)
{
  QIcon _icon = QIcon::fromTheme(icon);
  return addShortcut(name, command, &_icon);
}

void ShortcutBar::actionTriggered()
{
  QObject *s = sender();
  if (s && s->inherits("QAction"))
  {
    QAction *a = dynamic_cast<QAction*>(s);
    QVariant data = a->data();
    if (data.canConvert(QVariant::String))
    {
      QString command = data.toString();
      if (command.length() > 0)
      {
        console->fireCommand(command);
      }
    }
  }
}
