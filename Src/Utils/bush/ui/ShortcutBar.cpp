#include "Utils/bush/ui/ShortcutBar.h"
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"

#include <QIcon>
#include <QAction>

ShortcutBar::ShortcutBar(Console* console)
  : QToolBar("Shortcuts"),
    console(console)
{
  setObjectName("Shortcuts");
}

QAction* ShortcutBar::addShortcut(const QString& name,
                                  const QString& command)
{
  QAction* a = new QAction(name, this);
  a->setToolTip(name);
  a->setWhatsThis(command);
  a->setData(command);
  addAction(a);
  connect(a, SIGNAL(triggered()), this, SLOT(actionTriggered()));
  return a;
}

void ShortcutBar::actionTriggered()
{
  QObject* s = sender();
  if(s && s->inherits("QAction"))
  {
    QAction* a = dynamic_cast<QAction*>(s);
    QVariant data = a->data();
    if(data.canConvert(QVariant::String))
    {
      QString command = data.toString();
      if(command.length() > 0)
      {
        console->fireCommand(command);
      }
    }
  }
}
