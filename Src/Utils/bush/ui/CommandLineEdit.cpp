/**
 * @file CommandLineEdit.cpp Contains the implementation CommandLineEdit which is in
 * charge to present the user the ability to type some commands into a singele
 * line.
 *
 * @author <a href="ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#include "Utils/bush/ui/CommandLineEdit.h"
#include "Utils/bush/ui/CommandLineCompleter.h"
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/cmdlib/Commands.h"

#include <QKeyEvent>


bool CommandLineEdit::eventFilter(QObject* o, QEvent* e)
{
  if(e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease)
  {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(e);
    if(keyEvent->key() == Qt::Key_Tab && e->type() == QEvent::KeyPress)
    {
      if(completer->popup()->isVisible())
        completer->setCurrentRow(completer->currentRow() + 1);
      else
        complete();
      return true;
    }
  }
  return QLineEdit::eventFilter(o, e);
}

void CommandLineEdit::complete()
{
  completer->complete();
  /* Grab the focus so that the completer popup cannot take it with it into its grave. */
  setFocus();
}

CommandLineEdit::CommandLineEdit(Console* parent)
  : QLineEdit(parent),
    console(parent),
    history(),
    historyIter(history.end())
{
  setFrame(false);
  setAutoFillBackground(true);

  installEventFilter(this);

  completer = new CommandLineCompleter(this);
  completer->setWidget(this);
  connect(completer, SIGNAL(activated(const QString&)), this, SLOT(setText(const QString&)));
  connect(completer, SIGNAL(highlighted(const QString&)), this, SLOT(setText(const QString&)));
}

void CommandLineEdit::keyPressEvent(QKeyEvent* e)
{
  // ignore some special keys so that the completer can handle them
  if(completer->popup()->isVisible())
  {
    switch(e->key())
    {
      case Qt::Key_Enter:
      case Qt::Key_Return:
      case Qt::Key_Escape:
        completer->popup()->hide();
        setFocus();
        break;
      case Qt::Key_Tab:
      case Qt::Key_Backtab:
        e->accept();
        return;
    }
  }

  if(e->modifiers() & Qt::ControlModifier && e->key() == Qt::Key_D)
  {
    console->cancel();
    e->accept();
    return;
  }

  switch(e->key())
  {
    case Qt::Key_Up:        // history up
      if(!history.isEmpty() && historyIter != history.begin())
      {
        setText(*--historyIter);
        completer->popup()->hide();
      }
      break;
    case Qt::Key_Down:      // history down
      if(!history.isEmpty() && historyIter != history.end())
      {
        ++historyIter;
        if(!history.isEmpty() && historyIter != history.end())
        {
          setText(*historyIter);
          completer->popup()->hide();
        }
      }
      break;
    case Qt::Key_Enter:     // return: update history (rest is done by the console)
    case Qt::Key_Space:
    case Qt::Key_Return:
    {
      QString line = text();
      if(line.size() > 0)
      {
        history.removeAll(line);
        history.append(line);
        historyIter = history.end();
      }
      completer->setCompletionPrefix("");
      QLineEdit::keyPressEvent(e);
      break;
    }
    default:
    {
      historyIter = history.end();
      QLineEdit::keyPressEvent(e);

      /* This updates the completion prefix and assures that the completer
       * stays visible if we type more keys. Furthermore complete is called
       * which corrects the focus.
       */
      bool completerVisible = completer->popup()->isVisible();
      completer->setCompletionPrefix(text());
      if(completerVisible)
        complete();
    }
  }
  e->accept();
}
