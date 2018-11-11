/**
 * @file CommandLineCompleter.cpp Contains the implementations of
 * CommandLineCompleter and TabFilter.
 *
 * @author <a href="ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#include <QAbstractItemView>
#include <QStringListModel>
#include <QKeyEvent>
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/ui/CommandLineCompleter.h"

static const size_t QLINEEDIT_MAX_LENGTH = 32767;

static size_t countWords(const std::string& str)
{
  size_t c = split(str).size();
  if(!str.empty() && *(--str.end()) == ' ')
    ++c;
  return c;
}

TabFilter::TabFilter(QObject* parent)
  : QObject(parent)
{}

bool TabFilter::eventFilter(QObject* o, QEvent* e)
{
  if(e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease)
  {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(e);
    if(keyEvent->key() == Qt::Key_Tab && e->type() == QEvent::KeyPress)
    {
      return false;
    }
  }
  return QObject::eventFilter(o, e);
}

CommandLineCompleter::CommandLineCompleter(QObject* parent)
  : QCompleter(parent),
    wordCount(QLINEEDIT_MAX_LENGTH),
    model(new QStringListModel(this))
{
  popup()->installEventFilter(new TabFilter(this));
  setModel(model);
}

void CommandLineCompleter::setCompletionPrefix(const QString& text)
{
  size_t wc = countWords(toString(text));
  if(wc != wordCount)
  {
    QStringList strings;
    for(const std::string& s : Commands::getInstance().complete(toString(text)))
    {
      strings << fromString(s);
    }

    model->setStringList(strings);
  }

  wordCount = wc;
  QCompleter::setCompletionPrefix(text);
}
