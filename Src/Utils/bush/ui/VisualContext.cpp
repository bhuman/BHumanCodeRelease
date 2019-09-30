#include <QApplication>
#include <QLabel>
#include <QtCore>
#include <QTextDocument>
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/ui/VisualContext.h"

VisualContext::VisualContext(QWidget* parent)
  : QFrame(parent),
    entries(),
    widgets(),
    formLayout(new QFormLayout()),
    nl(true)
{
  if(parent && parent->inherits("VisualContext"))
    setFrameStyle(QFrame::Box);
  formLayout->setSpacing(1);
  formLayout->setContentsMargins(1, 1, 1, 1);
  setLayout(formLayout);
  setAutoFillBackground(true);
  QPalette p = palette();
  p.setColor(QPalette::Background, p.color(QPalette::AlternateBase));
  setPalette(p);
  setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));
}

VisualContext::Entry::~Entry()
{
  // do not delete context. It takes care of itself.
  if(type != CONTEXT && text) delete text;
}

static inline VisualContext::Entry::Type targetToType(ConsolePrintTarget target)
{
  return target == CPT_PRINT || target == CPT_PRINT_LINE ? VisualContext::Entry::TEXT_OUTPUT : VisualContext::Entry::TEXT_ERROR;
}

void VisualContext::updateWidget(size_t index, Entry* entry)
{
  QWidget* widget = widgets[static_cast<int>(index)];
  if(widget->inherits("QLabel"))
  {
    QLabel* label = dynamic_cast<QLabel*>(widget);
    QString text;
    if(entry->type == Entry::TEXT_ERROR)
      text = "<font color='red'>" + Qt::convertFromPlainText(*entry->text) + "</font>";
    else
      text = Qt::convertFromPlainText(*entry->text);
    label->setText(text);
  }
}

void VisualContext::addWidget(Entry* entry, const QString& commandLine)
{
  QWidget* widget;
  if(entry->type == Entry::CONTEXT)
  {
    VisualContextDecoration* vd = new VisualContextDecoration(commandLine, this, entry->context);
    widget = vd;
  }
  else
  {
    QString text;
    if(entry->type == Entry::TEXT_ERROR)
      text = "<font color='red'>" + Qt::convertFromPlainText(*entry->text) + "</font>";
    else
      text = Qt::convertFromPlainText(*entry->text);

    QLabel* label = new QLabel(text, this);
    label->setTextInteractionFlags(label->textInteractionFlags() | Qt::TextSelectableByMouse);
    label->setMargin(1);
    widget = label;
  }
  QFormLayout* l = getLayout();
  l->addRow(widget);
  widget->setAutoFillBackground(true);
  widgets << widget;
}

void VisualContext::doPrint(ConsolePrintTarget target, const QString& msg)
{
  Entry::Type currentType = targetToType(target);
  Entry* lastEntry = entries.empty() ? 0 : entries.last();
  if(lastEntry && lastEntry->type == currentType)
  {
    if(nl)
      lastEntry->text->append("\n");
    lastEntry->text->append(msg);
    updateWidget(entries.size() - 1, lastEntry);
  }
  else
  {
    Entry* entry = new Entry(currentType, msg);
    entries << entry;
    addWidget(entry);
  }
  nl = target == CPT_ERROR_LINE || target == CPT_PRINT_LINE;
}

void VisualContext::commandExecuted(Context* context, const QString& cmdLine)
{
  VisualContext* sub = new VisualContext(this);

  connect(context, SIGNAL(sExecute(Context*, const QString&)),
          sub, SLOT(commandExecuted(Context*, const QString&)),
          Qt::BlockingQueuedConnection);
  connect(context, SIGNAL(sPrint(ConsolePrintTarget, const QString&)),
          sub, SLOT(doPrint(ConsolePrintTarget, const QString&)));
  connect(context, SIGNAL(sFinished(bool)), sub, SLOT(commandFinished(bool)));
  connect(context, SIGNAL(sCancelFinished()), sub, SLOT(commandCanceled()));
  connect(sub, SIGNAL(sCancel()), context, SLOT(cancel()), Qt::DirectConnection);

  Entry* entry = new Entry(sub);
  entries << entry;
  addWidget(entry, cmdLine);
}

void VisualContext::commandFinished(bool status)
{
  emit statusChanged(status);
}

void VisualContext::commandCanceled()
{
  emit sCanceled();
}

void VisualContext::cancel()
{
  emit sCancel();
}

void VisualContext::executeInContext(Console* console, TeamSelector* teamSelector, const QString& cmdLine)
{
  Context* context = new Context(teamSelector->getSelectedRobots(),
                                 teamSelector->getSelectedTeam());

  /* Important: Use BlockingQueuedConnection so that the newly created thread
   * cannot emits all its print signals before they can be handled. */
  connect(context, SIGNAL(sExecute(Context*, const QString&)),
          this, SLOT(commandExecuted(Context*, const QString&)),
          Qt::BlockingQueuedConnection);
  connect(context, SIGNAL(sCancelFinished()), this, SLOT(commandCanceled()));

  /* Don't know if this signal is emitted a bit too often but this assures that
   * the newest output of the current command is always visible. */
  connect(this, SIGNAL(sCancel()), context, SLOT(cancel()), Qt::DirectConnection);
  context->execute(toString(cmdLine));

  /* Do not forget to disconnect the signal so that a new command can trigger
   * the scrolling. */
  disconnect(context);

  bool doQuit = context->isShutdown();
  // Since the context is a QObject it is better to let Qt do the cleanup
  context->deleteLater();

  if(doQuit)
    QApplication::quit();
}
