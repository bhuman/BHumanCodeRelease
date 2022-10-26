#include <QApplication>
#include <QLabel>
#include <QtCore>
#include <QTextDocument>
#include "cmdlib/Context.h"
#include "ui/Console.h"
#include "ui/Icons.h"
#include "ui/TeamSelector.h"
#include "ui/VisualContext.h"

VisualContext::VisualContext(QWidget* parent, const Icons& icons) :
  QFrame(parent),
  theIcons(icons),
  formLayout(new QFormLayout())
{
  if(parent && parent->inherits("VisualContext"))
    setFrameStyle(QFrame::Box);
  formLayout->setSpacing(1);
  formLayout->setContentsMargins(1, 1, 1, 1);
  setLayout(formLayout);
  setAutoFillBackground(true);
  QPalette p = palette();
  p.setColor(QPalette::Window, p.color(QPalette::AlternateBase));
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
  VisualContext* sub = new VisualContext(this, theIcons);

  connect(context, &Context::sExecute,
          sub, &VisualContext::commandExecuted,
          Qt::BlockingQueuedConnection);
  connect(context, &Context::sPrint,
          sub, &VisualContext::doPrint);
  connect(context, &Context::sFinished, sub, &VisualContext::commandFinished);
  connect(context, &Context::sCancelFinished, sub, &VisualContext::commandCanceled);
  connect(sub, &VisualContext::sCancel, context, &Context::cancel, Qt::DirectConnection);

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

void VisualContext::executeInContext(TeamSelector* teamSelector, const CommandBase* cmd, const CommandArgs* args)
{
  Context* context = new Context(teamSelector->getSelectedRobots(),
                                 teamSelector->getSelectedTeam());

  /* Important: Use BlockingQueuedConnection so that the newly created thread
   * cannot emits all its print signals before they can be handled. */
  connect(context, &Context::sExecute,
          this, &VisualContext::commandExecuted,
          Qt::BlockingQueuedConnection);
  connect(context, &Context::sCancelFinished, this, &VisualContext::commandCanceled);

  /* Don't know if this signal is emitted a bit too often but this assures that
   * the newest output of the current command is always visible. */
  connect(this, &VisualContext::sCancel, context, &Context::cancel, Qt::DirectConnection);
  context->execute(cmd, args);

  /* Do not forget to disconnect the signal so that a new command can trigger
   * the scrolling. */
  disconnect(context);

  // Since the context is a QObject it is better to let Qt do the cleanup
  context->deleteLater();
}

VisualContextDecoration::VisualContextDecoration(const QString& commandLine, VisualContext* parent, VisualContext* context)
  : QFrame(parent),
    button(new QPushButton(parent->theIcons.gray, "", this)),
    header(new QLabel(commandLine)),
    visualContext(context),
    parentContext(parent)
{
  setAutoFillBackground(true);
  QPalette p = palette();
  p.setColor(QPalette::Window, p.color(QPalette::AlternateBase));
  setPalette(p);
  QFormLayout* layout = new QFormLayout();
  layout->setSpacing(3);
  header->setFrameStyle(QFrame::Box);
  layout->addRow(button, header);
  button->setMaximumWidth(25);
  button->setFlat(true);
  button->setStyleSheet("background-color: rgba(255, 255, 255, 0);");
  connect(button, &QPushButton::pressed, visualContext, &VisualContext::cancel);
  layout->addRow(visualContext);
  setLayout(layout);

  connect(visualContext, &VisualContext::statusChanged, this, &VisualContextDecoration::updateStatus);
  connect(visualContext, &VisualContext::sCanceled, this, &VisualContextDecoration::canceled);
}

void VisualContextDecoration::updateStatus(bool status)
{
  if(button->icon().cacheKey() != parentContext->theIcons.orange.cacheKey())
  {
    if(status)
      button->setIcon(parentContext->theIcons.green);
    else
      button->setIcon(parentContext->theIcons.red);
  }
}

void VisualContextDecoration::canceled()
{
  button->setIcon(parentContext->theIcons.orange);
}
