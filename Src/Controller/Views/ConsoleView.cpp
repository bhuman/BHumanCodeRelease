/**
 * @file Controller/Views/ConsoleView.cpp
 * Implementation of class ConsoleView
 * @author Colin Graf
 */

#include <QScrollBar>
#include <QKeyEvent>
#include <QMenu>
#include <QSettings>
#include <QApplication>

#include "ConsoleView.h"
#include "Platform/BHAssert.h"
#include "Controller/ConsoleRoboCupCtrl.h"

SimRobot::Widget* ConsoleView::createWidget()
{
  ASSERT(!consoleWidget);
  consoleWidget = new ConsoleWidget(*this, console, output, consoleWidget);
  output.clear();
  return consoleWidget;
}

void ConsoleView::clear()
{
  output.clear();
  if(consoleWidget)
    consoleWidget->setPlainText(QString());
}

void ConsoleView::printLn(const QString& text)
{
  print(text + "\n");
}

void ConsoleView::print(const QString& text)
{
  if(consoleWidget)
    consoleWidget->print(text);
  else
    output += text;
}

ConsoleWidget::ConsoleWidget(ConsoleView& consoleView, ConsoleRoboCupCtrl& console, QString& output, ConsoleWidget*& consoleWidget) :
  consoleView(consoleView), console(console), output(output), consoleWidget(consoleWidget)
{
  setFrameStyle(QFrame::NoFrame);
  setAcceptRichText(false);

  bool restoredCursor = false;
  if(consoleView.loadAndSaveOutput)
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(consoleView.fullName);
    output = settings.value("Output").toString();
    setPlainText(output);
    output.clear();

    int selectionStart = settings.value("selectionStart").toInt();
    int selectionEnd = settings.value("selectionEnd").toInt();
    if(selectionStart || selectionEnd)
    {
      QTextCursor cursor = textCursor();
      cursor.setPosition(selectionStart);
      cursor.setPosition(selectionEnd, QTextCursor::KeepAnchor);
      setTextCursor(cursor);
      restoredCursor = true;
    }
    QScrollBar* scrollBar = verticalScrollBar();
    scrollBar->setMaximum(settings.value("verticalScrollMaximum").toInt());
    scrollBar->setValue(settings.value("verticalScrollPosition").toInt());

    settings.endGroup();
  }
  else
  {
    setPlainText(output);
    output.clear();
  }
  if(!restoredCursor)
  {
    QTextCursor cursor = textCursor();
    cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
    setTextCursor(cursor);
  }

  connect(this, SIGNAL(copyAvailable(bool)), this, SLOT(copyAvailable(bool)));
  connect(this, SIGNAL(undoAvailable(bool)), this, SLOT(undoAvailable(bool)));
  connect(this, SIGNAL(redoAvailable(bool)), this, SLOT(redoAvailable(bool)));
}

ConsoleWidget::~ConsoleWidget()
{
  output = toPlainText();
  consoleWidget = nullptr;

  if(consoleView.loadAndSaveOutput)
  {
    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(consoleView.fullName);
    settings.setValue("Output", output);

    QTextCursor cursor = textCursor();
    settings.setValue("selectionStart", cursor.anchor());
    settings.setValue("selectionEnd", cursor.position());
    QScrollBar* scrollBar = verticalScrollBar();
    settings.setValue("verticalScrollPosition", scrollBar->value());
    settings.setValue("verticalScrollMaximum", scrollBar->maximum());

    settings.endGroup();
    output.clear();
  }
}

void ConsoleWidget::print(const QString& text)
{
  QScrollBar* scrollBar = verticalScrollBar();
  bool scroll = scrollBar->value() == scrollBar->maximum();
  QTextCursor cursor = textCursor();
  cursor.movePosition(QTextCursor::StartOfBlock);
  /*
  cursor.movePosition(QTextCursor::NextBlock);
  if(cursor.atEnd() && !cursor.atStart())
  {
    cursor.insertText("\n");
    cursor.movePosition(QTextCursor::NextBlock);
  }*/
  cursor.insertText(text);
  if(scroll)
    scrollBar->setValue(scrollBar->maximum());
  cursor.movePosition(QTextCursor::End);
}

QMenu* ConsoleWidget::createEditMenu() const
{
  QMenu* menu = new QMenu(tr("&Edit"));

  QAction* action = menu->addAction(QIcon(":/Icons/arrow_undo.png"), tr("&Undo"));
  action->setShortcut(QKeySequence(QKeySequence::Undo));
  action->setStatusTip(tr("Undo the last action"));
  action->setEnabled(canUndo);
  connect(action, SIGNAL(triggered()), this, SLOT(undo()));
  connect(this, SIGNAL(undoAvailable(bool)), action, SLOT(setEnabled(bool)));

  action = menu->addAction(QIcon(":/Icons/arrow_redo.png"), tr("&Redo"));
  action->setShortcut(QKeySequence(QKeySequence::Redo));
  action->setStatusTip(tr("Redo the previously undone action"));
  action->setEnabled(canRedo);
  connect(action, SIGNAL(triggered()), this, SLOT(redo()));
  connect(this, SIGNAL(redoAvailable(bool)), action, SLOT(setEnabled(bool)));

  menu->addSeparator();

  action = menu->addAction(QIcon(":/Icons/cut.png"), tr("Cu&t"));
  action->setShortcut(QKeySequence(QKeySequence::Cut));
  action->setStatusTip(tr("Cut the current selection's contents to the clipboard"));
  action->setEnabled(canCopy);
  connect(action, SIGNAL(triggered()), this, SLOT(cut()));
  connect(this, SIGNAL(copyAvailable(bool)), action, SLOT(setEnabled(bool)));

  action = menu->addAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"));
  action->setShortcut(QKeySequence(QKeySequence::Copy));
  action->setStatusTip(tr("Copy the current selection's contents to the clipboard"));
  action->setEnabled(canCopy);
  connect(action, SIGNAL(triggered()), this, SLOT(copy()));
  connect(this, SIGNAL(copyAvailable(bool)), action, SLOT(setEnabled(bool)));

  action = menu->addAction(QIcon(":/Icons/page_paste.png"), tr("&Paste"));
  action->setShortcut(QKeySequence(QKeySequence::Paste));
  action->setStatusTip(tr("Paste the clipboard's contents into the current selection"));
  action->setEnabled(canPaste());
  connect(action, SIGNAL(triggered()), this, SLOT(paste()));
  connect(this, SIGNAL(pasteAvailable(bool)), action, SLOT(setEnabled(bool)));

  action = menu->addAction(tr("&Delete"));
  action->setShortcut(QKeySequence(QKeySequence::Delete));
  action->setStatusTip(tr("Delete the currently selected content"));
  action->setEnabled(canCopy);
  connect(action, SIGNAL(triggered()), this, SLOT(deleteText()));
  connect(this, SIGNAL(copyAvailable(bool)), action, SLOT(setEnabled(bool)));

  menu->addSeparator();

  action = menu->addAction(tr("Select &All"));
  action->setShortcut(QKeySequence(QKeySequence::SelectAll));
  action->setStatusTip(tr("Select the whole text"));
  connect(action, SIGNAL(triggered()), this, SLOT(selectAll()));

  return menu;
}

void ConsoleWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_Tab:
    case Qt::Key_Backtab:
      event->accept();
      {
        QTextCursor cursor = textCursor();
        int begin = cursor.position();
        int end = cursor.anchor();
        if(end < begin)
        {
          int tmp = end;
          end = begin;
          begin = tmp;
        }

        cursor.setPosition(begin);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
        QString line = cursor.selectedText();
        std::string command(line.toUtf8().constData());
        std::string inputCommand = command;
        console.completeConsoleCommand(command, event->key() == Qt::Key_Tab, begin == end);

        if(command == inputCommand)
          QApplication::beep();
        else
        {
          cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::MoveAnchor);
          cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
          QString fullLine = cursor.selectedText();
          std::string fullCommand(fullLine.toUtf8().constData());

          if(fullCommand == command) // There is only one possible completion -> move cursor so right arrow is unnecessary
          {
            cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::MoveAnchor);
          }
          else
          {
            cursor.insertText(command.c_str());
            cursor.setPosition(begin);
            cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
          }
          setTextCursor(cursor);
        }
      }
      break;

    case Qt::Key_Return:
    case Qt::Key_Enter:
      if(event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier))
      {
        QTextCursor cursor = textCursor();
        cursor.insertBlock();
        setTextCursor(cursor);
      }
      else
      {
        event->accept();
        {
          QTextCursor cursor = textCursor();
          cursor.movePosition(QTextCursor::StartOfBlock);
          cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
          QString line = cursor.selectedText();
          cursor.movePosition(QTextCursor::EndOfLine);
          if(cursor.atEnd())
            cursor.insertText("\n");
          cursor.movePosition(QTextCursor::NextBlock);
          setTextCursor(cursor);

          // save output for the case that the simulator crashes
          if(consoleView.loadAndSaveOutput)
          {
            QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
            settings.beginGroup(consoleView.fullName);
            settings.setValue("Output", toPlainText());
            settings.endGroup();
            output.clear();
          }

          // execute the command
          console.executeConsoleCommand(line.toUtf8().constData());

          // stores unix like history entry
          history.removeAll(line);
          history.append(line);
          history_iter = history.end();
        }
      }
      break;

    case Qt::Key_Right:
    {
      // avoid jumping to the next line when the right arrow key is used to accept suggestions from tab completion
      bool handled = false;
      if(event->modifiers() == 0)
      {
        QTextCursor cursor = textCursor();
        int position = cursor.position();
        if(position > cursor.anchor())
        {
          cursor.movePosition(QTextCursor::EndOfBlock);
          if(cursor.position() == position)
          {
            handled = true;
            setTextCursor(cursor);
          }
        }
      }
      if(!handled)
        QTextEdit::keyPressEvent(event);
    }
    break;

    // History browsing keys
    case Qt::Key_Up:
      if((event->modifiers() & Qt::ControlModifier)
         && !history.isEmpty() && history_iter != history.begin())
      {
        event->accept();
        history_iter--;
        QTextCursor cursor = textCursor();
        cursor.movePosition(QTextCursor::End);
        cursor.movePosition(QTextCursor::StartOfBlock, QTextCursor::KeepAnchor);
        cursor.removeSelectedText();
        cursor.insertText(*history_iter);
        setTextCursor(cursor);
      }
      else
      {
        QTextEdit::keyPressEvent(event);
      }
      break;
    case Qt::Key_Down:
      if(event->modifiers() & Qt::ControlModifier)
      {
        event->accept();
        QTextCursor cursor = textCursor();
        cursor.movePosition(QTextCursor::End);
        cursor.movePosition(QTextCursor::StartOfBlock, QTextCursor::KeepAnchor);
        cursor.removeSelectedText();
        if(!history.isEmpty() && history_iter != history.end())
        {
          history_iter++;
          if(history_iter != history.end())
          {
            cursor.insertText(*history_iter);
            setTextCursor(cursor);
          }
        }
      }
      else
      {
        QTextEdit::keyPressEvent(event);
      }
      break;
    default:

      QTextEdit::keyPressEvent(event);

      if(event->matches(QKeySequence::Copy) || event->matches(QKeySequence::Cut))
        emit pasteAvailable(canPaste());
      else if((!event->modifiers() || event->modifiers() & Qt::ShiftModifier) && event->key() >= Qt::Key_A && event->key() <= Qt::Key_Z)
      {
        QTextCursor cursor = textCursor();
        int begin = std::min(cursor.position(), cursor.anchor());
        cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
        QString line = cursor.selectedText();
        std::string command(line.toUtf8().constData());
        std::string inputCommand = command;
        console.completeConsoleCommandOnLetterEntry(command);
        if(command != inputCommand)
        {
          cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::MoveAnchor);
          cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
          cursor.insertText(command.c_str());

          cursor.setPosition(begin);
          cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);

          setTextCursor(cursor);
        }
      }
      break;
  }
}

void ConsoleWidget::contextMenuEvent(QContextMenuEvent* event)
{
  QWidget::contextMenuEvent(event);
}

void ConsoleWidget::focusInEvent(QFocusEvent* event)
{
  QTextEdit::focusInEvent(event);
  emit pasteAvailable(canPaste());
}

void ConsoleWidget::copyAvailable(bool available)
{
  canCopy = available;
}

void ConsoleWidget::redoAvailable(bool available)
{
  canRedo = available;
}

void ConsoleWidget::undoAvailable(bool available)
{
  canUndo = available;
}

void ConsoleWidget::cut()
{
  QTextEdit::cut();
  emit pasteAvailable(canPaste());
}

void ConsoleWidget::copy()
{
  QTextEdit::copy();
  emit pasteAvailable(canPaste());
}

void ConsoleWidget::deleteText()
{
  insertPlainText(QString());
}
