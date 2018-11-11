/**
* @file SimRobotEditor/EditorWidget.cpp
* Implementation of class EditorWidget
* @author Colin Graf
*/

#include <QMenu>
#include <QRegularExpression>
#include <QSet>
#include <QFileInfo>
#include <QTextStream>
#include <QMessageBox>
#include <QSettings>
#include <QScrollBar>
#include <QResizeEvent>

#include "EditorModule.h"
#include "EditorWidget.h"

EditorObject::EditorObject(const QString& name, EditorObject* parent) :
  parent(parent), name(name), fullName(parent ? parent->fullName + "." + name : name) {}

const QIcon* EditorObject::getIcon() const
{
  return &EditorModule::module->folderIcon;
}

SimRobotEditor::Editor* EditorObject::addFile(const QString& filePath, const QString& subFileRegExpPattern)
{
  return addEditor(filePath, subFileRegExpPattern, true);
}

SimRobotEditor::Editor* EditorObject::addEditor(const QString& filePath, const QString& subFileRegExpPattern, bool persistent)
{
  FileEditorObject* editor = EditorModule::module->findEditor(filePath);
  if(editor)
  {
    if(persistent)
      editor->persistent = true;
    return editor;
  }
  editor = new FileEditorObject(filePath, subFileRegExpPattern, persistent, this);
  editors.append(editor);
  EditorModule::module->registerEditor(editor);
  EditorModule::application->registerObject(*EditorModule::module, *editor, this);
  return editor;
}

SimRobotEditor::Editor* EditorObject::addFolder(const QString& name)
{
  EditorObject* folder = foldersByName.value(name);
  if(folder)
    return folder;
  folder = new EditorObject(name, this);
  foldersByName.insert(name, folder);
  editors.append(folder);
  EditorModule::application->registerObject(*EditorModule::module, *folder, this, SimRobot::Flag::windowless);
  return folder;
}

void EditorObject::removeEditor(FileEditorObject* editor)
{
  if(editor->persistent)
    return;

  if(!editors.removeOne(editor))
    return;

  EditorModule::application->unregisterObject(*editor);
  EditorModule::module->unregisterEditor(editor);
  delete editor;
}

void EditorObject::loadFromSettings()
{
  QSettings& settings = EditorModule::application->getLayoutSettings();
  int count = 1;
  for(int i = 0; i < count; ++i)
  {
    count = settings.beginReadArray(fullName);
    if(i >= count)
    {
      settings.endArray();
      break;
    }
    settings.setArrayIndex(i);
    QString filePath = settings.value("filePath").toString();
    if(filePath.isEmpty())
    {
      QString name = settings.value("name").toString();
      settings.endArray();
      ((EditorObject*)addFolder(name))->loadFromSettings();
    }
    else
    {
      QString subFileRegExpPattern = settings.value("subFileRegExpPattern").toString();
      settings.endArray();
      ((EditorObject*)addEditor(filePath, subFileRegExpPattern, false))->loadFromSettings();
    }
  }
}

EditorObject::~EditorObject()
{
  QSettings& settings = EditorModule::application->getLayoutSettings();
  settings.beginWriteArray(fullName, editors.size());
  int i = 0;
  for(const EditorObject* editor : editors)
  {
    settings.setArrayIndex(i++);
    const FileEditorObject* fileEditorObject = dynamic_cast<const FileEditorObject*>(editor);
    if(!fileEditorObject)
    {
      settings.setValue("filePath", QString());
      settings.setValue("name", editor->name);
    }
    else
    {
      settings.setValue("filePath", fileEditorObject->filePath);
      settings.setValue("subFileRegExpPattern", fileEditorObject->subFileRegExpPattern);
    }
  }
  settings.endArray();
  qDeleteAll(editors);
}

FileEditorObject::FileEditorObject(const QString& filePath, const QString& subFileRegExpPattern, bool persistent, EditorObject* parent) :
  EditorObject(QFileInfo(filePath).fileName(), parent), filePath(filePath), subFileRegExpPattern(subFileRegExpPattern), persistent(persistent) {}

const QIcon* FileEditorObject::getIcon() const
{
  return &EditorModule::module->fileIcon;
}

SimRobot::Widget* FileEditorObject::createWidget()
{
  QFile file(filePath);
  if(!file.open(QFile::ReadOnly | QFile::Text))
  {
    EditorModule::application->showWarning(QObject::tr("SimRobotEditor"), QObject::tr("Cannot read file %1:\n%2.").arg(filePath).arg(file.errorString()));
    return nullptr;
  }
  QTextStream in(&file);
  return new EditorWidget(this, in.readAll());
}

EditorWidget::EditorWidget(FileEditorObject* editorObject, const QString& fileContent) :
  editorObject(editorObject),
  canCopy(false), canUndo(false), canRedo(false),
  highlighter(0)
{
  if(editorObject->filePath.endsWith(".ros2") || editorObject->filePath.endsWith(".rsi2"))
    highlighter = new SyntaxHighlighter(document());
  setFrameStyle(QFrame::NoFrame);

#ifdef WINDOWS
  QFont font("Courier New", 10);
#elif defined MACOS
  QFont font("Monaco", 11);
#else
  QFont font("Bitstream Vera Sans Mono", 9);
#endif
  setFont(font);
  setLineWrapMode(QTextEdit::NoWrap);
  setAcceptRichText(false);
  setPlainText(fileContent);
  document()->setModified(false);

  QSettings& settings = EditorModule::application->getLayoutSettings();
  settings.beginGroup(editorObject->fullName);
  int selectionStart = settings.value("selectionStart").toInt();
  int selectionEnd = settings.value("selectionEnd").toInt();
  if(selectionStart || selectionEnd)
  {
    QTextCursor cursor = textCursor();
    cursor.setPosition(selectionStart);
    cursor.setPosition(selectionEnd, QTextCursor::KeepAnchor);
    setTextCursor(cursor);
  }
  verticalScrollBar()->setValue(settings.value("verticalScrollPosition").toInt());
  horizontalScrollBar()->setValue(settings.value("horizontalScrollPosition").toInt());
  useTabStop = settings.value("useTabStop", false).toBool();
  tabStopWidth = settings.value("tabStopWidth", 2).toInt();
  settings.endGroup();

  setTabStopWidth(tabStopWidth * QFontMetrics(font).width(' '));

  connect(this, SIGNAL(copyAvailable(bool)), this, SLOT(copyAvailable(bool)));
  connect(this, SIGNAL(undoAvailable(bool)), this, SLOT(undoAvailable(bool)));
  connect(this, SIGNAL(redoAvailable(bool)), this, SLOT(redoAvailable(bool)));
  connect(&openFileMapper, SIGNAL(mapped(const QString&)), this, SLOT(openFile(const QString&)));
}

EditorWidget::~EditorWidget()
{
  QSettings& settings = EditorModule::application->getLayoutSettings();
  settings.beginGroup(editorObject->fullName);
  QTextCursor cursor = textCursor();
  settings.setValue("selectionStart", cursor.anchor());
  settings.setValue("selectionEnd", cursor.position());
  settings.setValue("verticalScrollPosition", verticalScrollBar()->value());
  settings.setValue("horizontalScrollPosition", horizontalScrollBar()->value());
  settings.setValue("useTabStop", useTabStop);
  settings.setValue("tabStopWidth", tabStopWidth);
  settings.endGroup();

  if(!EditorModule::module->application->getFilePath().isEmpty()) // !closingDocument
    editorObject->parent->removeEditor(editorObject);

  if(highlighter)
    delete highlighter;
}

bool EditorWidget::canClose()
{
  if(!document()->isModified())
    return true;
  switch(QMessageBox::warning(this, tr("SimRobotEditor"), tr("Do you want to save changes to %1?").arg(editorObject->name), QMessageBox::Save  | QMessageBox::Discard | QMessageBox::Cancel))
  {
    case QMessageBox::Save:
      save();
      break;
    case QMessageBox::Discard:
      break;
    default:
      return false;
  }
  return true;
}

QMenu* EditorWidget::createFileMenu() const
{
  QMenu* menu = new QMenu(tr("&File"));

  QAction* action = menu->addAction(QIcon(":/Icons/disk.png"), tr("&Save"));
  action->setShortcut(QKeySequence(QKeySequence::Save));
  action->setStatusTip(tr("Save the document to disk"));
  action->setEnabled(document()->isModified());
  connect(action, SIGNAL(triggered()), this, SLOT(save()));
  connect(document(), SIGNAL(modificationChanged(bool)), action, SLOT(setEnabled(bool)));

  return menu;
}

QMenu* EditorWidget::createEditMenu() const
{
  QMenu* menu = new QMenu(tr("&Edit"));
  connect(menu, SIGNAL(aboutToShow()), this, SLOT(updateEditMenu()));
  updateEditMenu(menu, false);
  return menu;
}

void EditorWidget::updateEditMenu(QMenu* menu, bool aboutToShow) const
{
  menu->clear();

  if(aboutToShow && !editorObject->subFileRegExpPattern.isEmpty())
  {
    QRegularExpression rx(editorObject->subFileRegExpPattern, QRegularExpression::CaseInsensitiveOption);
    QString fileContent = toPlainText();
    QStringList includeFiles;
    QSet<QString> inculdeFilesSet;
    QString suffix = QFileInfo(editorObject->name).suffix();
    QRegularExpressionMatch match;
    int pos = 0;
    while((match = rx.match(fileContent, pos)).hasMatch())
    {
      QString file = match.captured(1).remove('\"');
      if(QFileInfo(file).suffix().isEmpty())
        (file += '.') += suffix;
      if(!inculdeFilesSet.contains(file))
      {
        includeFiles.append(file);
        inculdeFilesSet.insert(file);
      }
      pos = match.capturedEnd();
    }

    if(includeFiles.count() > 0)
    {
      for(const QString& str : includeFiles)
      {
        QAction* action = menu->addAction(tr("Open \"%1\"").arg(str));
        openFileMapper.setMapping(action, str);
        connect(action, SIGNAL(triggered()), &openFileMapper, SLOT(map()));
      }
      menu->addSeparator();
    }
  }

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
  action->setStatusTip(tr("Select the whole document"));
  connect(action, SIGNAL(triggered()), this, SLOT(selectAll()));
}

void EditorWidget::focusInEvent(QFocusEvent * event)
{
  QTextEdit::focusInEvent(event);
  emit pasteAvailable(canPaste());
}

void EditorWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_Tab:
    case Qt::Key_Backtab:
      event->accept();
      {
        QTextCursor cursor = textCursor();
        if(event->key() == Qt::Key_Tab && cursor.position() == cursor.anchor())
        {
          if(useTabStop)
            cursor.insertText("\t");
          else
          {
            cursor.beginEditBlock();
            const int position = cursor.position();
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
            const int diff = position - cursor.position();
            cursor.setPosition(position, QTextCursor::MoveAnchor);
            cursor.insertText(QString().fill(' ', tabStopWidth - (diff % tabStopWidth)));
            cursor.endEditBlock();
          }
        }
        else
        {
          int anchor = cursor.anchor();
          int position = cursor.position();

          int delta; // The number of characters that have been added / removed in a line

          cursor.beginEditBlock();
          cursor.setPosition(std::min(anchor, position), QTextCursor::MoveAnchor);
          cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
          do
          {
            const int insertionPosition = cursor.position();
            if(event->key() == Qt::Key_Tab)
            {
              cursor.insertText(useTabStop ? "\t" : QString().fill(' ', tabStopWidth));
              delta = useTabStop ? 1 : tabStopWidth;
            }
            else
            {
              cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
              QString line = cursor.selectedText();
              if(line[0] == '\t')
              {
                cursor.insertText(line.mid(1));
                delta = -1;
              }
              else
              {
                int width;
                for(width = 0; width < line.length() && width < tabStopWidth; ++width)
                  if(line[width] != ' ')
                    break;
                cursor.insertText(line.mid(width));
                delta = -width;
              }
            }
            // Adjust the original selection.
            // When unindenting, it must not happen that the cursor moves to a line above.
            if(insertionPosition <= anchor)
            {
              anchor += delta;
              if(anchor < insertionPosition)
                anchor = insertionPosition;
            }
            if(insertionPosition <= position)
            {
              position += delta;
              if(position < insertionPosition)
                position = insertionPosition;
            }
            // Continue with the next line.
            cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor);
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
            // Check if the last line has been reached.
            if(cursor.position() == insertionPosition)
              break;
          }
          while(cursor.position() < std::max(anchor, position));
          // Restore the original selection.
          cursor.setPosition(anchor, QTextCursor::MoveAnchor);
          cursor.setPosition(position, QTextCursor::KeepAnchor);
          cursor.endEditBlock();
        }
        setTextCursor(cursor);
      }
      break;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      event->accept();
      {
        QTextCursor cursor = textCursor();
        cursor.beginEditBlock();
        // Actually insert the new line.
        cursor.insertText("\n");
        // Find out how the line above was indented.
        cursor.movePosition(QTextCursor::Up, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
        QString indentation = cursor.selectedText();
        for(int i = 0; i < indentation.length(); ++i)
          if(indentation[i] != ' ' && indentation[i] != '\t')
          {
            indentation.truncate(i);
            break;
          }
        // Insert the indentation.
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor);
        cursor.insertText(indentation);
        cursor.endEditBlock();
        setTextCursor(cursor);
      }
      break;
    default:
      QTextEdit::keyPressEvent(event);
      if(event->matches(QKeySequence::Copy) || event->matches(QKeySequence::Cut))
        emit pasteAvailable(canPaste());
  }
}

void EditorWidget::contextMenuEvent(QContextMenuEvent* event)
{ // disable QTextEdit context menu to use our own
  QWidget::contextMenuEvent(event);
}

void EditorWidget::updateEditMenu()
{
  QMenu* menu = qobject_cast<QMenu*>(sender());
  updateEditMenu(menu, true);
}

void EditorWidget::copyAvailable(bool available)
{
  canCopy = available;
}

void EditorWidget::redoAvailable(bool available)
{
  canRedo = available;
}

void EditorWidget::undoAvailable(bool available)
{
  canUndo = available;
}

void EditorWidget::save()
{
  QFile file(editorObject->filePath);
  if(!file.open(QFile::WriteOnly | QFile::Text))
  {
    EditorModule::application->showWarning(QObject::tr("SimRobotEditor"), QObject::tr("Cannot write file %1:\n%2.").arg(editorObject->filePath).arg(file.errorString()));
    return;
  }
  QTextStream out(&file);
  out << toPlainText();
  document()->setModified(false);
}

void EditorWidget::cut()
{
  QTextEdit::cut();
  emit pasteAvailable(canPaste());
}

void EditorWidget::copy()
{
  QTextEdit::copy();
  emit pasteAvailable(canPaste());
}

void EditorWidget::deleteText()
{
  insertPlainText(QString());
}

void EditorWidget::openFile(const QString& fileName)
{
  QString filePath = QFileInfo(editorObject->filePath).path() + "/" + fileName;
  editorObject->addEditor(filePath, editorObject->subFileRegExpPattern, false);
  EditorModule::module->openEditor(filePath);
}
