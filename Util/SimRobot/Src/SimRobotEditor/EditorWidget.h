/**
* @file SimRobotEditor/EditorWidget.h
* Declaration of class EditorWidget
* @author Colin Graf
*/

#pragma once

#include <QTextEdit>
#include <QSignalMapper>

#include "SimRobotEditor.h"
#include "SyntaxHighlighter.h"

class FileEditorObject;

class EditorObject : public SimRobotEditor::Editor
{
public:
  EditorObject* parent;
  QString name;
  QString fullName;

  EditorObject(const QString& name, EditorObject* parent);
  ~EditorObject();

  SimRobotEditor::Editor* addEditor(const QString& filePath, const QString& subFileRegExpPattern, bool persistent);
  void removeEditor(FileEditorObject* editor);

protected:
  void loadFromSettings();

private:
  QList<EditorObject*> editors; /**< List of subfiles and folders */
  QHash<QString, EditorObject*> foldersByName;

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const;

  virtual SimRobotEditor::Editor* addFile(const QString& filePath, const QString& subFileRegExpPattern);
  virtual SimRobotEditor::Editor* addFolder(const QString& name);
};

class FileEditorObject : public EditorObject
{
public:
  QString filePath;
  QString subFileRegExpPattern;
  bool persistent;

  FileEditorObject(const QString& filePath, const QString& subFileRegExpPattern, bool persistent, EditorObject* parent);

  virtual const QIcon* getIcon() const;
  virtual SimRobot::Widget* createWidget();
};

class EditorWidget : public QTextEdit, public SimRobot::Widget
{
  Q_OBJECT

public:
  EditorWidget(FileEditorObject* editorObject, const QString& fileContent);
  ~EditorWidget();

private:
  FileEditorObject* editorObject;

  bool canCopy;
  bool canUndo;
  bool canRedo;

  QSignalMapper openFileMapper;
  SyntaxHighlighter* highlighter;

  void updateEditMenu(QMenu* menu, bool aboutToShow) const;

  virtual QWidget* getWidget() {return this;}
  virtual bool canClose();
  virtual QMenu* createFileMenu() const;
  virtual QMenu* createEditMenu() const;

  virtual QSize sizeHint () const { return QSize(640, 480); }
  virtual void contextMenuEvent(QContextMenuEvent* event);
  virtual void focusInEvent(QFocusEvent * event);
  virtual void keyPressEvent(QKeyEvent* event);

signals:
  void pasteAvailable(bool available);

private slots:
  void updateEditMenu();
  void copyAvailable(bool available);
  void redoAvailable(bool available);
  void undoAvailable(bool available);
  void save();
  void cut();
  void copy();
  void deleteText();
  void openFile(const QString& fileName);
};
