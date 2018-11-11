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

  const QString& getFullName() const override {return fullName;}
  const QIcon* getIcon() const override;

  SimRobotEditor::Editor* addFile(const QString& filePath, const QString& subFileRegExpPattern) override;
  SimRobotEditor::Editor* addFolder(const QString& name) override;
};

class FileEditorObject : public EditorObject
{
public:
  QString filePath;
  QString subFileRegExpPattern;
  bool persistent;

  FileEditorObject(const QString& filePath, const QString& subFileRegExpPattern, bool persistent, EditorObject* parent);

  const QIcon* getIcon() const override;
  SimRobot::Widget* createWidget() override;
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

  bool useTabStop;
  int tabStopWidth;

  mutable QSignalMapper openFileMapper;
  SyntaxHighlighter* highlighter;

  void updateEditMenu(QMenu* menu, bool aboutToShow) const;

  QWidget* getWidget() override {return this;}
  bool canClose() override;
  QMenu* createFileMenu() const override;
  QMenu* createEditMenu() const override;

  QSize sizeHint () const override { return QSize(640, 480); }
  void contextMenuEvent(QContextMenuEvent* event) override;
  void focusInEvent(QFocusEvent * event) override;
  void keyPressEvent(QKeyEvent* event) override;

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
