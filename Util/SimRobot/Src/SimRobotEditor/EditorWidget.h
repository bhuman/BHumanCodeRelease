/**
* @file SimRobotEditor/EditorWidget.h
* Declaration of class EditorWidget
* @author Colin Graf
*/

#pragma once

#include <QCheckBox>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalMapper>
#include <QSpinBox>
#include <QTextEdit>

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
  class EditorSettingsDialog : public QDialog
  {
  public:
    EditorSettingsDialog(QWidget* parent = 0);

    QLabel* useTabStopLabel;
    QCheckBox* useTabStopCheckBox;
    QLabel* tabStopWidthLabel;
    QSpinBox* tabStopWidthSpinBox;
    QPushButton* okayPushButton;
    QPushButton* closePushButton;
  };

  class FindAndReplaceDialog : public QDialog
  {
  public:
    FindAndReplaceDialog(QWidget* parent = 0);

    QLabel* findLabel;
    QLabel* replaceLabel;
    QLineEdit* findTextEdit;
    QLineEdit* replaceTextEdit;
    QCheckBox* caseCheckBox;
    QCheckBox* wholeWordsCheckBox;
    QCheckBox* regexCheckBox;
    QPushButton* nextPushButton;
    QPushButton* previousPushButton;
    QPushButton* replacePushButton;
    QPushButton* replaceAllPushButton;
  };

  enum FindAndReplaceAction
  {
    find,
    findBackwards,
    replace,
    replaceAll
  };

  FileEditorObject* editorObject;

  bool canCopy;
  bool canUndo;
  bool canRedo;

  bool useTabStop;
  int tabStopWidth;

  mutable QSignalMapper openFileMapper, findAndReplaceMapper;
  SyntaxHighlighter* highlighter;
  EditorSettingsDialog* editorSettingsDialog;
  FindAndReplaceDialog* findAndReplaceDialog;

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
  void openFindAndReplace();
  void findAndReplace(int action);
  void openSettings();
  void updateSettingsFromDialog();
  void openFile(const QString& fileName);
};
