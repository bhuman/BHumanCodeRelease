/**
* @file SimRobotEditor/EditorModule.cpp
* Implementation of class EditorModule
* @author Colin Graf
*/

#include <QString>
#include <QFile>
#include <QTextStream>
#include <QFileInfo>
#include <QSettings>

#include "EditorModule.h"
#include "EditorWidget.h"

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  return new EditorModule(simRobot);
}

EditorModule* EditorModule::module;
SimRobot::Application* EditorModule::application;

EditorModule::EditorModule(SimRobot::Application& application) : EditorObject("Editor", 0), fileIcon(":/Icons/page_white_text.png"), folderIcon(":/Icons/folder.png"), editorIcon(":/Icons/page_white_stack.png")
{
  this->module = this;
  this->application = &application;
}

void EditorModule::registerEditor(FileEditorObject* editor)
{
  //ASSERT(!editorsByPath.value(editor->filePath));
  editorsByPath.insert(editor->filePath, editor);
}

void EditorModule::unregisterEditor(FileEditorObject* editor)
{
  editorsByPath.remove(editor->filePath);
}

FileEditorObject* EditorModule::findEditor(const QString& filePath)
{
  return editorsByPath.value(filePath);
}

void EditorModule::openEditor(const QString& filePath)
{
  EditorObject* editor = editorsByPath.value(filePath);
  if(!editor)
    return;
  application->openObject(*editor);
}

bool EditorModule::compile()
{
  Q_ASSERT(static_cast<void*>(this) == static_cast<void*>(static_cast<SimRobotEditor::Editor*>(this)));

  application->registerObject(*this, *this, 0, SimRobot::Flag::windowless);

  QString filePath = application->getFilePath();
  addEditor(filePath, "href\\s*=\\s*\\\"([ \\\\/a-z0-9\\.\\-_]+\\.rsi2)\\\"", true);

  loadFromSettings();

  return true;
}
