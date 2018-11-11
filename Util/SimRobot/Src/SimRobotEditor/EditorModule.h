/**
* @file SimRobotEditor/EditorModule.h
* Declaration of class EditorModule
* @author Colin Graf
*/

#pragma once

#include <QIcon>
#include <QHash>

#include "SimRobotEditor.h"
#include "EditorWidget.h"

class EditorModule : public EditorObject, public SimRobot::Module
{
public:
  static EditorModule* module;
  static SimRobot::Application* application;

  QIcon fileIcon;
  QIcon folderIcon;
  QIcon editorIcon;

  EditorModule(SimRobot::Application& application);

  void registerEditor(FileEditorObject* editor);
  void unregisterEditor(FileEditorObject* editor);
  void openEditor(const QString& filePath);
  FileEditorObject* findEditor(const QString& filePath);

private:
  QHash<QString, FileEditorObject*> editorsByPath;

  bool compile() override;

  const QIcon* getIcon() const override {return &editorIcon;}
};
