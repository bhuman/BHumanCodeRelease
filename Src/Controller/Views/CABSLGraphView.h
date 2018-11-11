/**
 * @file Controller/Views/CABSLGraphView.h
 * Declaration of a class to represent a view displaying the cabsl option call graph of a behavior.
 * @author Colin Graf
 */

#pragma once

#include "DotView.h"

class RobotConsole;

/**
 * A class to represent a view displaying the cabsl option call graph of a behavior.
 */
class CABSLGraphViewObject : public DotViewObject
{
public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param behaviorName The name of the behavior directory in the BehaviorControl folder
   * @param optionsFile The name of the file that includes all other option files
   */
  CABSLGraphViewObject(const QString& fullName, const QString& behaviorName, const QString& optionsFile);

private:
  class Option
  {
    QString originalLine, filePath, optionName, dotColorDesc;
    QStringList calls;

  public:
    Option(const QString& line);
    void findCalls(const QString& behaviorName, QList<Option*>& options);
    QString declareNode();
    QString drawCalls();
    QString pattern();
  };

  QString behaviorName; /**< The name of the behavior directory in the BehaviorControl folder */
  QString optionsFile; /**< The name of the file that includes all other option files */

  /**
   * Checks whether the content that will be returned from a \c generateDotFileContent call has changed
   * @return \c true When \c generateDotFileContent will return something new
   */
  bool hasChanged() override { return false; }

  /**
   * Returns the content of the dot graph file that will be displayed
   * @return The content of the dot graph file
   */
  QString generateDotFileContent() override;
};
