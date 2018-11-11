/**
 * @file Controller/Views/ConsoleView.h
 * Declarations of class ConsoleView
 * @author Colin Graf
 */

#pragma once

#include <QIcon>
#include <QTextEdit>
#include <SimRobot.h>

class ConsoleRoboCupCtrl;
class ConsoleWidget;

/**
 * @class ConsoleView
 * The class implements a scene graph object for displaying and receiving user input and output via a text field
 */
class ConsoleView : public SimRobot::Object
{
public:
  /**
   * @param fullName The path in the scene graph pointing to this view
   * @param console The robot console
   * @param loadAndSaveOutput Whether the view stores and restores its output
   */
  ConsoleView(const QString& fullName, ConsoleRoboCupCtrl& console, bool loadAndSaveOutput = false) :
    fullName(fullName), icon(":/Icons/textfield.png"), console(console), loadAndSaveOutput(loadAndSaveOutput)
  {}

  void clear();
  void printLn(const QString& text);
  void print(const QString& text);

private:
  QString fullName; /**< The path to this view in the scene graph */
  QIcon icon; /**< The icon used for listing this view in the scene graph */
  ConsoleRoboCupCtrl& console; /**< A reference to the console object. */
  ConsoleWidget* consoleWidget = nullptr; /**< The widget for displaying the output and receiving input. */
  QString output; /**< A buffer used for storing output as long as there is no window */
  bool loadAndSaveOutput; /**< Whether the view stores and restores its output */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class ConsoleWidget;
};

/**
 * @class ConsoleWidget
 * The class implements a console like text edit
 */
class ConsoleWidget : public QTextEdit, public SimRobot::Widget
{
  Q_OBJECT

public:
  ConsoleWidget(ConsoleView& consoleView, ConsoleRoboCupCtrl& console, QString& output, ConsoleWidget*& consoleWidget);
  ~ConsoleWidget();

  void print(const QString& text);

private:
  ConsoleView& consoleView;
  ConsoleRoboCupCtrl& console;
  QString& output;
  ConsoleWidget*& consoleWidget;
  QStringList history;
  QStringList::iterator history_iter;

  bool canCopy = false;
  bool canUndo = false;
  bool canRedo = false;

  QSize sizeHint() const override { return QSize(640, 240); }

  QWidget* getWidget() override { return this; }
  void keyPressEvent(QKeyEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void focusInEvent(QFocusEvent* event) override;

  QMenu* createEditMenu() const override;

signals:
  void pasteAvailable(bool available);

private slots:
  void copyAvailable(bool available);
  void redoAvailable(bool available);
  void undoAvailable(bool available);
  void cut();
  void copy();
  void deleteText();
};
