/**
 * @file Controller/Views/DotView.h
 * Declaration of a view that displays a dot graph
 * @author Colin Graf
 */

#pragma once

#include <QString>
#include <QIcon>
#include <QGraphicsView>
#include <SimRobot.h>

/**
 * A scene graph object for SimRobot that can be used to open the widget
 */
class DotViewObject : public SimRobot::Object
{
public:
  /**
   * @param fullName The path to this object in the scene graph
   */
  DotViewObject(const QString& fullName) : fullName(fullName), icon(":/Icons/tag_green.png") {}

protected:
  /**
   * Checks whether the content that will be returned from a \c generateDotFileContent call has changed
   * @return \c true When \c generateDotFileContent will return something new
   */
  virtual bool hasChanged() = 0;

  /**
   * Returns the content of the dot graph file that will be displayed
   * @return The content of the dot graph file
   */
  virtual QString generateDotFileContent() = 0;

private:
  const QString fullName; /**< The path name to this object in the scene graph */
  const QIcon icon; /**< The icon used to list this view in the scene graph */

  /**
   * The method returns a new instance of a widget for this view.
   * The caller has to delete the returned instance. (Qt will take care of this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  /**
   * Accesses path name to the object in the scene graph
   * @return The path name
   */
  const QString& getFullName() const override { return fullName; }

  /**
   * Accesses the icon used to list this view in the scene graph
   * @return The icon
   */
  const QIcon* getIcon() const override { return &icon; }

  friend class DotViewWidget;
};

/**
 * The qt widget that displays the dot graph
 */
class DotViewWidget : public QGraphicsView, public SimRobot::Widget
{
  Q_OBJECT

public:
  /**
   * initializes the widget and loads the scroll and zoom state
   * @param dotViewObject The DotViewObject that created this widget
   */
  DotViewWidget(DotViewObject& dotViewObject);

private:
  DotViewObject& dotViewObject; /**< The DotViewObject that created this widget */

  /**
   * saves the scroll and zoom state and destroys the widget
   */
  ~DotViewWidget();

  /**
   * Returns a customized size hint that is used to compute the default size of the widget
   * @return The customized size
   */
  QSize sizeHint() const override { return QSize(640, 480); }

  /**
   * Returns this widget casted to a QWidget
   * @return Teh widget
   */
  QWidget* getWidget() override { return this; }

  /**
   * Updates the displayed content of the widget
   */
  void update() override;

  /**
   * Returns a menu that will be displayed when the widget has input focus
   * @return The menu
   */
  QMenu* createUserMenu() const override;

  /**
   * Loads and displays a svg from a file
   * @param fileName The path of the file
   * @return Whether the file was loaded successfully
   */
  bool openSvgFile(const QString& fileName);

  /**
   * Loads and displays a dot graph from a file
   * @param fileName The path of the file
   * @return Whether the file was loaded successfully
   */
  bool openDotFile(const QString& fileName);

  /**
   * Loads and displays a dot graph from a string
   * @param fileName The path of the file
   * @return Whether the file was loaded successfully
   */
  bool openDotFileContent(const QString& content);

  /**
   * Saves a dot graph
   * @param fileName The path of the file to save the dot graph in
   * @return Whether the dot graph was saved successfully
   */
  bool saveDotFileContent(const QString& content, const QString& fileName);

  /**
   * Converts a dot graph file into another format
   * @param fmt The format to convert the dot file in (see "dot -T" documentation for supported formats)
   * @param src The path of the dot graph file
   * @param dest The pathof the output file
   * @return Whether the dot graph was converted successfully
   */
  bool convertDotFile(const QString& fmt, const QString& src, const QString& dest);

  /**
   * Compiles a command to convert a dot graph file into another format
   * @param fmt The format to convert the dot file in (see "dot -T" documentation for supported formats)
   * @param src The path of the dot graph file
   * @param dest The pathof the output file
   * @return The command
   */
  QString builtDotCommand(const QString& fmt, const QString& src, const QString& dest) const;

  bool viewportEvent(QEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

private slots:
  /**
   * Opens the "Save As..." dialog and saves the displayed graph as svg
   */
  void exportAsSvg();

  /**
   * Opens the "Save As..." dialog and saves the displayed graph as dot graph file
   */
  void exportAsDot();

  /**
   * Opens the "Save As..." dialog and saves the displayed graph as pdf
   */
  void exportAsPdf();
};
