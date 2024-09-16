#pragma once

#include "Libs/SimulatedNao/Views/SACFieldView.h"
#include "Libs/SimulatedNao/Views/ImageView.h"
#include "SimRobot.h"

#include <QIcon>
#include <QLineEdit>
#include <QString>
#include <QWidget>
#include <QCheckBox>

class SACControlWidget;
class RobotConsole;

class QLabel;
class QRadioButton;

class SACControlView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  RobotConsole& console;
  QIcon icon; /**< The icon used to list this view in the scene graph */
  SACFieldView field;
  ImageView imageUpper;
  ImageView imageLower;

  friend class SACControlWidget;

public:
  SACControlView(const QString& fullName, RobotConsole& console) :
    fullName(fullName), console(console), icon(":/Icons/icons8-remote-50.png"),
    field("", console, "worldState", ""),
    imageUpper("", console, "CameraImage", "upper", "Upper", 1.0),
    imageLower("", console, "CameraImage", "lower", "Lower", 1.0)
  {
    icon.setIsMask(true);
  }

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
};

class SACControlWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

private:
  SACControlView& sacControlView;
  SACFieldWidget* fieldWidget;
  SimRobot::Widget* imageUpper;
  SimRobot::Widget* imageLower;

  QCheckBox* tactic;

  bool holdHeadPosition = false;
  std::vector<Qt::Key> pressedKeys;

  void paint(QPainter& painter) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;

public:
  SACControlWidget(SACControlView& sacControlView);

  QWidget* getWidget() override { return this; }
  void update() override;

private slots:
  void tacticChanged();
};
