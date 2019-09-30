#pragma once

#include "SimRobot.h"
#include "Controller/LogPlayer.h"

#include <QIcon>
#include <QLineEdit>
#include <QString>
#include <QWidget>

class LogPlayerControlWidget;
class RobotConsole;

class QLabel;
class QPushButton;
class QSlider;

class LogPlayerControlView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  LogPlayer& logPlayer;
  RobotConsole& console;
  const QIcon icon; /**< The icon used to list this view in the scene graph */

  friend class LogPlayerControlWidget;

public:
  LogPlayerControlView(const QString& fullName, LogPlayer& logPlayer, RobotConsole& console) :
    fullName(fullName), logPlayer(logPlayer), console(console), icon(":/Icons/tag_green.png")
  {}

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

class LogPlayerFrameInputField : public QLineEdit
{
  Q_OBJECT

public:
  LogPlayerFrameInputField(QWidget* parent = nullptr) : QLineEdit(parent) {};
  ~LogPlayerFrameInputField() = default;

signals:
  void focusGained(bool hasFocus);

protected:
  void focusInEvent(QFocusEvent* e) override
  {
    QLineEdit::focusInEvent(e);
    emit(focusGained(true));
  }
};

class LogPlayerControlWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

private:
  LogPlayerControlView& logPlayerControlView;

  QSlider* frameSlider;
  LogPlayerFrameInputField* currentFrameNumber;
  QLabel* frameNumberLabel;

  QIcon playIcon;
  QIcon pauseIcon;
  QPushButton* playPauseButton;
  QPushButton* loopButton;

  bool sliderPressed = false;
  bool wasPlaying = false;
  LogPlayer::LogPlayerState lastState;

public:
  LogPlayerControlWidget(LogPlayerControlView& logPlayerControlView);

  QWidget* getWidget() override { return this; }
  void update() override;

private:
  void updatePlayPauseButton();
  void updateLoopButton();
  QIcon loadIcon(const QString& name);

private slots:
  void changeFrame(int newFrame);
  void togglePlayPause();
};
