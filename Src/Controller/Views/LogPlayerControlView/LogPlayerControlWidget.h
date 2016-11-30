#pragma once

#include "SimRobot.h"
#include "Controller/LogPlayer.h"

#include <QIcon>
#include <QWidget>
#include <QLineEdit>

class LogPlayerControlView;

class QLabel;
class QPushButton;
class QSlider;

class LogPlayerFrameInputField : public QLineEdit
{
  Q_OBJECT

public:
  LogPlayerFrameInputField(QWidget* parent = nullptr) : QLineEdit(parent) {};
  virtual ~LogPlayerFrameInputField() = default;

signals:
  void focusGained(bool hasFocus);

protected:
  virtual void focusInEvent(QFocusEvent* e)
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

  virtual QWidget* getWidget() { return this; }
  virtual void update();

private:
  void updatePlayPauseButton();
  void updateLoopButton();

  private slots:
  void changeFrame(int newFrame);
  void togglePlayPause();
};