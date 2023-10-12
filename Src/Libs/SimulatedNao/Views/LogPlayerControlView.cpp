#include "LogPlayerControlView.h"
#include "SimulatedNao/RobotConsole.h"
#include "Theme.h"

#include <QBoxLayout>
#include <QEvent>
#include <QHelpEvent>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QProxyStyle>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>

// From http://stackoverflow.com/questions/11132597/qslider-mouse-direct-jump
class SliderAbsoluteSetButtonsStyle : public QProxyStyle
{
public:
  using QProxyStyle::QProxyStyle;

  int styleHint(QStyle::StyleHint hint, const QStyleOption* option = nullptr, const QWidget* widget = nullptr, QStyleHintReturn* returnData = nullptr) const
  {
    if(hint == QStyle::SH_Slider_AbsoluteSetButtons)
      return (Qt::LeftButton | Qt::MiddleButton | Qt::RightButton);
    return QProxyStyle::styleHint(hint, option, widget, returnData);
  }
};

LogPlayerControlWidget::LogPlayerControlWidget(LogPlayerControlView& logPlayerControlView) :
  logPlayerControlView(logPlayerControlView),
  playIcon(":/Icons/icons8-play-50.png"), pauseIcon(":/Icons/Player/icons8-pause-50.png"),
  lastState(logPlayerControlView.logPlayer.state)
{
  setFocusPolicy(Qt::StrongFocus);
  playIcon.setIsMask(true);
  pauseIcon.setIsMask(true);
  QVBoxLayout* vBoxLayout = new QVBoxLayout();
  QHBoxLayout* statusLayout = new QHBoxLayout();
  QHBoxLayout* buttonsLayout = new QHBoxLayout();
  statusLayout->setContentsMargins(0, 0, 0, 0);
  buttonsLayout->setAlignment(Qt::AlignLeft);
  vBoxLayout->addLayout(statusLayout);
  vBoxLayout->addLayout(buttonsLayout);

  frameSlider = new QSlider(Qt::Orientation::Horizontal);
  frameSlider->setRange(0, 0);
  frameSlider->setMinimumWidth(100);
  QStyle* absoluteStyle = new SliderAbsoluteSetButtonsStyle(frameSlider->style()->name());
  absoluteStyle->setParent(frameSlider);
  frameSlider->setStyle(absoluteStyle);
  connect(frameSlider, &QSlider::sliderPressed, [&]{ sliderPressed = true; });
  connect(frameSlider, &QSlider::sliderReleased, [&]{ sliderPressed = false; });
  connect(frameSlider, &QSlider::valueChanged, [&](int frame)
  {
    const auto state = logPlayerControlView.logPlayer.state;
    logPlayerControlView.console.handleConsole(("log goto " + std::to_string(frame)).c_str());
    if(state == LogPlayer::playing)
      logPlayerControlView.console.handleConsole("log start");
  });
  statusLayout->addWidget(frameSlider);

  currentFrameNumber = new LogPlayerFrameInputField(this);
  currentFrameNumber->setFixedWidth(53);
  connect(currentFrameNumber, &LogPlayerFrameInputField::returnPressed, [&]
  {
    logPlayerControlView.console.handleConsole(("log goto " + currentFrameNumber->text()).toStdString().c_str());
    currentFrameNumber->clearFocus();
    if(wasPlaying)
      logPlayerControlView.console.handleConsole("log start");
  });
  connect(currentFrameNumber, &LogPlayerFrameInputField::focusGained, [&]
  {
    wasPlaying = logPlayerControlView.logPlayer.state == LogPlayer::playing;
    logPlayerControlView.logPlayer.state = LogPlayer::stopped;
    currentFrameNumber->setText(QString("%1").arg(static_cast<int>(logPlayerControlView.logPlayer.frame())));
  });
  statusLayout->addWidget(currentFrameNumber);

  frameNumberLabel = new QLabel("/ 0");
  frameNumberLabel->setTextFormat(Qt::TextFormat::PlainText);
  frameNumberLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  frameNumberLabel->setFixedWidth(56);
  statusLayout->addWidget(frameNumberLabel);

  const QSize iconSize(16, 16);
#ifdef MACOS
  const QSize buttonSize(32, 22);
  const int spaceSize = 8;
  buttonsLayout->setSpacing(6);
#else
  const QSize buttonSize(24, 24);
  const int spaceSize = 24;
#endif

  playPauseButton = new QPushButton("");
  playPauseButton->setShortcut(QKeySequence(Qt::Key_Space));
  buttons.append(playPauseButton);
  playPauseButton->setFixedSize(buttonSize);
  updatePlayPauseButton();
  connect(playPauseButton, &QPushButton::released, [&]
  {
    logPlayerControlView.console.handleConsole(logPlayerControlView.logPlayer.state == LogPlayer::playing ? "log pause" : "log start");
  });
  buttonsLayout->addWidget(playPauseButton);

  QIcon stopIcon(":/Icons/Player/icons8-stop-50.png");
  stopIcon.setIsMask(true);
  QPushButton* stopButton = new QPushButton(stopIcon, "");
  stopButton->setShortcut(QKeySequence(Qt::Key_Home));
  buttons.append(stopButton);
  stopButton->setIconSize(iconSize);
  stopButton->setFixedSize(buttonSize);
  stopButton->setToolTip("Stop");
  connect(stopButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log stop");});
  buttonsLayout->addWidget(stopButton);

#ifdef MACOS
  buttonsLayout->addSpacing(spaceSize);
#else
  QFrame* line = new QFrame();
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);
  buttonsLayout->addWidget(line);
#endif

  QIcon repeatIcon(":/Icons/Player/icons8-reset-50.png");
  repeatIcon.setIsMask(true);
  QPushButton* repeatButton = new QPushButton(repeatIcon, "");
  repeatButton->setShortcut(QKeySequence(Qt::Key_End));
  buttons.append(repeatButton);
  repeatButton->setIconSize(iconSize);
  repeatButton->setFixedSize(buttonSize);
  repeatButton->setToolTip("Repeat frame");
  connect(repeatButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log repeat");});
  buttonsLayout->addWidget(repeatButton);

#ifdef MACOS
  buttonsLayout->addSpacing(spaceSize);
#else
  line = new QFrame();
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);
  buttonsLayout->addWidget(line);
#endif

  QIcon prevFastIcon(":/Icons/Player/icons8-rewind-50.png");
  prevFastIcon.setIsMask(true);
  QPushButton* prevFastButton = new QPushButton(prevFastIcon, "");
  prevFastButton->setShortcut(QKeySequence(Qt::Key_PageUp));
  buttons.append(prevFastButton);
  prevFastButton->setIconSize(iconSize);
  prevFastButton->setFixedSize(buttonSize);
  prevFastButton->setToolTip("Go 100 frames back");
  connect(prevFastButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log backward fast");});
  buttonsLayout->addWidget(prevFastButton);

  QIcon prevImageIcon(":/Icons/icons8-skip-to-start-50.png");
  prevImageIcon.setIsMask(true);
  QPushButton* prevImageButton = new QPushButton(prevImageIcon, "");
  prevImageButton->setShortcut(QKeySequence(Qt::Key_Up));
  buttons.append(prevImageButton);
  prevImageButton->setIconSize(iconSize);
  prevImageButton->setFixedSize(buttonSize);
  prevImageButton->setToolTip("Go to previous image");
  connect(prevImageButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log backward image");});
  buttonsLayout->addWidget(prevImageButton);

  QIcon prevFrameIcon(":/Icons/Player/icons8-back-50.png");
  prevFrameIcon.setIsMask(true);
  QPushButton* prevFrameButton = new QPushButton(prevFrameIcon, "");
  prevFrameButton->setShortcut(QKeySequence(Qt::Key_Left));
  buttons.append(prevFrameButton);
  prevFrameButton->setIconSize(iconSize);
  prevFrameButton->setFixedSize(buttonSize);
  prevFrameButton->setToolTip("Go to previous frame");
  connect(prevFrameButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log backward");});
  buttonsLayout->addWidget(prevFrameButton);

  QIcon nextFrameIcon(":/Icons/Player/icons8-forward-50.png");
  nextFrameIcon.setIsMask(true);
  QPushButton* nextFrameButton = new QPushButton(nextFrameIcon, "");
  nextFrameButton->setShortcut(QKeySequence(Qt::Key_Right));
  buttons.append(nextFrameButton);
  nextFrameButton->setIconSize(iconSize);
  nextFrameButton->setFixedSize(buttonSize);
  nextFrameButton->setToolTip("Go to next frame");
  connect(nextFrameButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log forward");});
  buttonsLayout->addWidget(nextFrameButton);

  QIcon nextImageIcon(":/Icons/Player/icons8-end-50.png");
  nextImageIcon.setIsMask(true);
  QPushButton* nextImageButton = new QPushButton(nextImageIcon, "");
  nextImageButton->setShortcut(QKeySequence(Qt::Key_Down));
  buttons.append(nextImageButton);
  nextImageButton->setIconSize(iconSize);
  nextImageButton->setFixedSize(buttonSize);
  nextImageButton->setToolTip("Go to next image");
  connect(nextImageButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log forward image");});
  buttonsLayout->addWidget(nextImageButton);

  QIcon nextFastIcon(":/Icons/Player/icons8-fast-forward-50.png");
  nextFastIcon.setIsMask(true);
  QPushButton* nextFastButton = new QPushButton(nextFastIcon, "");
  nextFastButton->setShortcut(QKeySequence(Qt::Key_PageDown));
  buttons.append(nextFastButton);
  nextFastButton->setIconSize(iconSize);
  nextFastButton->setFixedSize(buttonSize);
  nextFastButton->setToolTip("Go 100 frames forward");
  connect(nextFastButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole("log forward fast");});
  buttonsLayout->addWidget(nextFastButton);

  buttonsLayout->addSpacing(spaceSize);

  QIcon loopIcon(":/Icons/icons8-synchronize-50.png");
  loopIcon.setIsMask(true);
  loopButton = new QPushButton(loopIcon, "");
  buttons.append(loopButton);
  loopButton->setIconSize(iconSize);
  loopButton->setFixedSize(buttonSize);
  loopButton->setCheckable(true);
  connect(loopButton, &QPushButton::released, [&] {logPlayerControlView.console.handleConsole(logPlayerControlView.logPlayer.cycle ? "log once" : "log cycle");});
  buttonsLayout->addWidget(loopButton, 1, Qt::AlignRight);
  updateLoopButton();

  styleButtons();

  setLayout(vBoxLayout);
  setFixedHeight(vBoxLayout->minimumSize().height());
  setEnabled(!logPlayerControlView.logPlayer.empty());
}

void LogPlayerControlWidget::update()
{
  setEnabled(!logPlayerControlView.logPlayer.empty());

  if(!logPlayerControlView.logPlayer.empty())
  {
    SYNC_WITH(logPlayerControlView.console);
    frameNumberLabel->setText(QString("/ %1").arg(logPlayerControlView.logPlayer.frames() - 1));
    currentFrameNumber->setValidator(new QIntValidator(0, static_cast<int>(logPlayerControlView.logPlayer.frames()) - 1, this));

    if(lastState != logPlayerControlView.logPlayer.state)
      updatePlayPauseButton();
    frameSlider->setRange(0, static_cast<int>(logPlayerControlView.logPlayer.frames()) - 1);

    if(!sliderPressed)
    {
      bool alreadyBlocked = frameSlider->blockSignals(true);
      frameSlider->setValue(static_cast<int>(logPlayerControlView.logPlayer.frame()));
      frameSlider->blockSignals(alreadyBlocked);
    }

    if(!currentFrameNumber->hasFocus())
      currentFrameNumber->setText(QString("%1").arg(static_cast<int>(logPlayerControlView.logPlayer.frame())));

    updateLoopButton();
  }
  else
  {
    frameSlider->setRange(0, 0);
    frameNumberLabel->setText(" / 0");
    sliderPressed = false;
  }

  lastState = logPlayerControlView.logPlayer.state;
}

void LogPlayerControlWidget::updatePlayPauseButton()
{
  if(logPlayerControlView.logPlayer.state != LogPlayer::playing)
  {
    playPauseButton->setIcon(Theme::updateIcon(this, playIcon));
    playPauseButton->setToolTip("Play");
  }
  else
  {
    playPauseButton->setIcon(Theme::updateIcon(this, pauseIcon));
    playPauseButton->setToolTip("Pause");
  }
  playPauseButton->setIconSize(QSize(16, 16));
}

void LogPlayerControlWidget::updateLoopButton()
{
  loopButton->setChecked(logPlayerControlView.logPlayer.cycle);
  if(logPlayerControlView.logPlayer.cycle)
    loopButton->setToolTip("Deactivate loop mode");
  else
    loopButton->setToolTip("Activate loop mode");
}

void LogPlayerControlWidget::styleButtons()
{
#ifdef MACOS
  QColor hover(128, 128, 128, Theme::isDarkMode(this) ? 64 : 32);
  QColor pressed(128, 128, 128, Theme::isDarkMode(this) ? 128 : 64);
  QColor checkedHover(128, 128, 128, Theme::isDarkMode(this) ? 192 : 96);
  QColor checkedPressed(128, 128, 128, Theme::isDarkMode(this) ? 255 : 128);
#endif
  for(QPushButton* button : buttons)
  {
    Theme::updateIcon(this, button);
#ifdef MACOS
    button->setStyleSheet("QPushButton {background-color: transparent; padding: 3px 8px 3px 8px; border-width: 0px; border-radius: 4px}"
                          "QPushButton:checked {background-color: " + pressed.name(QColor::HexArgb) + "}"
                          "QPushButton:hover {background-color: " + hover.name(QColor::HexArgb) + "}"
                          "QPushButton:pressed {background-color: " + pressed.name(QColor::HexArgb) + "}"
                          "QPushButton:checked:hover {background-color: " + checkedHover.name(QColor::HexArgb) + "}"
                          "QPushButton:checked:pressed {background-color: " + checkedPressed.name(QColor::HexArgb) + "}");
#endif
  }
}

void LogPlayerControlWidget::changeEvent(QEvent* event)
{
  if(event->type() == QEvent::PaletteChange)
    styleButtons();
  QWidget::changeEvent(event);
}

SimRobot::Widget* LogPlayerControlView::createWidget()
{
  return new LogPlayerControlWidget(*this);
}
