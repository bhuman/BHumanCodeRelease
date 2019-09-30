#include "LogPlayerControlView.h"
#include "Controller/RobotConsole.h"

#include <QBoxLayout>
#include <QEvent>
#include <QHelpEvent>
#include <QLabel>
#include <QLineEdit>
#include <QProxyStyle>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QStyleOptionSlider>
#include <QToolTip>

// From http://stackoverflow.com/questions/11132597/qslider-mouse-direct-jump
class SliderAbsoluteSetButtonsStyle : public QProxyStyle
{
public:
  using QProxyStyle::QProxyStyle;

  int styleHint(QStyle::StyleHint hint, const QStyleOption* option = nullptr, const QWidget* widget = nullptr, QStyleHintReturn* returnData = nullptr) const
  {
    if(hint == QStyle::SH_Slider_AbsoluteSetButtons)
      return (Qt::LeftButton | Qt::MidButton | Qt::RightButton);
    return QProxyStyle::styleHint(hint, option, widget, returnData);
  }
};

class FrameSlider : public QSlider
{
  using QSlider::QSlider;

  // from qslider.cpp
  int pixelPosToRangeValue(int pos) const
  {
    QStyleOptionSlider opt;
    initStyleOption(&opt);
    QRect gr = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this);
    QRect sr = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
    int sliderMin, sliderMax, sliderLength;

    if(orientation() == Qt::Horizontal)
    {
      sliderLength = sr.width();
      sliderMin = gr.x();
      sliderMax = gr.right() - sliderLength + 1;
    }
    else
    {
      sliderLength = sr.height();
      sliderMin = gr.y();
      sliderMax = gr.bottom() - sliderLength + 1;
    }
    return QStyle::sliderValueFromPosition(minimum(), maximum(), pos - sliderMin, sliderMax - sliderMin, opt.upsideDown);
  }

  // from qslider.cpp
  inline int pick(const QPoint& pt) const
  {
    return orientation() == Qt::Horizontal ? pt.x() : pt.y();
  }

  bool event(QEvent* event) override
  {
    if(event->type() == QEvent::ToolTip)
    {
      QHelpEvent* helpEvent = static_cast<QHelpEvent*>(event);

      QStyleOptionSlider opt;
      initStyleOption(&opt);
      const QRect sliderRect = style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
      const QPoint center = sliderRect.center() - sliderRect.topLeft();
      const int val = pixelPosToRangeValue(pick(helpEvent->pos() - center));
      QToolTip::showText(helpEvent->globalPos(), QString::number(val));
      return true;
    }
    else
      return QSlider::event(event);
  }
};

LogPlayerControlWidget::LogPlayerControlWidget(LogPlayerControlView& logPlayerControlView) :
  logPlayerControlView(logPlayerControlView),
  playIcon(loadIcon(":/Icons/Player/play.png")), pauseIcon(loadIcon(":/Icons/Player/pause.png")),
  lastState(logPlayerControlView.logPlayer.state)
{
  QVBoxLayout* vBoxLayout = new QVBoxLayout();
  QHBoxLayout* statusLayout = new QHBoxLayout();
  QHBoxLayout* buttonsLayout = new QHBoxLayout();
  statusLayout->setContentsMargins(0, 0, 0, 0);
  buttonsLayout->setAlignment(Qt::AlignLeft);
  vBoxLayout->addLayout(statusLayout);
  vBoxLayout->addLayout(buttonsLayout);

  frameSlider = new FrameSlider(Qt::Orientation::Horizontal);
  frameSlider->setRange(0, 0);
  frameSlider->setMinimumWidth(100);
  frameSlider->setFixedHeight(20);
  frameSlider->setStyle(new SliderAbsoluteSetButtonsStyle(frameSlider->style()));
  connect(frameSlider, &QSlider::sliderPressed, [&]() { sliderPressed = true; });
  connect(frameSlider, &QSlider::sliderReleased, [&]() { sliderPressed = false; });
  connect(frameSlider, &QSlider::valueChanged, this, &LogPlayerControlWidget::changeFrame);
  statusLayout->addWidget(frameSlider);

  currentFrameNumber = new LogPlayerFrameInputField(this);
  currentFrameNumber->setFixedWidth(50);
  connect(currentFrameNumber, &LogPlayerFrameInputField::returnPressed, [&]()
  {
    logPlayerControlView.logPlayer.gotoFrame(currentFrameNumber->text().toInt());
    currentFrameNumber->clearFocus();
    if(wasPlaying)
      logPlayerControlView.logPlayer.play();
  });
  connect(currentFrameNumber, &LogPlayerFrameInputField::focusGained, [&]()
  {
    wasPlaying = logPlayerControlView.logPlayer.state == LogPlayer::LogPlayerState::playing;
    logPlayerControlView.logPlayer.pause();
    currentFrameNumber->setText(QString("%1").arg(logPlayerControlView.logPlayer.currentFrameNumber));
  });
  statusLayout->addWidget(currentFrameNumber);

  frameNumberLabel = new QLabel(" / 0");
  frameNumberLabel->setTextFormat(Qt::TextFormat::PlainText);
  frameNumberLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  frameNumberLabel->setFixedSize(50, 20);
  statusLayout->addWidget(frameNumberLabel);

#ifdef MACOS
  int buttonSize = devicePixelRatio() > 1 ? 38 : 25;
  buttonsLayout -> setSpacing(6);
#else
  int buttonSize = 25;
#endif

  playPauseButton = new QPushButton("");
  playPauseButton->setFixedSize(buttonSize, buttonSize);
  updatePlayPauseButton();
  connect(playPauseButton, &QPushButton::released, this, &LogPlayerControlWidget::togglePlayPause);
  buttonsLayout->addWidget(playPauseButton);

  QPushButton* stopButton = new QPushButton("");
  stopButton->setIcon(loadIcon(":/Icons/Player/stop.png"));
  stopButton->setIconSize(QSize(23, 23));
  stopButton->setFixedSize(buttonSize, buttonSize);
  stopButton->setToolTip("Stop");
  connect(stopButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.stop(); });
  buttonsLayout->addWidget(stopButton);

  QFrame* line = new QFrame();
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);
  buttonsLayout->addWidget(line);

  QPushButton* repeatButton = new QPushButton("");
  repeatButton->setIcon(loadIcon(":/Icons/Player/repeat.png"));
  repeatButton->setIconSize(QSize(23, 23));
  repeatButton->setFixedSize(buttonSize, buttonSize);
  repeatButton->setToolTip("Repeat frame");
  connect(repeatButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.stepRepeat(); });
  buttonsLayout->addWidget(repeatButton);

  line = new QFrame();
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);
  buttonsLayout->addWidget(line);

  QPushButton* prevImageButton = new QPushButton(loadIcon(":/Icons/Player/prevImage.png"), "");
  prevImageButton->setIconSize(QSize(23, 23));
  prevImageButton->setFixedSize(buttonSize, buttonSize);
  prevImageButton->setToolTip("Go to previous image");
  connect(prevImageButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.stepImageBackward(); });
  buttonsLayout->addWidget(prevImageButton);

  QPushButton* prevFrameButton = new QPushButton(loadIcon(":/Icons/Player/prevFrame.png"), "");
  prevFrameButton->setIconSize(QSize(23, 23));
  prevFrameButton->setFixedSize(buttonSize, buttonSize);
  prevFrameButton->setToolTip("Go to previous frame");
  connect(prevFrameButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.stepBackward(); });
  buttonsLayout->addWidget(prevFrameButton);

  QPushButton* nextFrameButton = new QPushButton(loadIcon(":/Icons/Player/nextFrame.png"), "");
  nextFrameButton->setIconSize(QSize(23, 23));
  nextFrameButton->setFixedSize(buttonSize, buttonSize);
  nextFrameButton->setToolTip("Go to next frame");
  connect(nextFrameButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.stepForward(); });
  buttonsLayout->addWidget(nextFrameButton);

  QPushButton* nextImageButton = new QPushButton(loadIcon(":/Icons/Player/nextImage.png"), "");
  nextImageButton->setIconSize(QSize(23, 23));
  nextImageButton->setFixedSize(buttonSize, buttonSize);
  nextImageButton->setToolTip("Go to next image");
  connect(nextImageButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.stepImageForward(); });
  buttonsLayout->addWidget(nextImageButton);

  buttonsLayout->addSpacing(25);

  loopButton = new QPushButton(loadIcon(":/Icons/Player/loop.png"), "");
  loopButton->setIconSize(QSize(23, 23));
  loopButton->setFixedSize(buttonSize, buttonSize);
  loopButton->setCheckable(true);
  connect(loopButton, &QPushButton::released, [&]() { SYNC_WITH(logPlayerControlView.console); logPlayerControlView.logPlayer.setLoop(!logPlayerControlView.logPlayer.getLoop()); });
  buttonsLayout->addWidget(loopButton, 1, Qt::AlignRight);
  updateLoopButton();

  setLayout(vBoxLayout);
  setFixedHeight(vBoxLayout->minimumSize().height());

  // assign shortcuts
  QShortcut* sc_play_pause = new QShortcut(QKeySequence("SPACE"), this);
  connect(sc_play_pause, &QShortcut::activated, this, &LogPlayerControlWidget::togglePlayPause);

  QShortcut* sc_next_frame = new QShortcut(QKeySequence("RIGHT"), this);
  connect(sc_next_frame, &QShortcut::activated, nextFrameButton, &QPushButton::released);

  QShortcut* sc_prev_frame = new QShortcut(QKeySequence("LEFT"), this);
  connect(sc_prev_frame, &QShortcut::activated, prevFrameButton, &QPushButton::released);

  QShortcut* sc_next_image = new QShortcut(QKeySequence("UP"), this);
  connect(sc_next_image, &QShortcut::activated, prevImageButton, &QPushButton::released);

  QShortcut* sc_prev_image = new QShortcut(QKeySequence("DOWN"), this);
  connect(sc_prev_image, &QShortcut::activated, nextImageButton, &QPushButton::released);

  QShortcut* sc_fast_forward = new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_Right), this);
  connect(sc_fast_forward, &QShortcut::activated, this, [&]()
  {
    SYNC_WITH(logPlayerControlView.console);
    const int frame = logPlayerControlView.logPlayer.currentFrameNumber + 100;
    logPlayerControlView.logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayerControlView.logPlayer.numberOfFrames - 1), 0));
  });

  QShortcut* sc_fast_backward = new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_Left), this);
  connect(sc_fast_backward, &QShortcut::activated, this, [&]()
  {
    SYNC_WITH(logPlayerControlView.console);
    const int frame = logPlayerControlView.logPlayer.currentFrameNumber - 100;
    logPlayerControlView.logPlayer.gotoFrame(std::max<>(std::min<>(frame - 1, logPlayerControlView.logPlayer.numberOfFrames - 1), 0));
  });

  setDisabled(!logPlayerControlView.logPlayer.logfilePath.empty());
}

void LogPlayerControlWidget::update()
{
  setEnabled(!logPlayerControlView.logPlayer.logfilePath.empty());

  if(!logPlayerControlView.logPlayer.logfilePath.empty())
  {
    SYNC_WITH(logPlayerControlView.console);
    frameNumberLabel->setText(QString(" / %1").arg(logPlayerControlView.logPlayer.numberOfFrames - 1));
    currentFrameNumber->setValidator(new QIntValidator(0, logPlayerControlView.logPlayer.numberOfFrames - 1, this));

    if(lastState != logPlayerControlView.logPlayer.state)
      updatePlayPauseButton();
    frameSlider->setRange(0, logPlayerControlView.logPlayer.numberOfFrames - 1);

    if(!sliderPressed)
    {
      bool alreadyBlocked = frameSlider->blockSignals(true);
      frameSlider->setValue(logPlayerControlView.logPlayer.currentFrameNumber);
      frameSlider->blockSignals(alreadyBlocked);
    }

    if(!currentFrameNumber->hasFocus())
      currentFrameNumber->setText(QString("%1").arg(logPlayerControlView.logPlayer.currentFrameNumber));

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
  if(logPlayerControlView.logPlayer.state != LogPlayer::LogPlayerState::playing)
  {
    playPauseButton->setIcon(playIcon);
    playPauseButton->setToolTip("Play");
  }
  else
  {
    playPauseButton->setIcon(pauseIcon);
    playPauseButton->setToolTip("Pause");
  }
  playPauseButton->setIconSize(QSize(23, 23));
}

void LogPlayerControlWidget::updateLoopButton()
{
  loopButton->setChecked(logPlayerControlView.logPlayer.getLoop());
  if(logPlayerControlView.logPlayer.getLoop())
    loopButton->setToolTip("Deactivate loop mode");
  else
    loopButton->setToolTip("Activate loop mode");
}

void LogPlayerControlWidget::changeFrame(int newFrame)
{
  SYNC_WITH(logPlayerControlView.console);
  if(logPlayerControlView.logPlayer.state != LogPlayer::LogPlayerState::initial)
  {
    bool wasPlaying = logPlayerControlView.logPlayer.state == LogPlayer::LogPlayerState::playing;
    logPlayerControlView.logPlayer.gotoFrame(newFrame);
    if(wasPlaying)
      logPlayerControlView.logPlayer.play();
  }
}

void LogPlayerControlWidget::togglePlayPause()
{
  SYNC_WITH(logPlayerControlView.console);
  if(logPlayerControlView.logPlayer.state == LogPlayer::LogPlayerState::playing)
    logPlayerControlView.logPlayer.pause();
  else
    logPlayerControlView.logPlayer.play();
}

QIcon LogPlayerControlWidget::loadIcon(const QString& name)
{
  QIcon icon(name);
#ifdef MACOS
  if(palette().background().color().lightness() < palette().foreground().color().lightness())
  {
    QIcon invertedIcon;
    for(auto& size : icon.availableSizes())
    {
      QImage image = icon.pixmap(size).toImage();
      image.invertPixels();
      invertedIcon.addPixmap(QPixmap::fromImage(std::move(image)));
    }
    return invertedIcon;
  }
  else
#endif
    return icon;
}

SimRobot::Widget* LogPlayerControlView::createWidget()
{
  return new LogPlayerControlWidget(*this);
}
