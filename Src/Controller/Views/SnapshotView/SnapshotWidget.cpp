#include <QFormLayout>
#include <QHBoxLayout>
#include <QString>
#include <QStyleOptionButton>
#include <QVBoxLayout>

#include "Controller/RobotConsole.h"
#include "SnapshotWidget.h"

SnapshotWidget::SnapshotWidget(SnapshotView& view) :
  view(view), alreadyLogging(false), count(0)
{
  /** Create layouts and containers to hold ui elements **/
  QVBoxLayout* mainLayout = new QVBoxLayout();
  mainLayout->setSpacing(2);
  mainLayout->setAlignment(Qt::AlignTop);
  QGroupBox* settingsBox = new QGroupBox();
  QFormLayout* settingsBoxLayout = new QFormLayout();
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->setSpacing(10);
  buttonLayout->setAlignment(Qt::AlignCenter);

  /** Connect these layouts and containers **/
  setLayout(mainLayout);
  settingsBox->setLayout(settingsBoxLayout);
  mainLayout->addWidget(settingsBox);
  mainLayout->addSpacing(4);
  mainLayout->addLayout(buttonLayout);

  /** Create ui elements **/
  prefixEdit = new QLineEdit();
  prefixEdit->setPlaceholderText("def. \"image\" - prefix for the filename");
  countEdit = new QLineEdit();
  countEdit->setPlaceholderText("def. \"0\" - where to start counting from");
  intervalEdit = new QLineEdit();
  intervalEdit->setPlaceholderText("def. \"2\" - log after every x seconds");
  upper = new QCheckBox();
  lower = new QCheckBox();
  loopButton = new QPushButton("Start Logging");
  snapshotButton = new QPushButton("Quick Snapshot");
  stateReportText = new QLabel("Status: Not Logging");

  /** Insert ui elements into layouts and containers **/
  settingsBoxLayout->addRow(new QLabel("Prefix:"), prefixEdit);
  settingsBoxLayout->addRow(new QLabel("Count:"), countEdit);
  settingsBoxLayout->addRow(new QLabel("Interval:"), intervalEdit);
  settingsBoxLayout->addRow(new QLabel("Log upper:"), upper);
  settingsBoxLayout->addRow(new QLabel("Log lower:"), lower);
  buttonLayout->addWidget(loopButton);
  buttonLayout->addWidget(snapshotButton);
  mainLayout->addWidget(stateReportText);

  /** On macOS, the buttons are not high enough.
  Make them both the same size, fitting the bigger label. **/
  QStyleOptionButton opt;
  opt.initFrom(snapshotButton);
  QSize size = snapshotButton->fontMetrics().size(Qt::TextShowMnemonic, snapshotButton->text());
  opt.rect.setSize(size);
  size = snapshotButton->style()->sizeFromContents(QStyle::CT_PushButton, &opt, size, snapshotButton);
  loopButton->setMinimumSize(size);
  snapshotButton->setMinimumSize(size);

  /** Connect ui elements to slots **/
  QWidget::connect(loopButton, SIGNAL(pressed()), this, SLOT(onLoop()));
  QWidget::connect(snapshotButton, SIGNAL(pressed()), this, SLOT(onSnapshot()));

  /** Prepare the looping */
  loopTimer = new QTimer();
  QWidget::connect(loopTimer, SIGNAL(timeout()), this, SLOT(run()));
}

SnapshotWidget::~SnapshotWidget()
{ }

void SnapshotWidget::saveImages(const std::string& path, int number)
{
  SYNC_WITH(view.console);
  if(upper->isChecked())
  {
    saveImage(view.console.upperCamImages["raw image"].image,
              "upper_" + path,
              number);
  }

  if(lower->isChecked())
  {
    saveImage(view.console.lowerCamImages["raw image"].image,
              "lower_" + path,
              number);
  }
}

void SnapshotWidget::saveImage(DebugImage* image, const std::string& path, int number)
{
  if(image)
  {
    Image i(false);
    image->toImage(i);
    LogPlayer::saveImage(i, path.c_str(), number);
  }
}

void SnapshotWidget::update()
{
  QWidget::update();
}

void SnapshotWidget::onLoop()
{
  if(!alreadyLogging)
  {
    alreadyLogging = true;
    loopButton->setText("Stop logging");
    prefix = prefixEdit->text().isEmpty() ? "image" : prefixEdit->text().toUtf8().toStdString();
    count = countEdit->text().isEmpty() ? 0 : countEdit->text().toInt();
    interval = intervalEdit->text().isEmpty() ? 2000 : intervalEdit->text().toLongLong() * 1000;
    elapsedtimer.start();
    loopTimer->start(1000);
    stateReportText->setText("Status: Start Logging");
  }
  else
  {
    loopTimer->stop();
    alreadyLogging = false;
    loopButton->setText("Start logging");
    stateReportText->setText("Status: Not Logging");
  }
}

void SnapshotWidget::run()
{
  if(elapsedtimer.hasExpired(interval))
  {
    QString text;
    text += "Status: ";
    text += QString::number(count);
    text += " per selected camera";
    stateReportText->setText(text);
    saveImages(prefix, count++);
    elapsedtimer.start();
  }
}

void SnapshotWidget::onSnapshot()
{
  prefix = prefixEdit->text().isEmpty() ? "image" : prefixEdit->text().toUtf8().toStdString();
  saveImages(prefix, count++);
  elapsedtimer.start();
}
