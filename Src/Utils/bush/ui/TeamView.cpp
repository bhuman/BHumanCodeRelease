#include "Utils/bush/ui/TeamView.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/ui/RobotView.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Filesystem.h"

#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QToolTip>
#include <QCursor>

void TeamView::init()
{
  if(team)
  {
    QFormLayout* layout = new QFormLayout();
    QHBoxLayout * settingsGrid = new QHBoxLayout();
    settingsGrid->setSpacing(6);
    settingsGrid->setAlignment(Qt::AlignmentFlag::AlignLeft);

    pbSave = new QPushButton(QIcon(":icons/disk.png"), "");
    pbSave->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
    pbSave->setToolTip("Save Team Configuration");
    settingsGrid->addWidget(pbSave);
    connect(pbSave, SIGNAL(clicked()), teamSelector, SLOT(saveTeams()));

    cbColor = new QComboBox(this);
    cbColor->addItem("red");
    cbColor->addItem("blue");
    cbColor->addItem("yellow");
    cbColor->addItem("black");
    cbColor->setCurrentIndex(cbColor->findText(fromString(team->color)));
    settingsGrid->addWidget(new QLabel("Color:", cbColor));
    settingsGrid->addWidget(cbColor);
    connect(cbColor, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(colorChanged(const QString&)));

    sbNumber = new QSpinBox(this);
    sbNumber->setRange(1, 99);
    sbNumber->setButtonSymbols(QAbstractSpinBox::NoButtons);
    sbNumber->setMaximumWidth(18);
    sbNumber->setValue(team->number);
    settingsGrid->addWidget(new QLabel("Number:", sbNumber));
    settingsGrid->addWidget(sbNumber);
    connect(sbNumber, SIGNAL(valueChanged(int)), this, SLOT(numberChanged(int)));

    cbLocation = new QComboBox(this);
    std::vector<std::string> locations = Filesystem::getLocations();
    for(size_t i = 0; i < locations.size(); ++i)
      cbLocation->addItem(fromString(locations[i]));
    cbLocation->setCurrentIndex(cbLocation->findText(fromString(team->location)));
    settingsGrid->addWidget(new QLabel("Location:", lePort));
    settingsGrid->addWidget(cbLocation);
    connect(cbLocation, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(locationChanged(const QString&)));

    cbWlanConfig = new QComboBox(this);
    std::vector<std::string> configs = Filesystem::getWlanConfigs();
    for(size_t i = 0; i < configs.size(); ++i)
      cbWlanConfig->addItem(fromString(configs[i]));
    cbWlanConfig->setCurrentIndex(cbWlanConfig->findText(fromString(team->wlanConfig)));
    settingsGrid->addWidget(new QLabel("Wlan:", cbWlanConfig));
    settingsGrid->addWidget(cbWlanConfig);
    connect(cbWlanConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(wlanConfigChanged(const QString&)));

    cbBuildConfig = new QComboBox(this);
    cbBuildConfig->addItem("Develop");
    cbBuildConfig->addItem("Release");
    cbBuildConfig->addItem("Debug");
    cbBuildConfig->setCurrentIndex(cbBuildConfig->findText(fromString(team->buildConfig)));
    settingsGrid->addWidget(new QLabel("Conf:", cbBuildConfig));
    settingsGrid->addWidget(cbBuildConfig);
    connect(cbBuildConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(buildConfigChanged(const QString&)));

    sVolume = new QSlider(this);
    sVolume->setMinimum(0);
    sVolume->setMaximum(100);
    sVolume->setTickInterval(1);
    sVolume->setValue(team->volume);
    settingsGrid->addWidget(new QLabel("Vol:", sVolume));
    settingsGrid->addWidget(sVolume);
    connect(sVolume, SIGNAL(valueChanged(int)), this, SLOT(volumeChanged(const int)));

    settingsGrid->addStretch();

    cbDeployDevice = new QComboBox(this);
    cbDeployDevice->addItem("auto");
    cbDeployDevice->addItem("lan");
    cbDeployDevice->addItem("wlan");
    cbDeployDevice->setCurrentIndex(cbDeployDevice->findText(fromString(team->deployDevice)));
    settingsGrid->addWidget(new QLabel("Device:", cbDeployDevice));
    settingsGrid->addWidget(cbDeployDevice);
    connect(cbDeployDevice, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(deployDeviceChanged(const QString&)));

    layout->addRow(settingsGrid);
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    layout->addRow(hr);

    QGridLayout* teamGrid = new QGridLayout();
    generateRobotViews(teamGrid);
    layout->addRow(teamGrid);

    setLayout(layout);
  }
}

TeamView::TeamView(TeamSelector* parent, Team* team)
  : QFrame(parent),
    teamSelector(parent),
    team(team),
    robotViews(),
    cbColor(0),
    sbNumber(0),
    lePort(0),
    cbLocation(0),
    cbWlanConfig(0),
    cbBuildConfig(0)/*,
    handicapSlider(0)*/
{
  init();
}

void TeamView::generateRobotViews(QGridLayout* teamGrid)
{
  std::vector<std::vector<Robot*> > robots = team->getPlayersPerNumber();
  size_t max = robots.size();
  bool backup = true;
  for(size_t j = 0; j < 2; ++j)
    for(size_t i = 0; i < max; ++i)
    {
      RobotView* rv = new RobotView(teamSelector, robots[i][j], (unsigned short) (i + 1), (unsigned short) j);
      robotViews.push_back(rv);
      teamGrid->addWidget(rv, j > 0 ? 2 : 0, (int) i);
    }
  if(backup)
  {
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    teamGrid->addWidget(hr, 1, 0, 1, (int) max);
  }
}

void TeamView::update(size_t index)
{
  robotViews[index]->update();
}

void TeamView::colorChanged(const QString& color)
{
  if(team)
    team->color = toString(color);
}

void TeamView::numberChanged(int number)
{
  if(team)
  {
    team->number = (unsigned short) number;
    team->port = (unsigned short) (10000 + number);
  }
}

void TeamView::locationChanged(const QString& location)
{
  if(team)
    team->location = toString(location);
}

void TeamView::wlanConfigChanged(const QString& config)
{
  if(team)
    team->wlanConfig = toString(config);
}

void TeamView::buildConfigChanged(const QString& build)
{
  if(team)
    team->buildConfig = toString(build);
}

void TeamView::volumeChanged(const int volume)
{
  if(team)
  {
    team->volume = static_cast<unsigned short>(volume);
    QToolTip::showText(QCursor::pos(), QString::number(volume) + QString("%"));
  }
}

void TeamView::deployDeviceChanged(const QString& device)
{
  if(team)
    team->deployDevice = toString(device);
}
