#include "ui/TeamView.h"
#include "models/Team.h"
#include "ui/RobotView.h"
#include "ui/TeamSelector.h"
#include "tools/Filesystem.h"

#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QDial>
#include <QLineEdit>
#include <QSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QToolTip>
#include <QCursor>
#include <QAction>
#include <QString>

void TeamView::init()
{
  if(!team)
    return;

  QFormLayout* layout = new QFormLayout();
  QHBoxLayout* settingsGrid = new QHBoxLayout();
  settingsGrid->setSpacing(6);
  settingsGrid->setAlignment(Qt::AlignmentFlag::AlignLeft);

  pbSave = new QPushButton(QIcon(":icons/disk.png"), "");
  pbSave->setShortcut(QKeySequence(static_cast<int>(Qt::CTRL) + static_cast<int>(Qt::Key_S)));
  pbSave->setToolTip("Save team configuration");
  settingsGrid->addWidget(pbSave);
  connect(pbSave, &QPushButton::clicked, teamSelector, [selector = teamSelector](){ selector->saveTeams(); });

  cbColor = new QComboBox(this);
  cbColor->addItem("blue");
  cbColor->addItem("red");
  cbColor->addItem("yellow");
  cbColor->addItem("black");
  cbColor->addItem("white");
  cbColor->addItem("green");
  cbColor->addItem("orange");
  cbColor->addItem("purple");
  cbColor->addItem("brown");
  cbColor->addItem("gray");
  cbColor->setCurrentIndex(cbColor->findText(QString::fromStdString(team->color)));
  cbColor->setToolTip("Select team color");
  settingsGrid->addWidget(cbColor);
  connect(cbColor, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TeamView::colorChanged);

  sbNumber = new QSpinBox(this);
  sbNumber->setRange(1, 99);
  sbNumber->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  sbNumber->setValue(team->number);
  sbNumber->setToolTip("Select team number");
  settingsGrid->addWidget(sbNumber);
  connect(sbNumber, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &TeamView::numberChanged);

  cbScenario = new QComboBox(this);
  std::vector<std::string> scenarios = Filesystem::getScenarios();
  std::sort(scenarios.begin(), scenarios.end());
  for(const std::string& scenario : scenarios)
    cbScenario->addItem(QString::fromStdString(scenario));
  cbScenario->setCurrentIndex(cbScenario->findText(QString::fromStdString(team->scenario)));
  cbScenario->setToolTip("Select scenario");
  settingsGrid->addWidget(cbScenario);
  connect(cbScenario, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TeamView::scenarioChanged);

  cbLocation = new QComboBox(this);
  std::vector<std::string> locations = Filesystem::getLocations();
  std::sort(locations.begin(), locations.end());
  for(const std::string& location : locations)
    cbLocation->addItem(QString::fromStdString(location));
  cbLocation->setCurrentIndex(cbLocation->findText(QString::fromStdString(team->location)));
  cbLocation->setToolTip("Select location");
  settingsGrid->addWidget(cbLocation);
  connect(cbLocation, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TeamView::locationChanged);

  cbWlanConfig = new QComboBox(this);
  std::vector<std::string> configs = Filesystem::getWlanConfigs();
  std::sort(configs.begin(), configs.end());
  for(const std::string& config : configs)
    cbWlanConfig->addItem(QString::fromStdString(config));
  cbWlanConfig->setCurrentIndex(cbWlanConfig->findText(QString::fromStdString(team->wlanConfig)));
  cbWlanConfig->setToolTip("Select wireless network");
  settingsGrid->addWidget(cbWlanConfig);
  connect(cbWlanConfig, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TeamView::wlanConfigChanged);

  cbBuildConfig = new QComboBox(this);
  cbBuildConfig->addItem("Debug");
  cbBuildConfig->addItem("Develop");
  cbBuildConfig->addItem("Release");
  cbBuildConfig->setCurrentIndex(cbBuildConfig->findText(QString::fromStdString(team->buildConfig)));
  cbBuildConfig->setToolTip("Select build configuration");
  settingsGrid->addWidget(cbBuildConfig);
  connect(cbBuildConfig, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TeamView::buildConfigChanged);

  sVolume = new QDial(this);
  sVolume->setMinimum(0);
  sVolume->setMaximum(11);
  sVolume->setPageStep(3);
  sVolume->setNotchesVisible(true);
  sVolume->setValue((team->volume * 11 + 50) / 100);
  sVolume->setToolTip("Set audio volume");
  settingsGrid->addWidget(sVolume);
  connect(sVolume, &QDial::valueChanged, this, &TeamView::volumeChanged);

  settingsGrid->addStretch();

  cbDeployDevice = new QComboBox(this);
  cbDeployDevice->addItem("auto");
  cbDeployDevice->addItem("Ethernet");
  cbDeployDevice->addItem("Wi-Fi");
  cbDeployDevice->setCurrentIndex(cbDeployDevice->findText(QString::fromStdString(team->deployDevice)));
  cbDeployDevice->setToolTip("Select connection to robot");
  settingsGrid->addWidget(cbDeployDevice);
  connect(cbDeployDevice, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &TeamView::deployDeviceChanged);

  sbMagic = new QSpinBox(this);
  sbMagic->setRange(-1, 255);
  sbMagic->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  sbMagic->setValue(team->magicNumber);
  sbMagic->setToolTip("Select magic number (-1 means random)");
  settingsGrid->addWidget(sbMagic);
  connect(sbMagic, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &TeamView::magicNumberChanged);

  cbCompile = new QCheckBox("Build", this);
  cbCompile->setChecked(team->compile);
  cbCompile->setToolTip("Build before deploying?");
  settingsGrid->addWidget(cbCompile);
  connect(cbCompile, &QCheckBox::toggled, this, &TeamView::compileChanged);

  layout->addRow(settingsGrid);

  QFrame* hr = new QFrame(this);
  hr->setFrameStyle(static_cast<int>(QFrame::Sunken) | static_cast<int>(QFrame::HLine));
  QLabel* players = new QLabel("<b>Players</b>");
  layout->addRow(players, hr);
  layout->setAlignment(hr, Qt::AlignVCenter);

  QGridLayout* teamGrid = new QGridLayout();
  generateRobotViews(teamGrid);
  layout->addRow(teamGrid);

  setLayout(layout);
}

TeamView::TeamView(TeamSelector* parent, Team* team) :
  QFrame(parent),
  teamSelector(parent),
  team(team)
{
  init();
}

void TeamView::generateRobotViews(QGridLayout* teamGrid)
{
  const auto& robots = team->getPlayersPerNumber();
  size_t max = robots.size();
  for(size_t j = 0; j < 2; ++j)
    for(size_t i = 0; i < max; ++i)
    {
      RobotView* rv = new RobotView(teamSelector, robots[i][j], static_cast<unsigned short>(i + 1), static_cast<unsigned short>(j));
      robotViews.push_back(rv);
      teamGrid->addWidget(rv, j > 0 ? 2 : 0, static_cast<int>(i));
    }
  QFrame* hr = new QFrame(this);
  hr->setFrameStyle(static_cast<int>(QFrame::Sunken) | static_cast<int>(QFrame::HLine));
  QFormLayout* subTitle = new QFormLayout();
  teamGrid->addLayout(subTitle, 1, 0, 1, static_cast<int>(max), Qt::AlignVCenter);

  QLabel* substitutes = new QLabel("<b>Substitutes</b> (bhuman will not start)");
  subTitle->addRow(substitutes, hr);
  subTitle->setAlignment(hr, Qt::AlignVCenter);
}

void TeamView::update(size_t index)
{
  robotViews[index]->update();
}

void TeamView::colorChanged(int index)
{
  if(team)
    team->color = cbColor->itemText(index).toStdString();
}

void TeamView::numberChanged(int number)
{
  if(team)
    team->number = static_cast<unsigned short>(number);
}

void TeamView::scenarioChanged(int index)
{
  if(team)
    team->scenario = cbScenario->itemText(index).toStdString();
}

void TeamView::locationChanged(int index)
{
  if(team)
    team->location = cbLocation->itemText(index).toStdString();
}

void TeamView::wlanConfigChanged(int index)
{
  if(team)
    team->wlanConfig = cbWlanConfig->itemText(index).toStdString();
}

void TeamView::buildConfigChanged(int index)
{
  if(team)
    team->buildConfig = cbBuildConfig->itemText(index).toStdString();
}

void TeamView::volumeChanged(const int volume)
{
  if(team)
  {
    team->volume = static_cast<unsigned short>((volume * 100 + 6) / 11);
    QToolTip::showText(QCursor::pos(), QString::number(volume));
  }
}

void TeamView::deployDeviceChanged(int index)
{
  if(team)
    team->deployDevice = cbDeployDevice->itemText(index).toStdString();
}

void TeamView::magicNumberChanged(const int magicnumber)
{
  if(team)
    team->magicNumber = magicnumber;
}

void TeamView::compileChanged(const bool checked)
{
  if(team)
    team->compile = checked;
}
