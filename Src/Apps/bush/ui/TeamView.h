#pragma once

#include <QFrame>
#include <vector>

class Team;
class QPushButton;
class QComboBox;
class QCheckBox;
class QDial;
class QLineEdit;
class QSpinBox;
class QLabel;
class QGridLayout;
class TeamSelector;
class RobotView;

class TeamView : public QFrame
{
  Q_OBJECT

  TeamSelector* teamSelector;
  Team* team;

  std::vector<RobotView*> robotViews;
  QPushButton* pbSave = nullptr;
  QComboBox* cbColor = nullptr;
  QSpinBox* sbNumber = nullptr;
  QComboBox* cbScenario = nullptr;
  QComboBox* cbLocation = nullptr;
  QComboBox* cbWlanConfig = nullptr;
  QComboBox* cbBuildConfig = nullptr;
  QComboBox* cbDeployDevice = nullptr;
  QDial* sVolume = nullptr;
  QSpinBox* sbMagic = nullptr;
  QCheckBox* cbCompile = nullptr;

  void init();
public:
  TeamView(TeamSelector* parent, Team* team);
  void generateRobotViews(QGridLayout* teamGrid);
  void update(size_t index);

private slots:
  void colorChanged(int index);
  void numberChanged(int number);
  void scenarioChanged(int index);
  void locationChanged(int index);
  void wlanConfigChanged(int index);
  void buildConfigChanged(int index);
  void volumeChanged(const int volume);
  void deployDeviceChanged(int index);
  void magicNumberChanged(int magicnumber);
  void compileChanged(bool checked);
};
