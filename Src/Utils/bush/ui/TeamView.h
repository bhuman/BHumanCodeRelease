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
  QPushButton* pbSave;
  QComboBox* cbColor;
  QSpinBox* sbNumber;
  QComboBox* cbScenario;
  QComboBox* cbLocation;
  QComboBox* cbWlanConfig;
  QComboBox* cbBuildConfig;
  QComboBox* cbDeployDevice;
  QDial* sVolume;
  QSpinBox* sbMagic;
  QCheckBox* cbCompile;

  void init();
public:
  TeamView(TeamSelector* parent, Team* team);
  void generateRobotViews(QGridLayout* teamGrid);
  void update(size_t index);

private slots:
  void colorChanged(const QString& color);
  void numberChanged(int number);
  void scenarioChanged(const QString& scenario);
  void locationChanged(const QString& location);
  void wlanConfigChanged(const QString& config);
  void buildConfigChanged(const QString& build);
  void volumeChanged(const int volume);
  void deployDeviceChanged(const QString& device);
  void magicNumberChanged(int magicnumber);
  void compileChanged(bool checked);
};
