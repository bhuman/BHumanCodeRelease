/**
 * @file SettingsArea.cpp
 *
 * This file implements a class that represents the settings area on the right side
 * of the dialog. It consists of two tab groups and a few buttons.
 *
 * @author Thomas Röfer
 */

#include "SettingsArea.h"
#include <iostream>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMenu>
#include <QRadioButton>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QToolTip>
#include "Streaming/InStreams.h"
#include "../../Util/SimRobot/Src/SimRobot/Theme.h"

/** A horizontal line to separate different rows in the preset tabs. */
class Line : public QFrame
{
  void updateColor(QWidget* widget)
  {
    QPalette pal = palette();
    QColor base = pal.dark().color();
    pal.setColor(QPalette::WindowText, Theme::isDarkMode(widget) ? base.darker() : base);
    setPalette(pal);
  }

public:
  Line(QWidget* parent)
  {
    setFrameShape(QFrame::HLine);
    updateColor(parent);
  }

  void changeEvent(QEvent* event) override
  {
    if(event->type() == QEvent::PaletteChange)
      updateColor(this);
    QFrame::changeEvent(event);
  }
};

/** A line editor to change the names of tabs. Mainly specializes leaving the editor. */
class LineEdit : public QLineEdit
{
  const QString originalText;

public:
  LineEdit(const QString& text) : QLineEdit(text), originalText(text)
  {
    installEventFilter(this);
  }

  bool eventFilter(QObject* source, QEvent* event)
  {
    if(event->type() == QEvent::KeyPress && source == this)
    {
      QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
      if(keyEvent->modifiers() == Qt::NoModifier
         && (keyEvent->key() == Qt::Key_Return
             || keyEvent->key() == Qt::Key_Enter
             || keyEvent->key() == Qt::Key_Escape))
      {
        if(keyEvent->key() == Qt::Key_Escape)
          setText(originalText);
        event->accept();
        emit editingFinished();
        return true;
      }
    }
    else if(event->type() == QEvent::FocusOut)
      emit editingFinished();
    return QLineEdit::eventFilter(source, event);
  }
};

SettingsArea::SettingsArea(Presets& presets, QDialog* dialog, RobotsTable* table, const QSettings& settings)
  : presets(presets), table(table)
{
  presetIndex = settings.value("presetIndex", 0).toInt();
  mode = static_cast<Mode>(settings.value("mode", robots).toInt());
  restart = settings.value("restart", true).toBool();
  deleteLogs = settings.value("deleteLogs", false).toBool();
  playerNumber = settings.value("playerNumber", 5).toInt();
  reboot = settings.value("reboot", false).toBool();
  usbCheck = settings.value("usbCheck", false).toBool();
  date = settings.value("date", false).toBool();
  logsMode = static_cast<LogsMode>(settings.value("logsMode", download).toInt());
  close = settings.value("close", true).toBool();

  // Read known teams.
  Teams teams;
  InMapFile teamsStream("teamList.cfg");
  if(teamsStream.exists())
    teamsStream >> teams;
  for(const Teams::Team& team : teams.teams)
    if(team.number)
      this->teams[team.name] = team.number;

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(createPresetTabs());

  QPushButton* deployButton = new QPushButton();
  deployButton->setFocusPolicy(Qt::StrongFocus);
  deployButton->setDefault(true);
  connect(deployButton, &QPushButton::clicked, dialog, &QDialog::accept);
  auto updateDeployButton = [=, this]
  {
    deployButton->setText(mode == robots ? "Deploy" : mode == image ? "Write" : logsMode == justDelete ? "Delete" : "Download");
    if(mode == image)
      deployButton->setEnabled(true);
    else
    {
      for(const std::string& player : selectedPreset->players)
        if(player != "_")
        {
          deployButton->setEnabled(true);
          return;
        }
      deployButton->setEnabled(false);
    }
  };
  updateDeployButton();
  connect(table, &RobotsTable::robotAssignmentChanged, updateDeployButton);

  QPushButton* cancelButton = new QPushButton("Cancel");
  cancelButton->setFocusPolicy(Qt::StrongFocus);
  deployButton->setDefault(false);
  connect(cancelButton, &QPushButton::clicked, dialog, &QDialog::reject);

  QTabWidget* modesWidget = new QTabWidget();
  layout->addWidget(modesWidget);
  modesWidget->addTab(createRobotsTab(), "Robots");
  modesWidget->addTab(createImageTab(), "Image");
  modesWidget->addTab(createLogsTab(updateDeployButton), "Logs");
  auto selectMode = [=, this](int index)
  {
    mode = static_cast<Mode>(index);
    updateDeployButton();
  };
  connect(modesWidget, &QTabWidget::currentChanged, selectMode);
  modesWidget->setCurrentIndex(mode);

  layout->addStretch();

  QGridLayout* buttonsLayout = new QGridLayout();
  layout->addLayout(buttonsLayout, Qt::AlignBottom);

  QCheckBox* closeButton = new QCheckBox("Close");
  closeButton->setChecked(close);
  closeButton->setFocusPolicy(Qt::StrongFocus);
  connect(closeButton, &QCheckBox::stateChanged, [&](int state) {close = state != Qt::Unchecked;});
  buttonsLayout->addWidget(closeButton);
  buttonsLayout->setColumnStretch(0, 2);

#ifdef MACOS
  constexpr int deployColumn = 2;
#else
  const int deployColumn = 1;
#endif
  buttonsLayout->addWidget(deployButton, 0, deployColumn);
  buttonsLayout->addWidget(cancelButton, 0, 3 - deployColumn);
  buttonsLayout->setColumnStretch(1, 3);
  buttonsLayout->setColumnStretch(2, 3);
}

QWidget* SettingsArea::createPresetTabs()
{
  QTabWidget* widget = new QTabWidget();
  widget->setUsesScrollButtons(true);

  for(Presets::Preset* preset : presets.teams)
    widget->addTab(createPresetTab(preset), preset->name.c_str());

  auto selectPreset = [=, this](int index)
  {
    presetIndex = index;
    selectedPreset = presets.teams[index];
    table->setSelectedPreset(selectedPreset, index);
  };
  QTabBar* tabBar = widget->tabBar();
  tabBar->setMovable(true);
  tabBar->setContextMenuPolicy(Qt::CustomContextMenu);

#ifdef MACOS
    tabBar->setStyleSheet("QToolButton {background-color: transparent; padding: 0 0 2 0; border 0; border-radius: 4}"
                          "QToolButton:hover {background-color: rgba(128, 128, 128, 64)}");
#endif

  connect(widget->tabBar(), &QTabBar::tabMoved, [=, this](int from, int to)
  {
    Presets::Preset* temp = presets.teams[from];
    presets.teams.erase(presets.teams.begin() + from);
    presets.teams.insert(presets.teams.begin() + to, temp);
    table->movePreset(from, to);
    selectPreset(widget->currentIndex());
  });

  connect(widget->tabBar(), &QTabBar::customContextMenuRequested, [=, this](const QPoint& point)
  {
    const int index = tabBar->tabAt(point);

    auto editName = [=, this](int index)
    {
      LineEdit* lineEdit = new LineEdit(tabBar->tabText(index));
      tabBar->setTabText(index, "");
      const QString prevStyleSheet = tabBar->styleSheet();
      tabBar->setStyleSheet(prevStyleSheet + "::tab:selected {padding: 0 0 0 0}");
      tabBar->setTabButton(index, QTabBar::RightSide, lineEdit);
      Qt::FocusPolicy policy = tabBar->focusPolicy();
      tabBar->setFocusPolicy(Qt::NoFocus);
      lineEdit->selectAll();
      lineEdit->setFocus(Qt::OtherFocusReason);
      connect(lineEdit, &QLineEdit::editingFinished, [=, this]
      {
        tabBar->setTabText(index, lineEdit->text());
        presets.teams[index]->name = lineEdit->text().toStdString();
        tabBar->setTabButton(index, QTabBar::RightSide, nullptr);
        tabBar->setStyleSheet(prevStyleSheet);
        tabBar->setFocusPolicy(policy);
      });
    };

    QMenu menu("Team", this);

    QAction* duplicate = new QAction("&Duplicate", this);
    connect(duplicate, &QAction::triggered, [=, this]
    {
      presets.teams.emplace_back(new Presets::Preset(*presets.teams[index]));
      table->addPreset(static_cast<int>(presets.teams.back()->players.size()));
      widget->addTab(createPresetTab(presets.teams.back()), presets.teams.back()->name.c_str());
      widget->setCurrentIndex(tabBar->count() - 1);
      editName(tabBar->count() - 1);
    });
    menu.addAction(duplicate);

    QAction* remove = new QAction("&Delete", this);
    remove->setEnabled(tabBar->count() > 1);
    connect(remove, &QAction::triggered, [=, this]
    {
      table->removePreset(index);
      delete presets.teams[index];
      presets.teams.erase(presets.teams.begin() + index);
      widget->setCurrentIndex(std::max(index, widget->count() - 2));
      widget->removeTab(index);
    });
    menu.addAction(remove);

    QAction* rename = new QAction("&Rename", this);
    connect(rename, &QAction::triggered, [=]
    {
      widget->setCurrentIndex(index);
      editName(index);
    });
    menu.addAction(rename);

    menu.exec(widget->tabBar()->mapToGlobal(point));
  });

  if(presetIndex >= static_cast<int>(presets.teams.size()))
    presetIndex = static_cast<int>(presets.teams.size()) - 1;
  widget->setCurrentIndex(presetIndex);
  selectPreset(presetIndex);
  connect(widget, &QTabWidget::currentChanged, selectPreset);

  return widget;
}

QWidget* SettingsArea::createPresetTab(Presets::Preset* preset)
{
  QWidget* widget = new QWidget();
  QFormLayout* layout = new QFormLayout(widget);

  QComboBox* teamSelector = new QComboBox();
  teamSelector->setFocusPolicy(Qt::StrongFocus);
  for(const auto& [team, number] : this->teams)
  {
    teamSelector->addItem(team.c_str());
    if(preset->number == number)
      teamSelector->setCurrentIndex(teamSelector->count() - 1);
  }
  teamSelector->setMaximumWidth(settingsFieldWidth);
  connect(teamSelector, &QComboBox::currentTextChanged, this, [=, this](const QString& team) {preset->number = this->teams[team.toStdString()];});
  layout->addRow("Team", teamSelector);

  const QStringList colors = {"black", "blue", "brown", "gray", "green", "orange", "purple", "red", "white", "yellow"};
  QComboBox* fieldPlayerColorSelector = new QComboBox();
  fieldPlayerColorSelector->setFocusPolicy(Qt::StrongFocus);
  fieldPlayerColorSelector->addItems(colors);
  fieldPlayerColorSelector->setCurrentText(preset->fieldPlayerColor.c_str());
  fieldPlayerColorSelector->setMaximumWidth(settingsFieldWidth);
  connect(fieldPlayerColorSelector, &QComboBox::currentTextChanged, this, [=](const QString& color) {preset->fieldPlayerColor = color.toStdString();});
  layout->addRow("Field player color", fieldPlayerColorSelector);

  QComboBox* goalkeeperColorSelector = new QComboBox();
  goalkeeperColorSelector->setFocusPolicy(Qt::StrongFocus);
  goalkeeperColorSelector->addItems(colors);
  goalkeeperColorSelector->setCurrentText(preset->goalkeeperColor.c_str());
  goalkeeperColorSelector->setMaximumWidth(settingsFieldWidth);
  connect(goalkeeperColorSelector, &QComboBox::currentTextChanged, this, [=](const QString& color) {preset->goalkeeperColor = color.toStdString();});
  layout->addRow("Goalkeeper color", goalkeeperColorSelector);

  layout->addRow(new Line(this));

  const QStringList scenarios = QDir("Scenarios").entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
  QComboBox* scenarioSelector = new QComboBox();
  scenarioSelector->setFocusPolicy(Qt::StrongFocus);
  scenarioSelector->addItems(scenarios);
  scenarioSelector->setCurrentText(preset->scenario.c_str());
  scenarioSelector->setMaximumWidth(settingsFieldWidth);
  connect(scenarioSelector, &QComboBox::currentTextChanged, this, [=](const QString& scenario) {preset->scenario = scenario.toStdString();});
  layout->addRow("Scenario", scenarioSelector);

  const QStringList locations = QDir("Locations").entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
  QComboBox* locationSelector = new QComboBox();
  locationSelector->setFocusPolicy(Qt::StrongFocus);
  locationSelector->addItems(locations);
  locationSelector->setCurrentText(preset->location.c_str());
  locationSelector->setMaximumWidth(settingsFieldWidth);
  connect(locationSelector, &QComboBox::currentTextChanged, this, [=](const QString& location) {preset->location = location.toStdString();});
  layout->addRow("Location", locationSelector);

  QSpinBox* magicNumberSelector = new QSpinBox();
  magicNumberSelector->setRange(-1, 255);
  magicNumberSelector->setSpecialValueText("auto");
  magicNumberSelector->setValue(preset->magicNumber);
#ifdef LINUX
  magicNumberSelector->setFixedWidth(55);
#else
  magicNumberSelector->setFixedWidth(50);
#endif
  connect(magicNumberSelector, &QSpinBox::valueChanged, this, [=](int magicNumber) {preset->magicNumber = magicNumber;});
  layout->addRow("Magic number", magicNumberSelector);

  layout->addRow(new Line(this));

  const QStringList profiles = QDir("../Install/Profiles").entryList(QDir::Files, QDir::Name);
  QComboBox* profileSelector = new QComboBox();
  profileSelector->setFocusPolicy(Qt::StrongFocus);
  profileSelector->addItems(profiles);
  profileSelector->setCurrentText(preset->wlanConfig.c_str());
  profileSelector->setMaximumWidth(settingsFieldWidth);
  connect(profileSelector, &QComboBox::currentTextChanged, this, [=](const QString& profile) {preset->wlanConfig = profile.toStdString();});
  layout->addRow("Wireless profile", profileSelector);

  QSlider* volumeSelector = new QSlider(Qt::Horizontal);
  volumeSelector->setFocusPolicy(Qt::StrongFocus);
  volumeSelector->setRange(0, 100);
  volumeSelector->setValue(preset->volume);
  volumeSelector->setFixedWidth(settingsFieldWidth);
  connect(volumeSelector, &QSlider::sliderMoved, [&](int volume) {QToolTip::showText(QCursor::pos(), QString("%1").arg(volume));});
  connect(volumeSelector, &QSlider::valueChanged, [=](int volume) {preset->volume = volume;});
  layout->addRow("Volume", volumeSelector);

  return widget;
}

QWidget* SettingsArea::createRobotsTab()
{
  QWidget* widget = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(widget);

  QCheckBox* restartSelector = new QCheckBox("Restart bhuman");
  restartSelector->setChecked(restart);
  restartSelector->setFocusPolicy(Qt::StrongFocus);
  connect(restartSelector, &QCheckBox::stateChanged, [&](int state) {restart = state != Qt::Unchecked;});
  layout->addWidget(restartSelector);

  QCheckBox* deleteLogsSelector = new QCheckBox("Delete logs on internal drive");
  deleteLogsSelector->setChecked(deleteLogs);
  deleteLogsSelector->setFocusPolicy(Qt::StrongFocus);
  connect(deleteLogsSelector, &QCheckBox::stateChanged, [&](int state) {deleteLogs = state != Qt::Unchecked;});
  layout->addWidget(deleteLogsSelector);

  return widget;
}

QWidget* SettingsArea::createImageTab()
{
  QWidget* widget = new QWidget();
  QGridLayout* layout = new QGridLayout(widget);

  QFormLayout* playerNumberLayout = new QFormLayout();
  playerNumberLayout->setFormAlignment(Qt::AlignLeft);
  layout->addLayout(playerNumberLayout, 0, 0);
  QSpinBox* playerNumberSelector = new QSpinBox();
  playerNumberSelector->setRange(1, presets.teams.empty() ? 7 : static_cast<int>(presets.teams[0]->players.size()));
  playerNumberSelector->setValue(playerNumber);
  connect(playerNumberSelector, &QSpinBox::valueChanged, this, [&](int number) {playerNumber = number;});
  playerNumberLayout->addRow("Player", playerNumberSelector);

  QCheckBox* usbCheckSelector = new QCheckBox("USB check");
  usbCheckSelector->setChecked(usbCheck);
  usbCheckSelector->setFocusPolicy(Qt::StrongFocus);
  connect(usbCheckSelector, &QCheckBox::stateChanged, [&](int state) {usbCheck = state != Qt::Unchecked;});
  layout->addWidget(usbCheckSelector, 0, 1);

  QCheckBox* dateSelector = new QCheckBox("Add date");
  dateSelector->setChecked(date);
  dateSelector->setFocusPolicy(Qt::StrongFocus);
  connect(dateSelector, &QCheckBox::stateChanged, [&](int state) {date = state != Qt::Unchecked;});
  layout->addWidget(dateSelector, 1, 0);

  QCheckBox* rebootSelector = new QCheckBox("Reboot");
  rebootSelector->setChecked(reboot);
  rebootSelector->setFocusPolicy(Qt::StrongFocus);
  connect(rebootSelector, &QCheckBox::stateChanged, [&](int state) {reboot = state != Qt::Unchecked;});
  layout->addWidget(rebootSelector, 1, 1);

  return widget;
}

QWidget* SettingsArea::createLogsTab(const std::function<void()>& updateDeployButton)
{
  QWidget* widget = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(widget);
  QHBoxLayout* rowLayout = new QHBoxLayout();
  layout->addLayout(rowLayout);

  QRadioButton* downloadSelector = new QRadioButton("Download");
  downloadSelector->setChecked(logsMode == download);
  downloadSelector->setFocusPolicy(Qt::StrongFocus);
  connect(downloadSelector, &QRadioButton::toggled, [=, this](bool checked) {if(checked) logsMode = download; updateDeployButton();});
  rowLayout->addWidget(downloadSelector);

  QRadioButton* deleteSelector = new QRadioButton("Delete");
  deleteSelector->setChecked(logsMode == justDelete);
  deleteSelector->setFocusPolicy(Qt::StrongFocus);
  connect(deleteSelector, &QRadioButton::toggled, [=, this](bool checked) {if(checked) logsMode = justDelete; updateDeployButton();});
  rowLayout->addWidget(deleteSelector);

  // Hack: Without this, the buttons are not aligned correctly
  rowLayout = new QHBoxLayout();
  layout->addLayout(rowLayout);

  QRadioButton* downloadAndDeleteSelector = new QRadioButton("Download and delete");
  downloadAndDeleteSelector->setChecked(logsMode == downloadAndDelete);
  downloadAndDeleteSelector->setFocusPolicy(Qt::StrongFocus);
  connect(downloadAndDeleteSelector, &QRadioButton::toggled, [=, this](bool checked) {if(checked) logsMode = downloadAndDelete; updateDeployButton();});
  rowLayout->addWidget(downloadAndDeleteSelector);

  return widget;
}

void SettingsArea::writeOutput(std::map<std::string, Robot>& robots) const
{
  if(mode == logs)
  {
    std::cout << "logs ";
    table->writeOutput(robots, true);
    if(logsMode == downloadAndDelete)
      std::cout << "-d";
    else if(logsMode == justDelete)
      std::cout << "-D";
    std::cout << std::endl;
    return;
  }
  else if(mode == image)
    std::cout << "-i -p " << playerNumber << " ";
  else
    table->writeOutput(robots, false);

  std::cout <<  "-nc"
            << " -t " << selectedPreset->number
            << " -c " << selectedPreset->fieldPlayerColor
            << " -g " << selectedPreset->goalkeeperColor
            << " -s " << selectedPreset->scenario
            << " -l " << selectedPreset->location
            << " -m " << selectedPreset->magicNumber
            << " -w " << selectedPreset->wlanConfig
            << " -v " << selectedPreset->volume;

  if(mode == image)
  {
    if(usbCheck)
      std::cout << " -u";
    if(date)
      std::cout << " -d";
    if(reboot)
      std::cout << " -b";
  }
  else
  {
    if(deleteLogs)
      std::cout << " -d";
    if(restart)
      std::cout << " -b";
  }
  std::cout << std::endl;
}

bool SettingsArea::modified(const QSettings& settings) const
{
  return presetIndex != std::min(static_cast<int>(presets.teams.size()) - 1, settings.value("presetIndex", 0).toInt())
         || mode != settings.value("mode", false).toInt()
         || restart != settings.value("restart", true).toBool()
         || deleteLogs != settings.value("deleteLogs", false).toBool()
         || playerNumber != settings.value("playerNumber", 5).toInt()
         || reboot != settings.value("reboot", false).toBool()
         || usbCheck != settings.value("usbCheck", false).toBool()
         || date != settings.value("date", false).toBool()
         || logsMode != settings.value("logsMode", download).toInt()
         || close != settings.value("close", true).toBool();
}

bool SettingsArea::save(QSettings& settings) const
{
  settings.setValue("presetIndex", presetIndex);
  settings.setValue("mode", mode);
  settings.setValue("restart", restart);
  settings.setValue("deleteLogs", deleteLogs);
  settings.setValue("playerNumber", playerNumber);
  settings.setValue("reboot", reboot);
  settings.setValue("usbCheck", usbCheck);
  settings.setValue("date", date);
  settings.setValue("logsMode", logsMode);
  settings.setValue("close", close);
  return close;
}
