#include "FootViewWidget.h"
#include "FootWidget.h"
#include "Controller/RobotConsole.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Sensing/FootGroundContactState.h"

#include <QBoxLayout>
#include <QProgressBar>

FootViewWidget::FootViewWidget(RobotConsole& robotConsole, const FootGroundContactState& footGroundContactState,
                               const JointSensorData& jointSensorData, const RobotDimensions& robotDimensions) :
  robotConsole(robotConsole), footGroundContactState(footGroundContactState),
  jointSensorData(jointSensorData), robotDimensions(robotDimensions)
{
  QVBoxLayout* vbox = new QVBoxLayout();
  setLayout(vbox);

  //QHBoxLayout* upperHLayout = new QHBoxLayout();
  QHBoxLayout* lowerHLayout = new QHBoxLayout();
  //vbox->addLayout(upperHLayout);
  vbox->addLayout(lowerHLayout);

  leftTotalWeightIndicator = new QProgressBar();
  leftTotalWeightIndicator->setOrientation(Qt::Orientation::Vertical);
  leftTotalWeightIndicator->setTextVisible(false);
  leftTotalWeightIndicator->setMinimum(0);
  leftTotalWeightIndicator->setStyleSheet("QProgressBar::chunk { background-color: red; }");
  leftTotalWeightIndicator->setFixedWidth(10);

  rightTotalWeightIndicator = new QProgressBar();
  rightTotalWeightIndicator->setOrientation(Qt::Orientation::Vertical);
  rightTotalWeightIndicator->setTextVisible(false);
  rightTotalWeightIndicator->setMinimum(0);
  rightTotalWeightIndicator->setStyleSheet("QProgressBar::chunk { background-color: blue; }");
  rightTotalWeightIndicator->setFixedWidth(10);

  lowerHLayout->addWidget(leftTotalWeightIndicator);
  lowerHLayout->addWidget(new FootWidget(*this));
  lowerHLayout->addWidget(rightTotalWeightIndicator);
}

void FootViewWidget::update()
{
  if(needsRepaint())
  {
    updateTotalWeightIndicators();

    QWidget::update();
  }
}

bool FootViewWidget::needsRepaint()
{
  SYNC_WITH(robotConsole);
  bool needsRepaint = jointSensorData.timestamp != lastUpdateTimeStamp;
  lastUpdateTimeStamp = jointSensorData.timestamp;
  return needsRepaint;
}

void FootViewWidget::updateTotalWeightIndicators()
{
  leftTotalWeightIndicator->setMaximum(static_cast<int>(std::round(footGroundContactState.maxTotal * 1000.f)));
  rightTotalWeightIndicator->setMaximum(static_cast<int>(std::round(footGroundContactState.maxTotal * 1000.f)));

  leftTotalWeightIndicator->setValue(static_cast<int>(std::round(footGroundContactState.totalMass[FootGroundContactState::left] * 1000.f)));
  rightTotalWeightIndicator->setValue(static_cast<int>(std::round(footGroundContactState.totalMass[FootGroundContactState::right] * 1000.f)));
}
