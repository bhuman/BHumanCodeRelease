#include "FsrDataProvider.h"

MAKE_MODULE(FsrDataProvider, Sensing)

FsrDataProvider::FsrDataProvider()
{
  LFsrFL = Vector2<>(70.25f, 29.9f);
  LFsrFR = Vector2<>(70.25f, -23.1f);
  LFsrRL = Vector2<>(-30.25f, 29.9f);
  LFsrRR = Vector2<>(-29.65f, -19.1f);
  RFsrFL = Vector2<>(70.25f, 23.1f);
  RFsrFR = Vector2<>(70.25f, -29.9f);
  RFsrRL = Vector2<>(-30.25f, 19.1f);
  RFsrRR = Vector2<>(-29.65f, -29.9f);
}

void FsrDataProvider::update(FsrData& fsrData)
{
  float leftSum = 0;
  leftSum += theSensorData.data[SensorData::fsrLBL];
  leftSum += theSensorData.data[SensorData::fsrLBR];
  leftSum += theSensorData.data[SensorData::fsrLFL];
  leftSum += theSensorData.data[SensorData::fsrLFR];
  fsrL.add(leftSum);

  if(fsrL.isFilled() && fsrL.getAverageFloat() + fsrWeightOffset >= theRobotModel.totalMass) fsrData.leftFootContact = true;
  else fsrData.leftFootContact = false;

  float rightSum = 0;
  rightSum += theSensorData.data[SensorData::fsrRBL];
  rightSum += theSensorData.data[SensorData::fsrRBR];
  rightSum += theSensorData.data[SensorData::fsrRFL];
  rightSum += theSensorData.data[SensorData::fsrRFR];
  fsrR.add(rightSum);

  if(fsrR.isFilled() && fsrR.getAverageFloat() + fsrWeightOffset >= theRobotModel.totalMass) fsrData.rightFootContact = true;
  else fsrData.rightFootContact = false;

  if(fsrData.leftFootContact == true && fsrData.rightFootContact == false)
  {
    fsrData.centerOfPressure = LFsrFL * theSensorData.data[SensorData::fsrLFL] +
    LFsrFR * theSensorData.data[SensorData::fsrLFR] +
    LFsrRL * theSensorData.data[SensorData::fsrLBL] +
    LFsrRR * theSensorData.data[SensorData::fsrLBR];

    fsrData.centerOfPressure /= leftSum;
  }
  else if(fsrData.leftFootContact == false && fsrData.rightFootContact == true)
  {
    fsrData.centerOfPressure = RFsrFL * theSensorData.data[SensorData::fsrRFL] +
    RFsrFR * theSensorData.data[SensorData::fsrRFR] +
    RFsrRL * theSensorData.data[SensorData::fsrRBL] +
    RFsrRR * theSensorData.data[SensorData::fsrRBR];

     fsrData.centerOfPressure /= rightSum;
  }



}