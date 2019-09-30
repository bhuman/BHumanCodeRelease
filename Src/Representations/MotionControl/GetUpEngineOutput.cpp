/**
 * @file Representations/MotionControl/GetUpEngineOutput.cpp
 * @author Philip Reichenberg
 */

#include "GetUpEngineOutput.h"
#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void GetUpEngineOutput::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:GetUpEngineOutput:coordinates", "robot");
  Pose3f comInSupport = gyroInTorso.inverse() * comInTorso;
  comInSupport.translation.z() = supportCenterInTorso.translation.z();
  comInSupport = gyroInTorso * comInSupport;
  LINE3D("representation:GetUpEngineOutput:coordinates", comInSupport.translation.x(), comInSupport.translation.y(), comInSupport.translation.z(), comInTorso.translation.x(), comInTorso.translation.y(), comInTorso.translation.z(), 4, ColorRGBA::red);
  std::vector<Pose3f> limbsInTorso;
  for(const Pose3f& l : supportPolygon)
    limbsInTorso.push_back(gyroInTorso * l);
  for(unsigned i = 0; i < limbsInTorso.size(); i++)
    LINE3D("representation:GetUpEngineOutput:coordinates", limbsInTorso[i].translation.x(), limbsInTorso[i].translation.y(), limbsInTorso[i].translation.z(), limbsInTorso[(i + 1) % limbsInTorso.size()].translation.x(), limbsInTorso[(i + 1) % limbsInTorso.size()].translation.y(), limbsInTorso[(i + 1) % limbsInTorso.size()].translation.z(), 4, ColorRGBA::blue);
  CROSS3D("representation:GetUpEngineOutput:coordinates", supportCenterInTorso.translation.x(), supportCenterInTorso.translation.y(), supportCenterInTorso.translation.z(), 3, 5, ColorRGBA::orange);
}
