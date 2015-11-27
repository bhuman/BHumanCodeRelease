/**
* @file SimObjectRenderer.h
* Declaration of class SimObjectRenderer
* @author Colin Graf
*/

#pragma once

#include "SimRobotCore2.h"
#include "Tools/Vector3.h"

class SimObject;
class Body;

/**
* @class SimObjectRenderer
* An interface for rendering scene objects on an OpenGL context
*/
class SimObjectRenderer : public SimRobotCore2::Renderer
{
public:
  /**
  * Constructor
  * @param simObject The object to render
  */
  SimObjectRenderer(SimObject& simObject);

  /** Destructor */
  virtual ~SimObjectRenderer() = default;

private:
  SimObject& simObject;
  unsigned int width;
  unsigned int height;

  CameraMode cameraMode;
  Vector3<> defaultCameraPos;
  Vector3<> cameraPos;
  Vector3<> cameraTarget;
  float cameraTransformation[16];
  float fovy;
  float projection[16];
  int viewport[4];

  ShadeMode surfaceShadeMode;
  ShadeMode physicsShadeMode;
  ShadeMode drawingsShadeMode;
  unsigned int renderFlags;

  bool dragging;
  DragType dragType;
  Body* dragSelection;
  Vector3<> dragStartPos;
  DragAndDropPlane dragPlane;
  Vector3<> dragPlaneVector;
  DragAndDropMode dragMode;
  unsigned int dragStartTime;

  bool moving;
  unsigned int movingLeftStartTime;
  unsigned int movingRightStartTime;
  unsigned int movingUpStartTime;
  unsigned int movingDownStartTime;

  void moveCamera(float x, float y);
  void updateCameraTransformation();

  bool intersectRayAndPlane(const Vector3<>& point, const Vector3<>& v,
                                       const Vector3<>& plane, const Vector3<>& n,
                                       Vector3<>& intersection) const;
  Vector3<> projectClick(int x, int y);
  Body* selectObject(const Vector3<>& projectedClick);

public:
  // API
  virtual void init(bool hasSharedDisplayLists);
  virtual void draw();
  virtual void resize(float fovy, unsigned int width, unsigned int height);
  virtual void getSize(unsigned int& width, unsigned int& height) const;
  virtual void setSurfaceShadeMode(ShadeMode shadeMode) {surfaceShadeMode = shadeMode;}
  virtual ShadeMode getSurfaceShadeMode() const {return surfaceShadeMode;}
  virtual void setPhysicsShadeMode(ShadeMode shadeMode) {physicsShadeMode = shadeMode;}
  virtual ShadeMode getPhysicsShadeMode() const {return physicsShadeMode;}
  virtual void setDrawingsShadeMode(ShadeMode shadeMode) {drawingsShadeMode = shadeMode;}
  virtual ShadeMode getDrawingsShadeMode() const {return drawingsShadeMode;}
  virtual void zoom(float change);
  virtual void setRenderFlags(unsigned int renderFlags) {this->renderFlags = renderFlags;}
  virtual unsigned int getRenderFlags() const {return renderFlags;}
  virtual void setCameraMode(CameraMode mode);
  virtual CameraMode getCameraMode() const {return cameraMode;}
  virtual void toggleCameraMode();
  virtual void resetCamera();
  virtual void fitCamera();
  virtual int getFovY() const {return int(fovy);}
  virtual void setDragPlane(DragAndDropPlane plane) {dragPlane = plane;}
  virtual DragAndDropPlane getDragPlane() const {return dragPlane;}
  virtual void setDragMode(DragAndDropMode mode) {dragMode = mode;}
  virtual DragAndDropMode getDragMode() const {return dragMode;}
  virtual bool startDrag(int x, int y, DragType type);
  virtual SimRobotCore2::Object* getDragSelection();
  virtual void setCameraMove(bool left, bool right, bool up, bool down);
  virtual bool moveDrag(int x, int y);
  virtual bool releaseDrag(int x, int y);
  virtual void setCamera(const float* pos, const float* target);
  virtual void getCamera(float* pos, float* target);
  virtual void rotateCamera(float x, float y);
};
