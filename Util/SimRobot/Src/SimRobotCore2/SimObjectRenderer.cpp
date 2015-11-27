/**
* @file SimObjectRenderer.cpp
* Declaration of class SimObjectRenderer
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "SimObjectRenderer.h"
#include "Simulation/Simulation.h"
#include "Simulation/Body.h"
#include "Simulation/Scene.h"
#include "Platform/System.h"
#include "Platform/Assert.h"
#include "Tools/OpenGLTools.h"

template <class V> inline V normalize(const V& data)
{
  const float pi = 3.1415926535897932384626433832795f;
  const float pi2 = 2.0f * pi;
  if(data < V(pi) && data >= -V(pi))
    return data;
  V ndata = data - ((int)(data / V(pi2))) * V(pi2);
  if(ndata >= V(pi))
    ndata -= V(pi2);
  else if(ndata < -V(pi))
    ndata += V(pi2);
  return ndata;
}

SimObjectRenderer::SimObjectRenderer(SimObject& simObject) :
  simObject(simObject), width(0), height(0),
  cameraMode(targetCam), defaultCameraPos(3.f, 6.f, 4.f), cameraPos(defaultCameraPos), fovy(40.f),
  surfaceShadeMode(smoothShading), physicsShadeMode(noShading), drawingsShadeMode(smoothShading), renderFlags(enableLights | enableTextures | showCoordinateSystem),
  dragging(false), dragPlane(xyPlane), dragMode(keepDynamics),
  moving(false), movingLeftStartTime(0), movingRightStartTime(0), movingUpStartTime(0), movingDownStartTime(0)
{
}

void SimObjectRenderer::resetCamera()
{
  cameraPos = defaultCameraPos;
  cameraTarget = Vector3<>();
  updateCameraTransformation();
}

void SimObjectRenderer::updateCameraTransformation()
{
  static const Vector3<> cameraUpVector(0.f, 0.f, 1.f);
  OpenGLTools::computeCameraTransformation(cameraPos, cameraTarget, cameraUpVector, cameraTransformation);
}

void SimObjectRenderer::init(bool hasSharedDisplayLists)
{
#ifdef OSX
  CGLContextObj ctx = CGLGetCurrentContext();
  VERIFY(CGLEnable(ctx, kCGLCEMPEngine) == kCGLNoError);
#endif

  Simulation::simulation->scene->createGraphics(hasSharedDisplayLists);
}

void SimObjectRenderer::draw()
{
  // update a moving camera
  if(moving)
  {
    unsigned int now = System::getTime();
    int x = 0;
    int y = 0;
    if(movingLeftStartTime)
    {
      x -= now - movingLeftStartTime;
      movingLeftStartTime = now;
    }
    if(movingRightStartTime)
    {
      x += now - movingRightStartTime;
      movingRightStartTime = now;
    }
    if(movingUpStartTime)
    {
      y -= now - movingUpStartTime;
      movingUpStartTime = now;
    }
    if(movingDownStartTime)
    {
      y += now - movingDownStartTime;
      movingDownStartTime = now;
    }
    moveCamera(float(x) * 0.001f, float(y) * 0.002f);
  }

  // set flags
  if(renderFlags & enableLights)
    glEnable(GL_LIGHTING);
  else
    glDisable(GL_LIGHTING);
  if(renderFlags & enableMultisample)
    glEnable(GL_MULTISAMPLE);
  else
    glDisable(GL_MULTISAMPLE);
  if(renderFlags & enableTextures)
    glEnable(GL_TEXTURE_2D);
  else
    glDisable(GL_TEXTURE_2D);

  // clear
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // load camera position
  glLoadMatrixf(cameraTransformation);

  // make sure transformations of movable bodies are up-to-date
  // note: a not-physical-object has a constant offset pose relative its parent. hence there is no transformation update required to draw not-physical-objects
  PhysicalObject* physicalObject = dynamic_cast<PhysicalObject*>(&simObject);
  GraphicalObject* graphicalObject = dynamic_cast<GraphicalObject*>(&simObject);
  if(physicalObject)
    Simulation::simulation->scene->updateTransformations();

  // since each object will be drawn relative to its parent we need to shift the coordinate system when we want the object to be in the center
  if(&simObject != Simulation::simulation->scene && !(renderFlags & showAsGlobalView))
  {
    const float* transformation = simObject.transformation;
    Pose3<> pose(Matrix3x3<>(Vector3<>(transformation[0], transformation[1], transformation[2]),
      Vector3<>(transformation[4], transformation[5], transformation[6]),
      Vector3<>(transformation[8], transformation[9], transformation[10])),
      Vector3<>(transformation[12], transformation[13], transformation[14]));
    float invTrans[16];
    OpenGLTools::convertTransformation(pose.invert(), invTrans);
    glMultMatrixf(invTrans);
  }

  // draw origin
  if(renderFlags & showCoordinateSystem)
  {
    Simulation::simulation->scene->defaultSurface->set();

    glBegin(GL_LINES);
      glNormal3f (0,0,1);
      glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(1, 0, 0);
      glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 1, 0);
      glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1);
    glEnd();

    Simulation::simulation->scene->defaultSurface->unset();
  }

  // draw object / scene appearance
  if(graphicalObject && surfaceShadeMode != noShading)
  {
    switch(surfaceShadeMode)
    {
      case flatShading:
        glPolygonMode(GL_FRONT, GL_FILL);
        glShadeModel(GL_FLAT);
        break;
      case wireframeShading:
        glPolygonMode(GL_FRONT, GL_LINE);
        glShadeModel(GL_FLAT);
        break;
      case smoothShading:
        glPolygonMode(GL_FRONT, GL_FILL);
        glShadeModel(GL_SMOOTH);
        break;
      default:
        ASSERT(false);
        break;
    }
    graphicalObject->drawAppearances();

    // check matrix stack size
#ifdef _DEBUG
    int stackDepth;
    glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &stackDepth);
    ASSERT(stackDepth == 1);
#endif
  }

  // draw object / scene physics
  if(physicalObject && (physicsShadeMode != noShading || renderFlags & showSensors))
  {
    Simulation::simulation->scene->defaultSurface->set();

    unsigned int renderFlags = (this->renderFlags | showPhysics) & ~showControllerDrawings;
    switch(physicsShadeMode)
    {
      case noShading:
        glPolygonMode(GL_FRONT, GL_LINE);
        glShadeModel(GL_FLAT);
        renderFlags &= ~showPhysics;
        break;
      case flatShading:
        glPolygonMode(GL_FRONT, GL_FILL);
        glShadeModel(GL_FLAT);
        break;
      case wireframeShading:
        glPolygonMode(GL_FRONT, GL_LINE);
        glShadeModel(GL_FLAT);
        break;
      case smoothShading:
        glPolygonMode(GL_FRONT, GL_FILL);
        glShadeModel(GL_SMOOTH);
        break;
      default:
        ASSERT(false);
        break;
    }
    physicalObject->drawPhysics(renderFlags);

    Simulation::simulation->scene->defaultSurface->unset();

    // check matrix stack size
#ifdef _DEBUG
    int stackDepth;
    glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &stackDepth);
    ASSERT(stackDepth == 1);
#endif
  }

  // draw drag plane
  if(dragging && dragSelection)
  {
    Simulation::simulation->scene->defaultSurface->set();

    glPolygonMode(GL_FRONT, GL_FILL);
    glShadeModel(GL_FLAT);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_SRC_ALPHA);

    glPushMatrix();
    if(dragType == dragRotate || dragType == dragNormalObject)
      glMultMatrixf(dragSelection->transformation);
    else
      glTranslatef(dragSelection->pose.translation.x, dragSelection->pose.translation.y, dragSelection->pose.translation.z);

    switch(dragPlane)
    {
      case xyPlane:
        break; // do nothing
      case xzPlane:
        glRotatef(90.f, 1.f, 0.f, 0.f);
        break;
      case yzPlane:
        glRotatef(90.f, 0.f, 1.f, 0.f);
        break;
    }

    GLUquadricObj* q = gluNewQuadric();
    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    glNormal3f(0,0,1);
    gluDisk(q, 0.003f, 0.5f, 30, 1);
    glRotatef(180.f, 1.f, 0.f, 0.f);
    gluDisk(q, 0.003f, 0.5f, 30, 1);
    gluDeleteQuadric(q);
    glPopMatrix();

    glDisable(GL_BLEND);

    Simulation::simulation->scene->defaultSurface->unset();
  }

  // draw controller drawings
  if(physicalObject && drawingsShadeMode != noShading)
  {
    Simulation::simulation->scene->defaultSurface->set();

    switch(drawingsShadeMode)
    {
      case flatShading:
        glPolygonMode(GL_FRONT, GL_FILL);
        glShadeModel(GL_FLAT);
        break;
      case wireframeShading:
        glPolygonMode(GL_FRONT, GL_LINE);
        glShadeModel(GL_FLAT);
        break;
      case smoothShading:
        glPolygonMode(GL_FRONT, GL_FILL);
        glShadeModel(GL_SMOOTH);
        break;
      default:
        ASSERT(false);
        break;
    }

    if(renderFlags & (enableDrawingsTransparentOcclusion | enableDrawingsOcclusion))
    {
      physicalObject->drawPhysics(showControllerDrawings);
      if(renderFlags & enableDrawingsTransparentOcclusion)
      {
        glAccum(GL_LOAD, 0.5f);
        glClear(GL_DEPTH_BUFFER_BIT);
        physicalObject->drawPhysics(showControllerDrawings);
        glAccum(GL_ACCUM, 0.5f);
        glAccum(GL_RETURN, 1.f);
      }
    }
    else
    {
      glClear(GL_DEPTH_BUFFER_BIT);
      physicalObject->drawPhysics(showControllerDrawings);
    }

    Simulation::simulation->scene->defaultSurface->unset();

    // check matrix stack size
#ifdef _DEBUG
    int stackDepth;
    glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &stackDepth);
    ASSERT(stackDepth == 1);
#endif
  }
}

void SimObjectRenderer::resize(float fovy, unsigned int width, unsigned int height)
{
  this->fovy = fovy;
  this->width = width;
  this->height = height;

  glViewport(0, 0, width, height);
  viewport[0] = viewport[1] = 0; viewport[2] = width; viewport[3] = height; // store viewport for gluUnProject

  glMatrixMode(GL_PROJECTION);
  OpenGLTools::computePerspective(fovy * (float(M_PI) / 180.f), float(width) / float(height), 0.1f, 500.f, projection);
  glLoadMatrixf(projection);

  glMatrixMode(GL_MODELVIEW);
}

Vector3<> SimObjectRenderer::projectClick(int x, int y)
{
  GLdouble mvmatrix[16];
  GLdouble projmatrix[16];
  for(int i = 0; i < 16; ++i)
  {
    mvmatrix[i] = (GLdouble) cameraTransformation[i];
    projmatrix[i] = (GLdouble) projection[i];
  }

  GLdouble tx, ty, tz;
  gluUnProject((GLdouble)(x),(GLdouble)(height - y), 1.0, mvmatrix, projmatrix, viewport, &tx, &ty, &tz);
  return Vector3<>(float(tx), float(ty), float(tz));
}

void SimObjectRenderer::rotateCamera(float x, float y)
{
  if(cameraMode == SimRobotCore2::Renderer::targetCam)
  {
    Vector3<> v = cameraPos - cameraTarget;
    Matrix3x3<> rotateY(Vector3<>(0.f, y, 0.f));
    Matrix3x3<> rotateZ(Vector3<>(0.f, 0.f, x));
    Vector3<> v2(sqrtf(v.x * v.x + v.y * v.y), 0.f, v.z);
    v2 = rotateY * v2;
    if(v2.x < 0.001f)
    {
      v2.x = 0.001f;
      v2.normalize(v.abs());
    }
    Vector3<> v3(v.x, v.y, 0.f);
    v3.normalize(v2.x);
    v3.z = v2.z;
    v = rotateZ * v3;
    cameraPos = cameraTarget + v;
  }
  else // if(cameraMode == SimRobotCore2::Renderer::FREECAM)
  {
    Vector3<> v = cameraTarget - cameraPos;
    Matrix3x3<> rotateY(Vector3<>(0.f, y, 0.f));
    Matrix3x3<> rotateZ(Vector3<>(0.f, 0.f, x));
    Vector3<> v2(sqrtf(v.x * v.x + v.y * v.y), 0.f, v.z);
    v2 = rotateY * v2;
    if(v2.x < 0.001f)
    {
      v2.x = 0.001f;
      v2.normalize(v.abs());
    }
    Vector3<> v3(v.x, v.y, 0.f);
    v3.normalize(v2.x);
    v3.z = v2.z;
    v = rotateZ * v3;
    cameraTarget = cameraPos + v;
  }
  updateCameraTransformation();
}

void SimObjectRenderer::moveCamera(float x, float y)
{
  if(cameraMode == SimRobotCore2::Renderer::targetCam)
  {
    if(x != 0.f)
      rotateCamera(x, 0);
    if(y != 0.f)
    {
      Vector3<> v = cameraPos - cameraTarget;
      float len = v.abs() + y;
      if(len < 0.0001f)
        len = 0.0001f;
      v.normalize(len);
      cameraPos = cameraTarget + v;
    }
  }
  else // if(cameraMode == SimRobotCore2::Renderer::FREECAM)
  {
    if(x != 0.f)
    {
      Vector3<> v = cameraTarget - cameraPos;
      Vector3<> v2 = v ^ Vector3<>(0.f, 0.f, 1.f);
      v2.normalize(x);
      cameraTarget += v2;
      cameraPos += v2;
    }
    if(y != 0.f)
    {
      Vector3<> v = cameraTarget - cameraPos;
      v.normalize(y);
      cameraTarget -= v;
      cameraPos -= v;
    }
  }
  updateCameraTransformation();
}

void SimObjectRenderer::getSize(unsigned int& width, unsigned int& height) const
{
  width = this->width;
  height = this->height;
}

void SimObjectRenderer::zoom(float change)
{
  moveCamera(0.f, change * 0.001f);
}

void SimObjectRenderer::setCameraMode(SimRobotCore2::Renderer::CameraMode mode)
{
  cameraMode = mode;
  if(cameraMode == targetCam)
  {
    cameraTarget = Vector3<>();
    updateCameraTransformation();
  }
}

void SimObjectRenderer::toggleCameraMode()
{
  setCameraMode(cameraMode == SimRobotCore2::Renderer::targetCam ? SimRobotCore2::Renderer::freeCam : SimRobotCore2::Renderer::targetCam);
}

void SimObjectRenderer::fitCamera()
{
  /*
  if(simObject)
    simulation->fitCamera(cameraTarget, cameraPos, width, height, simObject);
  calculateUpVector();
  */
}

bool SimObjectRenderer::intersectRayAndPlane(const Vector3<>& point, const Vector3<>& v,
                                       const Vector3<>& plane, const Vector3<>& n,
                                       Vector3<>& intersection) const
{
  Vector3<> p = plane - point;
  float denominator = n * v;
  if(denominator == 0.f)
    return false;
  float r = n * p / denominator;
  if(r < 0.f)
    return false;
  intersection = v;
  intersection *= r;
  intersection += point;
  return true;
}

Body* SimObjectRenderer::selectObject(const Vector3<>& projectedClick)
{
  if(&simObject != Simulation::simulation->scene)
    return 0;

  class Callback
  {
  public:
    Body* closestBody;
    float closestSqrDistance;
    const Vector3<>& cameraPos;

    Callback(const Vector3<>& cameraPos) : closestBody(0), cameraPos(cameraPos) {}

    static void staticCollisionCallback(Callback* callback, dGeomID geom1, dGeomID geom2)
    {
      ASSERT(!dGeomIsSpace(geom1));
      ASSERT(!dGeomIsSpace(geom2));
      ASSERT(dGeomGetBody(geom1) || dGeomGetBody(geom2));
      dContact contact[1];
      if(dCollide(geom1, geom2, 1, &contact[0].geom, sizeof(dContact)) < 1)
        return;

      dGeomID geom = geom2;
      dBodyID bodyId = dGeomGetBody(geom2);
      if(!bodyId)
      {
        bodyId = dGeomGetBody(geom1);
        geom = geom1;
      }
      const dReal* pos = dGeomGetPosition(geom);
      float sqrDistance = (Vector3<>((float) pos[0], (float) pos[1], (float) pos[2]) - callback->cameraPos).squareAbs();
      if(!callback->closestBody || sqrDistance < callback->closestSqrDistance)
      {
        callback->closestBody = (Body*)dBodyGetData(bodyId);
        callback->closestSqrDistance = sqrDistance;
      }
    }

    static void staticCollisionWithSpaceCallback(Callback* callback, dGeomID geom1, dGeomID geom2)
    {
      ASSERT(!dGeomIsSpace(geom1));
      ASSERT(dGeomIsSpace(geom2));
      dSpaceCollide2(geom1, geom2, callback, (dNearCallback*)&staticCollisionCallback);
    }
  };

  Callback callback(cameraPos);
  dGeomID ray = dCreateRay(Simulation::simulation->staticSpace, 10000.f);
  Vector3<> dir = projectedClick - cameraPos;
  dGeomRaySet(ray, cameraPos.x, cameraPos.y, cameraPos.z, dir.x, dir.y, dir.z);
  dSpaceCollide2(ray, (dGeomID)Simulation::simulation->movableSpace, &callback, (dNearCallback*)&Callback::staticCollisionWithSpaceCallback);
  dGeomDestroy(ray);

  if(!callback.closestBody)
    return 0;
  Body* body = callback.closestBody;
  return body->rootBody;
}

bool SimObjectRenderer::startDrag(int x, int y, DragType type)
{
  if(dragging)
    return true;

  // look if the user clicked on an object
  dragSelection = 0;
  if(&simObject == Simulation::simulation->scene)
  {
    Vector3<> projectedClick = projectClick(x, y);
    dragSelection = selectObject(projectedClick);

    if(dragSelection)
    {
      switch(dragPlane)
      {
        case xyPlane: dragPlaneVector = Vector3<>(0.f, 0.f, 1.f); break;
        case xzPlane: dragPlaneVector = Vector3<>(0.f, 1.f, 0.f); break;
        case yzPlane: dragPlaneVector = Vector3<>(1.f, 0.f, 0.f); break;
      }
      if(type == dragRotate || type == dragNormalObject)
        dragPlaneVector = dragSelection->pose.rotation * dragPlaneVector;
      if(!intersectRayAndPlane(cameraPos, projectedClick - cameraPos, dragSelection->pose.translation, dragPlaneVector, dragStartPos))
        dragSelection = 0;
      else
      {
        dragSelection->enablePhysics(false);
        if(dragMode == resetDynamics)
          dragSelection->resetDynamics();

        dragging = true;
        dragType = type;
        if(dragMode == adoptDynamics)
          dragStartTime = System::getTime();
        return true;
      }
    }
  }

  if(!dragSelection) // camera control
  {
    dragStartPos.x = x;
    dragStartPos.y = y;
    dragging = true;
    dragType = type;
    return true;
  }
  return false;
}

SimRobotCore2::Object* SimObjectRenderer::getDragSelection()
{
  return dragSelection;
}

bool SimObjectRenderer::moveDrag(int x, int y)
{
  if(!dragging)
    return false;

  if(!dragSelection) // camera control
  {
    if(cameraMode == SimRobotCore2::Renderer::targetCam)
    {
      if(dragType == dragRotate || dragType == dragRotateWorld)
      {
      }
      else // if(dragType == DRAG_NORMAL)
        rotateCamera((x - dragStartPos.x) * -0.01f, (y - dragStartPos.y) * -0.01f);
    }
    else // if(cameraMode == SimRobotCore2::Renderer::FREECAM)
    {
      if(dragType == dragRotate || dragType == dragRotateWorld)
      {
      }
      else // if(dragType == DRAG_NORMAL)
        rotateCamera((x - dragStartPos.x) * -0.001f, (y - dragStartPos.y) * 0.001f);
    }

    dragStartPos.x = x;
    dragStartPos.y = y;
    return true;
  }
  else // object control
  {
    if(dragMode == applyDynamics)
      return true;
    Vector3<> projectedClick = projectClick(x, y);
    Vector3<> currentPos;
    if(intersectRayAndPlane(cameraPos, projectedClick - cameraPos, dragSelection->pose.translation, dragPlaneVector, currentPos))
    {
      if(dragType == dragRotate || dragType == dragRotateWorld)
      {
        Vector3<> oldv = dragStartPos - dragSelection->pose.translation;
        Vector3<> newv = currentPos - dragSelection->pose.translation;

        if(dragType != dragRotateWorld)
        {
          Matrix3x3<> invRotation = dragSelection->pose.rotation.transpose();
          oldv = invRotation * oldv;
          newv = invRotation * newv;
        }

        float angle = 0.f;
        if(dragPlane == yzPlane)
          angle = normalize(atan2f(newv.z, newv.y) - atan2f(oldv.z, oldv.y));
        else if(dragPlane == xzPlane)
          angle = normalize(atan2f(newv.x, newv.z) - atan2f(oldv.x, oldv.z));
        else
          angle = normalize(atan2f(newv.y, newv.x) - atan2f(oldv.y, oldv.x));

        Vector3<> offset = dragPlaneVector * angle;
        Matrix3x3<> rotation(offset);
        Vector3<> center = dragSelection->pose.translation;
        dragSelection->rotate(rotation, center);
        if(dragMode == adoptDynamics)
        {
          unsigned int now = System::getTime();
          float t = (now - dragStartTime) * 0.001f;
          Vector3<> velocity = offset / t;
          const dReal* oldVel = dBodyGetAngularVel(dragSelection->body);
          velocity = velocity * 0.3f + Vector3<>((float) oldVel[0], (float) oldVel[1], (float) oldVel[2]) * 0.7f;
          dBodySetAngularVel(dragSelection->body, velocity.x, velocity.y, velocity.z);
          dragStartTime = now;
        }
        dragStartPos = currentPos;
      }
      else
      {
        const Vector3<> offset = currentPos - dragStartPos;
        dragSelection->move(offset);
        if(dragMode == adoptDynamics)
        {
          unsigned int now = System::getTime();
          float t = (now - dragStartTime) * 0.001f;
          Vector3<> velocity = offset / t;
          const dReal* oldVel = dBodyGetLinearVel(dragSelection->body);
          velocity = velocity * 0.3f + Vector3<>((float) oldVel[0], (float) oldVel[1], (float) oldVel[2]) * 0.7f;
          dBodySetLinearVel(dragSelection->body, velocity.x, velocity.y, velocity.z);
          dragStartTime = now;
        }
        dragStartPos = currentPos;
      }
    }
    return true;
  }
}

bool SimObjectRenderer::releaseDrag(int x, int y)
{
  if(!dragging)
    return false;

  if(!dragSelection) // camera control
  {
    dragging = false;
    return true;
  }
  else // object control
  {
    if(dragMode == adoptDynamics)
      moveDrag(x, y);
    else if(dragMode == applyDynamics)
    {
      Vector3<> projectedClick = projectClick(x, y);
      Vector3<> currentPos;
      if(intersectRayAndPlane(cameraPos, projectedClick - cameraPos, dragSelection->pose.translation, dragPlaneVector, currentPos))
      {
        if(dragType == dragRotate || dragType == dragRotateWorld)
        {
          Vector3<> oldv = dragStartPos - dragSelection->pose.translation;
          Vector3<> newv = currentPos - dragSelection->pose.translation;

          if(dragType != dragRotateWorld)
          {
            Matrix3x3<> invRotation = dragSelection->pose.rotation.transpose();
            oldv = invRotation * oldv;
            newv = invRotation * newv;
          }

          float angle = 0.f;
          if(dragPlane == yzPlane)
            angle = normalize(atan2f(newv.z, newv.y) - atan2f(oldv.z, oldv.y));
          else if(dragPlane == xzPlane)
            angle = normalize(atan2f(newv.x, newv.z) - atan2f(oldv.x, oldv.z));
          else
            angle = normalize(atan2f(newv.y, newv.x) - atan2f(oldv.y, oldv.x));

          Vector3<> offset = dragPlaneVector * angle;
          Vector3<> torque = offset * (float) dragSelection->mass.mass * 50.f;
          dBodyAddTorque(dragSelection->body, torque.x, torque.y, torque.z);
        }
        else
        {
          const Vector3<> offset = currentPos - dragStartPos;
          Vector3<> force = offset * (float) dragSelection->mass.mass * 500.f;
          dBodyAddForce(dragSelection->body, force.x, force.y, force.z);
        }
      }
    }

    dragSelection->enablePhysics(true);

    dragging = false;
    return true;
  }
}

void SimObjectRenderer::setCameraMove(bool left, bool right, bool up, bool down)
{
  if(!left)
    movingLeftStartTime = 0;
  else if(!movingLeftStartTime)
    movingLeftStartTime = System::getTime();

  if(!right)
    movingRightStartTime = 0;
  else if(!movingRightStartTime)
    movingRightStartTime = System::getTime();

  if(!up)
    movingUpStartTime = 0;
  else if(!movingUpStartTime)
    movingUpStartTime = System::getTime();

  if(!down)
    movingDownStartTime = 0;
  else if(!movingDownStartTime)
    movingDownStartTime = System::getTime();

  moving = movingLeftStartTime || movingRightStartTime || movingUpStartTime || movingDownStartTime;
}

void SimObjectRenderer::setCamera(const float* pos, const float* target)
{
  cameraPos = Vector3<>(pos[0], pos[1], pos[2]);
  cameraTarget = Vector3<>(target[0], target[1], target[2]);
  updateCameraTransformation();
}

void SimObjectRenderer::getCamera(float* pos, float* target)
{
  pos[0] = cameraPos.x; pos[1] = cameraPos.y; pos[2] = cameraPos.z;
  target[0] = cameraTarget.x; target[1] = cameraTarget.y; target[2] = cameraTarget.z;
}

