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
#include "Tools/Math/Constants.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math.h"
#include "Tools/OpenGLTools.h"

SimObjectRenderer::SimObjectRenderer(SimObject& simObject) :
  simObject(simObject), width(0), height(0),
  cameraMode(targetCam), defaultCameraPos(3.f, 6.f, 4.f), cameraPos(defaultCameraPos), cameraTarget(Vector3f::Zero()), fovy(40.f),
  surfaceShadeMode(smoothShading), physicsShadeMode(noShading), drawingsShadeMode(smoothShading), renderFlags(enableLights | enableTextures | enableMultisample),
  dragging(false), dragPlane(xyPlane), dragMode(keepDynamics), degreeSteps(15)
{
}

void SimObjectRenderer::resetCamera()
{
  cameraPos = defaultCameraPos;
  cameraTarget = Vector3f::Zero();
  updateCameraTransformation();
}

void SimObjectRenderer::updateCameraTransformation()
{
  static const Vector3f cameraUpVector(0.f, 0.f, 1.f);
  OpenGLTools::computeCameraTransformation(cameraPos, cameraTarget, cameraUpVector, cameraTransformation);
}

void SimObjectRenderer::init(bool hasSharedDisplayLists)
{
  Simulation::simulation->scene->createGraphics(hasSharedDisplayLists);
  calcDragPlaneVector();
}

void SimObjectRenderer::draw()
{
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
    Pose3f pose((Matrix3f() << transformation[0], transformation[4], transformation[8],
                               transformation[1], transformation[5], transformation[9],
                               transformation[2], transformation[6], transformation[10]).finished(),
                Vector3f(transformation[12], transformation[13], transformation[14]));
    float invTrans[16];
    OpenGLTools::convertTransformation(pose.invert(), invTrans);
    glMultMatrixf(invTrans);
  }

  // draw origin
  if(renderFlags & showCoordinateSystem)
  {
    Simulation::simulation->scene->defaultSurface->set();
    glBegin(GL_LINES);
    glNormal3f(0, 0, 1);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
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
    graphicalObject->drawAppearances(SurfaceColor(0), false);

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
      glTranslatef(dragSelection->pose.translation.x(), dragSelection->pose.translation.y(), dragSelection->pose.translation.z());

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
    glNormal3f(0, 0, 1);
    gluDisk(q, 0.003f, 0.5f, 30, 1);
    glRotatef(180.f, 1.f, 0.f, 0.f);
    gluDisk(q, 0.003f, 0.5f, 30, 1);
    gluDeleteQuadric(q);
    glPopMatrix();

    glDisable(GL_BLEND);

    Simulation::simulation->scene->defaultSurface->unset();
  }

  // draw controller drawings
  if(drawingsShadeMode != noShading)
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

    glEnable(GL_BLEND);
    glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
    glBlendColor(1.0f, 1.0f, 1.0f, 1.0f);

    if(renderFlags & enableDrawingsTransparentOcclusion)
    {
      if(physicalObject)
        physicalObject->drawPhysics(showControllerDrawings);
      if(graphicalObject)
        graphicalObject->drawAppearances(SurfaceColor(0), true);
    }

    if((renderFlags & enableDrawingsTransparentOcclusion)
       || !(renderFlags & enableDrawingsOcclusion))
      glClear(GL_DEPTH_BUFFER_BIT);

    if(renderFlags & enableDrawingsTransparentOcclusion)
      glBlendColor(0.5f, 0.5f, 0.5f, 0.5f);

    if(physicalObject)
      physicalObject->drawPhysics(showControllerDrawings);
    if(graphicalObject)
      graphicalObject->drawAppearances(SurfaceColor(0), true);

    glDisable(GL_BLEND);

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
  viewport[0] = viewport[1] = 0;
  viewport[2] = width;
  viewport[3] = height; // store viewport for gluUnProject

  glMatrixMode(GL_PROJECTION);
  OpenGLTools::computePerspective(fovy * (pi / 180.f), float(width) / float(height), 0.1f, 500.f, projection);
  glLoadMatrixf(projection);

  glMatrixMode(GL_MODELVIEW);
}

Vector3f SimObjectRenderer::projectClick(int x, int y) const
{
  GLdouble mvmatrix[16];
  GLdouble projmatrix[16];
  for(int i = 0; i < 16; ++i)
  {
    mvmatrix[i] = static_cast<GLdouble>(cameraTransformation[i]);
    projmatrix[i] = static_cast<GLdouble>(projection[i]);
  }

  GLdouble tx, ty, tz;
  gluUnProject(static_cast<GLdouble>(x), static_cast<GLdouble>(height - y), 1.0, mvmatrix, projmatrix, viewport, &tx, &ty, &tz);
  return Vector3f(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
}

void SimObjectRenderer::getSize(unsigned int& width, unsigned int& height) const
{
  width = this->width;
  height = this->height;
}

void SimObjectRenderer::zoom(float change, float x, float y)
{
  Vector3f v = cameraTarget - cameraPos;
  if(x < 0 || y < 0)
  {
    v.normalize(v.norm() * change * 0.0005f);
    cameraPos -= v;
  }
  else
  {
    Vector3f translationVector;
    if(intersectClickAndCoordinatePlane(static_cast<int>(x), static_cast<int>(y), dragPlane, translationVector))
    {
      translationVector = translationVector - cameraPos;
      cameraPos += translationVector * change * 0.0005f;
      intersectRayAndPlane(cameraPos, v, cameraTarget, dragPlaneVector, cameraTarget);
    }
  }
  updateCameraTransformation();
}

void SimObjectRenderer::fitCamera()
{
  /*
  if(simObject)
    simulation->fitCamera(cameraTarget, cameraPos, width, height, simObject);
  calculateUpVector();
   */
}

void SimObjectRenderer::setDragPlane(DragAndDropPlane plane)
{
  dragPlane = plane;
  calcDragPlaneVector();
}

bool SimObjectRenderer::intersectRayAndPlane(const Vector3f& point, const Vector3f& v,
                                             const Vector3f& plane, const Vector3f& n,
                                             Vector3f& intersection) const
{
  Vector3f p = plane - point;
  float denominator = n.dot(v);
  if(denominator == 0.f)
    return false;
  float r = n.dot(p) / denominator;
  if(r < 0.f)
    return false;
  intersection = v;
  intersection *= r;
  intersection += point;
  return true;
}

bool SimObjectRenderer::intersectClickAndCoordinatePlane(int x, int y, DragAndDropPlane plane, Vector3f& point) const
{
  return intersectRayAndPlane(cameraPos, projectClick(x, y) - cameraPos, cameraTarget, dragPlaneVector, point);
}

void SimObjectRenderer::calcDragPlaneVector()
{
  switch(dragPlane)
  {
    case xyPlane:
      dragPlaneVector = Vector3f(0.f, 0.f, 1.f);
      break;
    case xzPlane:
      dragPlaneVector = Vector3f(0.f, 1.f, 0.f);
      break;
    case yzPlane:
      dragPlaneVector = Vector3f(1.f, 0.f, 0.f);
      break;
  }
}

Body* SimObjectRenderer::selectObject(const Vector3f& projectedClick)
{
  if(&simObject != Simulation::simulation->scene)
    return nullptr;

  class Callback
  {
  public:
    Body* closestBody;
    float closestSqrDistance;
    const Vector3f& cameraPos;

    Callback(const Vector3f& cameraPos) : closestBody(0), cameraPos(cameraPos) {}

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
      float sqrDistance = (Vector3f(static_cast<float>(pos[0]), static_cast<float>(pos[1]), static_cast<float>(pos[2])) - callback->cameraPos).squaredNorm();
      if(!callback->closestBody || sqrDistance < callback->closestSqrDistance)
      {
        callback->closestBody = static_cast<Body*>(dBodyGetData(bodyId));
        callback->closestSqrDistance = sqrDistance;
      }
    }

    static void staticCollisionWithSpaceCallback(Callback* callback, dGeomID geom1, dGeomID geom2)
    {
      ASSERT(!dGeomIsSpace(geom1));
      ASSERT(dGeomIsSpace(geom2));
      dSpaceCollide2(geom1, geom2, callback, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
    }
  };

  Callback callback(cameraPos);
  dGeomID ray = dCreateRay(Simulation::simulation->staticSpace, 10000.f);
  Vector3f dir = projectedClick - cameraPos;
  dGeomRaySet(ray, cameraPos.x(), cameraPos.y(), cameraPos.z(), dir.x(), dir.y(), dir.z());
  dSpaceCollide2(ray, reinterpret_cast<dGeomID>(Simulation::simulation->movableSpace), &callback, reinterpret_cast<dNearCallback*>(&Callback::staticCollisionWithSpaceCallback));
  dGeomDestroy(ray);

  if(!callback.closestBody)
    return nullptr;
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
    Vector3f projectedClick = projectClick(x, y);
    dragSelection = selectObject(projectedClick);

    if(dragSelection)
    {
      calcDragPlaneVector();
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
    dragStartPos.x() = x;
    dragStartPos.y() = y;
    interCameraPos = cameraPos;
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

bool SimObjectRenderer::moveDrag(int x, int y, DragType type)
{
  if(!dragging)
    return false;

  dragType = type;

  if(!dragSelection) // camera control
  {
    if(dragType == dragRotate || dragType == dragRotateWorld)
    {
      Vector3f v = (dragType == dragRotate ? cameraPos : interCameraPos) - cameraTarget;
      const RotationMatrix rotateY = RotationMatrix::aroundY((y - dragStartPos.y()) * -0.01f);
      const RotationMatrix rotateZ = RotationMatrix::aroundZ((x - dragStartPos.x()) * -0.01f);
      const float hypoLength = std::sqrt(v.x() * v.x() + v.y() * v.y());
      Vector3f v2(hypoLength, 0.f, v.z());
      v2 = rotateY * v2;
      if(v2.x() < 0.001f)
      {
        v2.x() = 0.001f;
        v2.normalize(v.norm());
      }
      Vector3f v3(v.x(), v.y(), 0.f);
      v3.normalize(v2.x());
      v3.z() = v2.z();
      v = rotateZ * v3;
      interCameraPos = cameraTarget + v;
      if(dragType == dragRotate)
        cameraPos = cameraTarget + v;
      else
      {
        float angleZ = std::atan2(v.y(), v.x()) * (180.f / pi);
        float angleY = ((pi / 2.f) - std::atan2(v.z(), hypoLength)) * (180.f / pi);
        int zRounded = ((static_cast<int>(angleZ) + degreeSteps / 2) / degreeSteps) * degreeSteps;
        int yRounded = ((static_cast<int>(angleY) + degreeSteps / 2) / degreeSteps) * degreeSteps;
        angleZ = zRounded * (pi / 180.f);
        angleY = yRounded * (pi / 180.f);
        if(angleY == 0)
          angleY = 0.00001f;
        cameraPos = cameraTarget + Vector3f(std::sin(angleY) * std::cos(angleZ), std::sin(angleY) * std::sin(angleZ), std::cos(angleY)).normalize(v.norm());
      }
    }
    else // if(dragType == dragNormal)
    {
      Vector3f start;
      Vector3f end;
      if(intersectClickAndCoordinatePlane(static_cast<int>(dragStartPos.x()), static_cast<int>(dragStartPos.y()), dragPlane, start))
        if(intersectClickAndCoordinatePlane(x, y, dragPlane, end))
        {
          Vector3f translate = end - start;
          cameraPos -= translate;
          cameraTarget -= translate;
        }
    }

    dragStartPos.x() = x;
    dragStartPos.y() = y;
    updateCameraTransformation();
    return true;
  }
  else // object control
  {
    if(dragMode == applyDynamics)
      return true;
    Vector3f projectedClick = projectClick(x, y);
    Vector3f currentPos;
    if(intersectRayAndPlane(cameraPos, projectedClick - cameraPos, dragSelection->pose.translation, dragPlaneVector, currentPos))
    {
      if(dragType == dragRotate || dragType == dragRotateWorld)
      {
        Vector3f oldv = dragStartPos - dragSelection->pose.translation;
        Vector3f newv = currentPos - dragSelection->pose.translation;

        if(dragType != dragRotateWorld)
        {
          const RotationMatrix invRotation = dragSelection->pose.rotation.inverse();
          oldv = invRotation * oldv;
          newv = invRotation * newv;
        }

        float angle = 0.f;
        if(dragPlane == yzPlane)
          angle = normalize(std::atan2(newv.z(), newv.y()) - std::atan2(oldv.z(), oldv.y()));
        else if(dragPlane == xzPlane)
          angle = normalize(std::atan2(newv.x(), newv.z()) - std::atan2(oldv.x(), oldv.z()));
        else
          angle = normalize(std::atan2(newv.y(), newv.x()) - std::atan2(oldv.y(), oldv.x()));

        const Vector3f offset = dragPlaneVector * angle;
        const RotationMatrix rotation = Rotation::AngleAxis::unpack(offset);
        Vector3f center = dragSelection->pose.translation;
        dragSelection->rotate(rotation, center);
        if(dragMode == adoptDynamics)
        {
          const unsigned int now = System::getTime();
          const float t = std::max(1U, now - dragStartTime) * 0.001f;
          Vector3f velocity = offset / t;
          const dReal* oldVel = dBodyGetAngularVel(dragSelection->body);
          velocity = velocity * 0.3f + Vector3f(static_cast<float>(oldVel[0]), static_cast<float>(oldVel[1]), static_cast<float>(oldVel[2])) * 0.7f;
          dBodySetAngularVel(dragSelection->body, velocity.x(), velocity.y(), velocity.z());
          dragStartTime = now;
        }
        dragStartPos = currentPos;
      }
      else
      {
        const Vector3f offset = currentPos - dragStartPos;
        dragSelection->move(offset);
        if(dragMode == adoptDynamics)
        {
          const unsigned int now = System::getTime();
          const float t = std::max(1U, now - dragStartTime) * 0.001f;
          Vector3f velocity = offset / t;
          const dReal* oldVel = dBodyGetLinearVel(dragSelection->body);
          velocity = velocity * 0.3f + Vector3f(static_cast<float>(oldVel[0]), static_cast<float>(oldVel[1]), static_cast<float>(oldVel[2])) * 0.7f;
          dBodySetLinearVel(dragSelection->body, velocity.x(), velocity.y(), velocity.z());
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
      moveDrag(x, y, dragType);
    else if(dragMode == applyDynamics)
    {
      Vector3f projectedClick = projectClick(x, y);
      Vector3f currentPos;
      if(intersectRayAndPlane(cameraPos, projectedClick - cameraPos, dragSelection->pose.translation, dragPlaneVector, currentPos))
      {
        if(dragType == dragRotate || dragType == dragRotateWorld)
        {
          Vector3f oldv = dragStartPos - dragSelection->pose.translation;
          Vector3f newv = currentPos - dragSelection->pose.translation;

          if(dragType != dragRotateWorld)
          {
            const RotationMatrix invRotation = dragSelection->pose.rotation.inverse();
            oldv = invRotation * oldv;
            newv = invRotation * newv;
          }

          float angle = 0.f;
          if(dragPlane == yzPlane)
            angle = normalize(std::atan2(newv.z(), newv.y()) - std::atan2(oldv.z(), oldv.y()));
          else if(dragPlane == xzPlane)
            angle = normalize(std::atan2(newv.x(), newv.z()) - std::atan2(oldv.x(), oldv.z()));
          else
            angle = normalize(std::atan2(newv.y(), newv.x()) - std::atan2(oldv.y(), oldv.x()));

          const Vector3f offset = dragPlaneVector * angle;
          const Vector3f torque = offset * static_cast<float>(dragSelection->mass.mass) * 50.f;
          dBodyAddTorque(dragSelection->body, torque.x(), torque.y(), torque.z());
        }
        else
        {
          const Vector3f offset = currentPos - dragStartPos;
          const Vector3f force = offset * static_cast<float>(dragSelection->mass.mass) * 500.f;
          dBodyAddForce(dragSelection->body, force.x(), force.y(), force.z());
        }
      }
    }

    dragSelection->enablePhysics(true);

    dragging = false;
    return true;
  }
}

void SimObjectRenderer::setCamera(const float* pos, const float* target)
{
  cameraPos = Vector3f(pos[0], pos[1], pos[2]);
  cameraTarget = Vector3f(target[0], target[1], target[2]);
  updateCameraTransformation();
}

void SimObjectRenderer::getCamera(float* pos, float* target)
{
  pos[0] = cameraPos.x();
  pos[1] = cameraPos.y();
  pos[2] = cameraPos.z();
  target[0] = cameraTarget.x();
  target[1] = cameraTarget.y();
  target[2] = cameraTarget.z();
}
