/**
* @file Simulation/Sensors/ObjectSegmentedImageSensor.cpp
* Implementation of class ObjectSegmentedImageSensor
*
* @author Jesse Richter-Klug
*  includes a lot of duplicated code origin in Simulation/Sensors/Camera.cpp authored by Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Sensors/ObjectSegmentedImageSensor.h"
#include "Simulation/Body.h"
#include "Simulation/Scene.h"
#include "Platform/Assert.h"
#include "Platform/OffscreenRenderer.h"
#include "Tools/OpenGLTools.h"
#include "CoreModule.h"

ObjectSegmentedImageSensor::ObjectSegmentedImageSensor()
{
  sensor.camera = this;
  sensor.sensorType = SimRobotCore2::SensorPort::cameraSensor;
  sensor.imageBuffer = 0;
  sensor.imageBufferSize = 0;
}

ObjectSegmentedImageSensor::~ObjectSegmentedImageSensor()
{
  if(sensor.imageBuffer)
    delete[] sensor.imageBuffer;
}

void ObjectSegmentedImageSensor::createPhysics()
{
  OpenGLTools::convertTransformation(rotation, translation, transformation);

  sensor.dimensions.append(imageWidth);
  sensor.dimensions.append(imageHeight);
  sensor.dimensions.append(3);

  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;

  float aspect = std::tan(angleX * 0.5f) / std::tan(angleY * 0.5f);
  OpenGLTools::computePerspective(angleY, aspect, 0.01f, 500.f, sensor.projection);
}

void ObjectSegmentedImageSensor::addParent(Element& element)
{
  sensor.physicalObject = dynamic_cast< ::PhysicalObject*>(&element);
  ASSERT(sensor.physicalObject);
  Sensor::addParent(element);
}

void ObjectSegmentedImageSensor::registerObjects()
{
  sensor.fullName = fullName + ".image";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void ObjectSegmentedImageSensor::ObjectSegmentedImageSensorPort::updateValue()
{
  // allocate buffer
  const unsigned int imageWidth = camera->imageWidth;
  const unsigned int imageHeight = camera->imageHeight;
  const unsigned int imageSize = imageWidth * imageHeight * 3;
  if(imageBufferSize < imageSize)
  {
    if(imageBuffer)
      delete[] imageBuffer;
    imageBuffer = new unsigned char[imageSize];
    imageBufferSize = imageSize;
  }

  // make sure the poses of all movable objects are up to date
  Simulation::simulation->scene->updateTransformations();

  // prepare offscreen renderer
  OffscreenRenderer& renderer = Simulation::simulation->renderer;
  renderer.makeCurrent(imageWidth, imageHeight);

  // setup image size and angle of view
  glViewport(0, 0, imageWidth, imageHeight);
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projection);
  glMatrixMode(GL_MODELVIEW);

  // disable lighting, textures, and do flat shading
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glPolygonMode(GL_FRONT, GL_FILL);
  glShadeModel(GL_FLAT);

  // setup camera position
  Pose3f pose = physicalObject->pose;
  pose.conc(offset);
  static const RotationMatrix cameraRotation = (Matrix3f() << Vector3f(0.f, -1.f, 0.f), Vector3f(0.f, 0.f, 1.f), Vector3f(-1.f, 0.f, 0.f)).finished();
  pose.rotate(cameraRotation);
  float transformation[16];
  OpenGLTools::convertTransformation(pose.invert(), transformation);
  glLoadMatrixf(transformation);

  // draw all objects
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // draw all objects
  Simulation::simulation->scene->GraphicalObject::drawAppearances(SurfaceColor(0), false);
  int j = 0;
  for(std::list<Body*>::const_iterator iter = Simulation::simulation->scene->bodies.begin(),
      end = Simulation::simulation->scene->bodies.end(); iter != end; ++iter, ++j)
    (*iter)->drawAppearances(SurfaceColor((j % (SurfaceColor::numOfSurfaceColors - 1)) + 1), false);

  // read frame buffer
  renderer.finishImageRendering(imageBuffer, imageWidth, imageHeight);
  data.byteArray = imageBuffer;
}

bool ObjectSegmentedImageSensor::ObjectSegmentedImageSensorPort::renderCameraImages(SimRobotCore2::SensorPort** cameras, unsigned int count)
{
  if(lastSimulationStep == Simulation::simulation->simulationStep)
    return true;

  // allocate buffer
  const unsigned int imageWidth = camera->imageWidth;
  const unsigned int imageHeight = camera->imageHeight;
  const unsigned int imageSize = imageWidth * imageHeight * 3;
  int imagesOfCurrentSize = 0;
  for(unsigned int i = 0; i < count; ++i)
  {
    ObjectSegmentedImageSensorPort* sensor = static_cast<ObjectSegmentedImageSensorPort*>(cameras[i]);
    if(sensor && sensor->lastSimulationStep != Simulation::simulation->simulationStep &&
       sensor->camera->imageWidth == imageWidth && sensor->camera->imageHeight == imageHeight)
      ++imagesOfCurrentSize;
  }
  const unsigned int multiImageBufferSize = imageSize * imagesOfCurrentSize;

  if(imageBufferSize < multiImageBufferSize)
  {
    if(imageBuffer)
      delete[] imageBuffer;
    imageBuffer = new unsigned char[multiImageBufferSize];
    imageBufferSize = multiImageBufferSize;
  }

  // make sure the poses of all movable objects are up to date
  Simulation::simulation->scene->updateTransformations();

  // prepare offscreen renderer
  OffscreenRenderer& renderer = Simulation::simulation->renderer;
  renderer.makeCurrent(imageWidth, imageHeight * count);

  // setup angle of view
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projection);
  glMatrixMode(GL_MODELVIEW);

  // disable lighting, textures, and do flat shading
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glPolygonMode(GL_FRONT, GL_FILL);
  glShadeModel(GL_FLAT);

  // clear buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // render images
  int currentHorizontalPos = 0;
  unsigned char* currentBufferPos = imageBuffer;
  for(unsigned int i = 0; i < count; ++i)
  {
    ObjectSegmentedImageSensorPort* sensor = static_cast<ObjectSegmentedImageSensorPort*>(cameras[i]);
    if(sensor && sensor->lastSimulationStep != Simulation::simulation->simulationStep &&
       sensor->camera->imageWidth == imageWidth && sensor->camera->imageHeight == imageHeight)
    {
      glViewport(0, currentHorizontalPos, imageWidth, imageHeight);

      // setup camera position
      Pose3f pose = sensor->physicalObject->pose;
      pose.conc(sensor->offset);
      static const RotationMatrix cameraRotation = (Matrix3f() << Vector3f(0.f, -1.f, 0.f), Vector3f(0.f, 0.f, 1.f), Vector3f(-1.f, 0.f, 0.f)).finished();
      pose.rotate(cameraRotation);
      float transformation[16];
      OpenGLTools::convertTransformation(pose.invert(), transformation);
      glLoadMatrixf(transformation);

      // draw all objects
      Simulation::simulation->scene->GraphicalObject::drawAppearances(SurfaceColor(0), false);
      int j = 0;
      for(std::list<Body*>::const_iterator iter = Simulation::simulation->scene->bodies.begin(),
          end = Simulation::simulation->scene->bodies.end(); iter != end; ++iter, ++j)
        (*iter)->drawAppearances(SurfaceColor((j % (SurfaceColor::numOfSurfaceColors - 1)) + 1), false);

      sensor->data.byteArray = currentBufferPos;
      sensor->lastSimulationStep = Simulation::simulation->simulationStep;

      currentHorizontalPos += imageHeight;
      currentBufferPos += imageSize;
    }
  }

  // read frame buffer
  renderer.finishImageRendering(imageBuffer, imageWidth, currentHorizontalPos);
  return true;
}

void ObjectSegmentedImageSensor::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showSensors)
  {
    const Vector3f ml(1.f, -std::tan(angleX * 0.5f), 0);
    const Vector3f mt(1.f, 0, std::tan(angleY * 0.5f));
    const Vector3f tl(1.f, ml.y(), mt.z());
    const Vector3f tr(1.f, -ml.y(), mt.z());
    const Vector3f bl(1.f, ml.y(), -mt.z());
    const Vector3f br(1.f, -ml.y(), -mt.z());

    glBegin(GL_LINE_LOOP);
      glColor3f(0, 0, 0.5f);
      glNormal3f (0, 0, 1.f);
      glVertex3f(tl.x(), tl.y(), tl.z());
      glVertex3f(tr.x(), tr.y(), tr.z());
      glVertex3f(br.x(), br.y(), br.z());
      glVertex3f(bl.x(), bl.y(), bl.z());
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(tl.x(), tl.y(), tl.z());
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(tr.x(), tr.y(), tr.z());
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(bl.x(), bl.y(), bl.z());
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(br.x(), br.y(), br.z());
    glEnd();
  }

  Sensor::drawPhysics(flags);

  glPopMatrix();
}
