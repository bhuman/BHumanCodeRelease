/**
* @file Simulation/Sensors/DepthImageSensor.cpp
* Implementation of class DepthImageSensor
* @author Colin Graf
*/

#include "Platform/OpenGL.h"
#include <algorithm>

#include "Simulation/Sensors/DepthImageSensor.h"
#include "Simulation/Body.h"
#include "Simulation/Scene.h"
#include "Platform/Assert.h"
#include "Platform/OffscreenRenderer.h"
#include "Tools/OpenGLTools.h"
#include "CoreModule.h"

DepthImageSensor::DepthImageSensor()
{
  sensor.depthImageSensor = this;
  sensor.sensorType = SimRobotCore2::SensorPort::floatArraySensor;
  sensor.unit = "m";
  sensor.imageBuffer = 0;
  sensor.renderBuffer = 0;
  sensor.lut = 0;
}

DepthImageSensor::~DepthImageSensor()
{
  if(sensor.imageBuffer)
    delete[] sensor.imageBuffer;
  if(projection == sphericalProjection && sensor.renderBuffer)
    delete[] sensor.renderBuffer;
  if(sensor.lut)
    delete[] sensor.lut;
}

void DepthImageSensor::createPhysics()
{
  sensor.imageBuffer = new float[imageWidth * imageHeight];
  sensor.renderHeight = imageHeight;

  if(projection == sphericalProjection)
  {
    ASSERT(imageHeight == 1);

    sensor.numOfBuffers = (unsigned int) ceil(angleX / (float(M_PI) * 2.0f / 3.0f));
    sensor.bufferWidth = (unsigned int) ceil(float(imageWidth) / float(sensor.numOfBuffers));
    sensor.lut = new float*[sensor.bufferWidth];
    sensor.renderAngleX = angleX * sensor.bufferWidth / imageWidth;

    //Compute new resolution of rendering buffer
    float maxAngle(sensor.renderAngleX / 2.0f);
    float minPixelWidth(tanf(maxAngle/(float(sensor.bufferWidth) / 2.0f)));
    float totalWidth(tanf(maxAngle));
    float newXRes(totalWidth / minPixelWidth);
    sensor.renderWidth = (unsigned int) ceil(newXRes) * 2;
    sensor.renderBuffer = new float[sensor.renderWidth];

    //Compute values for LUT (sensor data -> rendering buffer)
    float firstAngle(-maxAngle);
    float step(maxAngle / ((float) sensor.bufferWidth / 2.0f));
    float currentAngle(firstAngle);
    float gToPixelFactor(newXRes / tanf(maxAngle));
    for(unsigned int i = 0; i < sensor.bufferWidth; ++i)
    {
      float g(tanf(currentAngle));
      g *= gToPixelFactor;
      int gPixel((int) g + (int) sensor.renderWidth / 2);
      sensor.lut[i] = &sensor.renderBuffer[gPixel];
      currentAngle += step;
    }
  }
  else
  {
    sensor.numOfBuffers = 1;
    sensor.bufferWidth = imageWidth;
    sensor.renderWidth = imageWidth;
    sensor.renderAngleX = angleX;
    sensor.renderBuffer = sensor.imageBuffer;
  }
  OpenGLTools::convertTransformation(rotation, translation, transformation);

  sensor.dimensions.append(imageWidth);
  if(imageHeight > 1)
    sensor.dimensions.append(imageHeight);
  sensor.data.floatArray = sensor.imageBuffer;

  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;

  sensor.min = min;
  sensor.max = max;

  const float zNear = std::max(min, 0.001f); // at least 1mm since zNear must not be zero
  float aspect = tanf(sensor.renderAngleX * 0.5f) / tanf(angleY * 0.5f);
  OpenGLTools::computePerspective(angleY, aspect, zNear, max, sensor.projection);
}

void DepthImageSensor::addParent(Element& element)
{
  sensor.physicalObject = dynamic_cast< ::PhysicalObject*>(&element);
  ASSERT(sensor.physicalObject);
  Sensor::addParent(element);
}

void DepthImageSensor::registerObjects()
{
  sensor.fullName = fullName + ".image";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void DepthImageSensor::DistanceSensor::updateValue()
{
  // make sure the poses of all movable objects are up to date
  Simulation::simulation->scene->updateTransformations();

  OffscreenRenderer& renderer = Simulation::simulation->renderer;

  renderer.makeCurrent(renderWidth, renderHeight, false);
  glViewport(0, 0, renderWidth, renderHeight);

  // setup image size and angle of view
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projection);
  glMatrixMode(GL_MODELVIEW);

  // disable lighting and textures, and use flat shading
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glPolygonMode(GL_FRONT, GL_FILL);
  glShadeModel(GL_FLAT);

  // setup camera position
  Pose3<> pose = physicalObject->pose;
  pose.conc(offset);
  static const Matrix3x3<> cameraRotation(Vector3<>(0.f, -1.f, 0.f), Vector3<>(0.f, 0.f, 1.f), Vector3<>(-1.f, 0.f, 0.f));
  pose.rotate(cameraRotation);
  pose.rotate(Matrix3x3<>(Vector3<>(0, (depthImageSensor->angleX - renderAngleX) / 2.0f, 0)));

  float* val = imageBuffer;
  unsigned int widthLeft = depthImageSensor->imageWidth;
  for(unsigned int i = 0; i < numOfBuffers; ++i)
  {
    float transformation[16];
    OpenGLTools::convertTransformation(pose.invert(), transformation);
    glLoadMatrixf(transformation);

    // disable color rendering
    glColorMask(0, 0, 0, 0);

    // draw all objects
    glClear(GL_DEPTH_BUFFER_BIT);
    Simulation::simulation->scene->drawAppearances();

    // enable color rendering again
    glColorMask(1, 1, 1, 1);

    // read frame buffer
    renderer.finishDepthRendering(renderBuffer, renderWidth, renderHeight);

    if(depthImageSensor->projection == perspectiveProjection)
    {
      // convert pixels to points in world and compute the depth (renderBuffer == imageBuffer)
      const float halfP34 = projection[14] * 0.5f;
      const float halfP33m1 = projection[10] * 0.5f - 0.5f;
      for(float* end = val + renderWidth * renderHeight; val < end; ++val)
        *val = halfP34 / (*val + halfP33m1);
    }
    else
    {
      // convert pixels to points in world and compute the distances (renderBuffer != imageBuffer)
      const float fInvSqr = 1.f / (projection[0] * projection[0]);
      const float halfP34 = projection[14] * 0.5f;
      const float halfP33m1 = projection[10] * 0.5f - 0.5f;
      float* const mid = lut[bufferWidth / 2];
      const float factor = 2.0f / float(renderWidth);
      const unsigned int end = std::min(bufferWidth, widthLeft);
      for(unsigned int i = 0; i < end; ++i)
      {
        const float vx = (lut[i] - mid) * factor;
        *val++ = std::min<float>(halfP34 / (*lut[i] + halfP33m1) * sqrtf(1.f + vx * vx * fInvSqr), max);
      }
      widthLeft -= end;
      pose.rotate(Matrix3x3<>(Vector3<>(0, -renderAngleX, 0)));
    }
  }
}

bool DepthImageSensor::DistanceSensor::getMinAndMax(float& min, float& max) const
{
  min = this->min;
  max = this->max;
  return true;
}

void DepthImageSensor::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showSensors)
  {
    Vector3<> ml;
    if(projection == perspectiveProjection)
      ml = Vector3<> (max, -tanf(angleX * 0.5f) * max, 0);
    else
      ml = Vector3<>(cosf(angleX * 0.5f) * max, -sinf(angleX * 0.5f) * max, 0);
    Vector3<> mt(ml.x, 0, tanf(angleY * 0.5f) * max);
    Vector3<> tl(ml.x, ml.y, mt.z);
    Vector3<> tr(ml.x, -ml.y, mt.z);
    Vector3<> bl(ml.x, ml.y, -mt.z);
    Vector3<> br(ml.x, -ml.y, -mt.z);

    glBegin(GL_LINE_LOOP);
      glColor3f(0, 0, 0.5f);
      glNormal3f (0, 0, 1.f);

      unsigned segments = int(18 * angleX / M_PI);
      if(projection == perspectiveProjection && segments > 0)
      {
        glVertex3f(tl.x, tl.y, tl.z);
        glVertex3f(tr.x, tr.y, tr.z);
        glVertex3f(br.x, br.y, br.z);
      }
      else
      {
        float rotX = cosf(angleX / float(segments));
        float rotY = sinf(angleX / float(segments));
        float x = tl.x;
        float y = tl.y;
        for(unsigned int i = 0; i < segments; ++i)
        {
          glVertex3f(x, y, tl.z);
          float x2 = x * rotX - y * rotY;
          y = y * rotX + x * rotY;
          x = x2;
        }
        glVertex3f(tr.x, tr.y, tr.z);
        for(unsigned int i = 0; i < segments; ++i)
        {
          glVertex3f(x, y, br.z);
          float x2 = x * rotX + y * rotY;
          y = y * rotX - x * rotY;
          x = x2;
        }
      }

      glVertex3f(bl.x, bl.y, bl.z);
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(tl.x, tl.y, tl.z);
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(tr.x, tr.y, tr.z);
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(bl.x, bl.y, bl.z);
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(br.x, br.y, br.z);
    glEnd();
  }

  Sensor::drawPhysics(flags);

  glPopMatrix();
}
