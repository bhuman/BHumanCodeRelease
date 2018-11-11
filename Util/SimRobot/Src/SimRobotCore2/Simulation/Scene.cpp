/**
* @file Simulation/Scene.h
* Implementation of class Scene
* @author Colin Graf
*/

#include "CoreModule.h"
#include "Platform/OpenGL.h"
#include "Platform/Assert.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Body.h"
#include "Simulation/Actuators/Actuator.h"
#include "Tools/Math/Constants.h"

void Scene::updateTransformations()
{
  if(lastTransformationUpdateStep != Simulation::simulation->simulationStep)
  {
    for(std::list<Body*>::const_iterator iter = bodies.begin(), end = bodies.end(); iter != end; ++iter)
      (*iter)->updateTransformation();
    lastTransformationUpdateStep = Simulation::simulation->simulationStep;
  }
}

void Scene::updateActuators()
{
  for(std::list<Actuator::Port*>::const_iterator iter = actuators.begin(), end = actuators.end(); iter != end; ++iter)
    (*iter)->act();
}

void Scene::createGraphics(bool isShared)
{
  float* clearColor = Simulation::simulation->scene->color;
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  // enable depth test
  glClearDepth(1.0f);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);

  // avoid rendering the backside of surfaces
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);

  //
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  // setup lights
  glEnable(GL_COLOR_MATERIAL);
  GLfloat data[4];
  data[0] = 0.2f; data[1] = 0.2f; data[2] = 0.2f; data[3] = 1.f;
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, data);
  //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

  int i = 0;
  for(std::list<Light*>::const_iterator iter = lights.begin(), end = lights.end(); iter != end; ++iter, ++i)
  {
    const Light& light = *(*iter);
    glLightfv(GL_LIGHT0 + i, GL_AMBIENT, light.ambientColor);
    glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, light.diffuseColor);
    glLightfv(GL_LIGHT0 + i, GL_SPECULAR, light.specularColor);
    glLightfv(GL_LIGHT0 + i, GL_POSITION, light.position);
    glLightfv(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, &light.constantAttenuation);
    glLightfv(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, &light.linearAttenuation);
    glLightfv(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, &light.quadraticAttenuation);
    float spotCutoff = light.spotCutoff * (180.f / pi);
    glLightfv(GL_LIGHT0 + i, GL_SPOT_CUTOFF, &spotCutoff);
    glLightfv(GL_LIGHT0 + i, GL_SPOT_DIRECTION, light.spotDirection.data());
    glLightfv(GL_LIGHT0 + i, GL_SPOT_EXPONENT, &light.spotExponent);
    glEnable(GL_LIGHT0 + i);
  }

  // load display lists and textures
  if(!isShared)
  {
    for(std::list<Body*>::const_iterator iter = bodies.begin(), end = bodies.end(); iter != end; ++iter)
      (*iter)->createGraphics();
    GraphicalObject::createGraphics();

    if(!textures.empty())
    {
      for(std::unordered_map<std::string, Texture>::iterator iter = textures.begin(), end = textures.end(); iter != end; ++iter)
        iter->second.createGraphics();
      glBindTexture(GL_TEXTURE_2D, 0);
    }
  }
}

void Scene::drawAppearances(SurfaceColor color, bool drawControllerDrawings) const
{
  for(std::list<Body*>::const_iterator iter = bodies.begin(), end = bodies.end(); iter != end; ++iter)
    (*iter)->drawAppearances(color, drawControllerDrawings);
  GraphicalObject::drawAppearances(color, drawControllerDrawings);
}

void Scene::drawPhysics(unsigned int flags) const
{
  for(std::list<Body*>::const_iterator iter = bodies.begin(), end = bodies.end(); iter != end; ++iter)
    (*iter)->drawPhysics(flags);
  ::PhysicalObject::drawPhysics(flags);
}

const QIcon* Scene::getIcon() const
{
  return &CoreModule::module->sceneIcon;
}

unsigned int Scene::getStep() const
{
  return Simulation::simulation->simulationStep;
}

double Scene::getTime() const
{
  return Simulation::simulation->simulatedTime;
}

unsigned int Scene::getFrameRate() const
{
  return Simulation::simulation->currentFrameRate;
}

Texture* Scene::loadTexture(const std::string& file)
{
  std::unordered_map<std::string, Texture>::iterator iter = textures.find(file);
  if(iter != textures.end())
  {
    Texture& texture = iter->second;
    return texture.imageData ? &texture : 0;
  }
  Texture& texture = textures[file];
  texture.load(file);
  return texture.imageData ? &texture : 0;
}

Scene::Light::Light() : constantAttenuation(1.f), linearAttenuation(0.f), quadraticAttenuation(0.f), spotCutoff(pi), spotDirection(0.f, 0.f, -1.f), spotExponent(0.f)
{
  diffuseColor[0] = diffuseColor[1] = diffuseColor[2] = diffuseColor[3] = 1.f;
  ambientColor[0] = ambientColor[1] = ambientColor[2] = 0.f;
  ambientColor[3] = 1.f;
  specularColor[0] = specularColor[1] = specularColor[2] = specularColor[3] = 1.f;
  position[0] = position[1] = position[3] = 0.f;
  position[2] = 1.f;
}

void Scene::Light::addParent(Element& element)
{
  Scene* scene = dynamic_cast<Scene*>(&element);
  ASSERT(scene);
  scene->lights.push_back(this);
}
