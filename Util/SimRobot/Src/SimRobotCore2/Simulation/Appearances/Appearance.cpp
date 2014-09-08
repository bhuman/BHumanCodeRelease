/**
* @file Simulation/Appearances/Appearance.cpp
* Implementation of class Appearance
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Appearances/Appearance.h"
#include "Simulation/Scene.h"
#include "Platform/Assert.h"
#include "Tools/OpenGLTools.h"
#include "Tools/Texture.h"
#include "CoreModule.h"

Appearance::Surface::Surface() : texture(0)
{
  hasAmbientColor = false;

  diffuseColor[0] = 0.8f;
  diffuseColor[1] = 0.8f;
  diffuseColor[2] = 0.8f;
  diffuseColor[3] = 1.0f;

  ambientColor[0] = 0.2f;
  ambientColor[1] = 0.2f;
  ambientColor[2] = 0.2f;
  ambientColor[3] = 1.0f;

  specularColor[0] = 0.0f;
  specularColor[1] = 0.0f;
  specularColor[2] = 0.0f;
  specularColor[3] = 1.0f;

  shininess = 0.0f;

  emissionColor[0] = 0.0f;
  emissionColor[1] = 0.0f;
  emissionColor[2] = 0.0f;
  emissionColor[3] = 1.0f;
}

void Appearance::Surface::set(bool defaultTextureSize) const
{
  if(hasAmbientColor)
  {
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambientColor);
  }
  else
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor4fv(diffuseColor);

  glMaterialfv(GL_FRONT, GL_SPECULAR, specularColor);
  glMaterialf(GL_FRONT, GL_SHININESS, shininess);
  glMaterialfv(GL_FRONT, GL_EMISSION, emissionColor);

  if(texture)
  {
    glBindTexture(GL_TEXTURE_2D, texture->textureId);

    if(texture->hasAlpha)
    {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    if(defaultTextureSize)
    {
      glEnable(GL_TEXTURE_GEN_S);
      glEnable(GL_TEXTURE_GEN_T);
      static const float texGen_s_coeff[4] = {1.f, 0.f, 0.f, 0.f};
      static const float texGen_t_coeff[4]= {0.f, 1.f, 0.f, 0.f};
      glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
      glTexGenfv(GL_S, GL_OBJECT_PLANE, texGen_s_coeff);
      glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
      glTexGenfv(GL_T, GL_OBJECT_PLANE, texGen_t_coeff);
    }
  }
  else if(diffuseColor[3] < 1.0f)
  {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
}

void Appearance::Surface::unset(bool defaultTextureSize) const
{
  if(texture)
  {
    if(defaultTextureSize)
    {
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
  }
  if(diffuseColor[3] < 1.0f || (texture && texture->hasAlpha))
    glDisable(GL_BLEND);
}

void Appearance::createGraphics()
{
  if(initializedContexts == 0)
  {
    OpenGLTools::convertTransformation(rotation, translation, transformation);
    if(surface && !surface->diffuseTexture.empty())
      surface->texture = Simulation::simulation->scene->loadTexture(surface->diffuseTexture);
  }

  GraphicalObject::createGraphics();
}

const QIcon* Appearance::getIcon() const
{
  return &CoreModule::module->appearanceIcon;
}

void Appearance::addParent(Element& element)
{
  SimObject::addParent(element);
  GraphicalObject::addParent(element);
}

void Appearance::Surface::addParent(Element& element)
{
  Appearance* appearance = dynamic_cast<Appearance*>(&element);
  ASSERT(!appearance->surface);
  appearance->surface = this;
}

void Appearance::assembleAppearances() const
{
  glPushMatrix();
  glMultMatrixf(transformation);
  GraphicalObject::assembleAppearances();
  glPopMatrix();
}
