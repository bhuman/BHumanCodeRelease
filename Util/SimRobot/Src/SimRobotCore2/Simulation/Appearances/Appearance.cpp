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

static float colorDef[numOfSurfaceColors][4] =
{
  { 1,1,1,1 }, // ownColor,
  { 1,0,0,1 }, // red,
  { 0,0,1,1 }, // blue,
  { 0,0.5f,0,1 }, // green,
  { 1,1,0,1 }, // yellow,
  { .5f,.12f,.12f,1 }, // brown,
  { 1,.37f,.73f,1 }, // pink,
  { .5f,0,.5f,1 },  // purple,
  { 0 ,0 ,0.5f ,1.f}, // navy,
  { 138.f / 255.f,43.f / 255.f,226.f / 255.f,1.f}, // blueviolet
  {  0 , 191/ 255.f , 1 ,1.f}, // deepskyblue,
  { 0.5f , 0.5f , 0,1.f}, // olive,
  { 0 ,1 ,0 ,1.f}, // lime,
  { 32.f/ 255.f ,178.f / 255.f , 170.f/ 255.f ,1.f}, // lightseagreen,
  { 210.f / 255.f , 105.f/ 255.f , 30.f/ 255.f ,1.f}, // chocolate,
  { 1, 165 / 255.f,0 ,1.f}, // orange,
  { 1, 140.f / 255.f, 0 ,1.f} // darkorange,
};

void Appearance::Surface::set(SurfaceColor color, bool defaultTextureSize) const
{
  if(hasAmbientColor)
  {
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glMaterialfv(GL_FRONT, GL_AMBIENT, color != ownColor ? colorDef[color] : ambientColor);
  }
  else
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor4fv(color != ownColor ? colorDef[color] : diffuseColor);

  glMaterialfv(GL_FRONT, GL_SPECULAR, color != ownColor ? colorDef[color] : specularColor);
  glMaterialf(GL_FRONT, GL_SHININESS, shininess);
  glMaterialfv(GL_FRONT, GL_EMISSION, color != ownColor ? colorDef[color] : emissionColor);

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

void Appearance::assembleAppearances(SurfaceColor color) const
{
  glPushMatrix();
  glMultMatrixf(transformation);
  GraphicalObject::assembleAppearances(color);
  glPopMatrix();
}

void Appearance::drawAppearances(SurfaceColor color, bool drawControllerDrawings) const
{
  if(drawControllerDrawings)
  {
    glPushMatrix();
    glMultMatrixf(transformation);
    GraphicalObject::drawAppearances(color, drawControllerDrawings);
    glPopMatrix();
  }
  else
    GraphicalObject::drawAppearances(color, drawControllerDrawings);
}
