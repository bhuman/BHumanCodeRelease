/**
* @file Simulation/Appearances/Appearance.h
* Declaration of class Appearance
* @author Colin Graf
*/

#pragma once

#include "Simulation/SimObject.h"
#include "Simulation/GraphicalObject.h"

class Texture;

/**
* @class Appearance
* Abstract class for the graphical representation of physical objects
*/
class Appearance : public SimObject, public GraphicalObject, public SimRobotCore2::Appearance
{
public:
  class Surface : public Element
  {
  public:
    float diffuseColor[4];
    bool hasAmbientColor;
    float ambientColor[4];
    float specularColor[4];
    float emissionColor[4];
    float shininess;
    std::string diffuseTexture;
    Texture* texture;

    /** Constructor */
    Surface();

    /**
    * Selects this material surface in the currently selected OpenGL context.
    * @param defaultTextureSize Initialize a default texture size if the surface has a texture.
    */
    void set(bool defaultTextureSize = true) const;

    /**
    * Unbinds a texture that might be bound on a \c set call before. Call function this every time
    * after using the material in order to allow combining textured and not textured surfaces within
    * a scene without enabling an disabling GL_TEXTURE_2D
    * @param defaultTextureSize Clean up the default texture size if the surface has a texture.
    */
    void unset(bool defaultTextureSize = true) const;

  private:
    /**
    * Registers an element as parent
    * @param element The element to register
    */
    virtual void addParent(Element& element);
  };

  Surface* surface; /**< The visual material of the object */

  /** Default constructor */
  Appearance() : surface(0) {}

protected:
  /**
  * Prepares the object and the currently selected OpenGL context for drawing the object.
  * Loads textures and creates display lists. Hence, this function is called for each OpenGL
  * context the object should be drawn in.
  */
  virtual void createGraphics();

private:
  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  virtual void assembleAppearances() const;

  //API
  virtual const QString& getFullName() const {return SimObject::getFullName();}
  virtual SimRobot::Widget* createWidget() {return SimObject::createWidget();}
  virtual const QIcon* getIcon() const;
  virtual SimRobotCore2::Renderer* createRenderer() {return SimObject::createRenderer();}
};
