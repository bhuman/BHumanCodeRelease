/**
* @file Simulation/Appearances/Appearance.h
* Declaration of class Appearance
* @author Colin Graf
*/

#pragma once

#include "Simulation/SimObject.h"
#include "Simulation/GraphicalObject.h"

class Texture;

enum SurfaceColor : unsigned char
{
  ownColor,
  red,
  blue,
  green,
  yellow,
  brown,
  pink,
  purple,
  navy,
  blueviolet,
  deepskyblue,
  olive,
  lime,
  lightseagreen,
  chocolate,
  orange,
  darkorange,
  numOfSurfaceColors //<-- this has to be last
};

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
    Texture* texture = nullptr;

    /** Constructor */
    Surface();

    /**
    * Selects this material surface in the currently selected OpenGL context.
    * @param defaultTextureSize Initialize a default texture size if the surface has a texture.
    */
    void set(SurfaceColor color = ownColor, bool defaultTextureSize = true) const;

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
    void addParent(Element& element) override;
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
  void createGraphics() override;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (as fast as possible) */
  void drawAppearances(SurfaceColor color, bool drawControllerDrawings) const override;

private:
  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;

  //API
  const QString& getFullName() const override {return SimObject::getFullName();}
  SimRobot::Widget* createWidget() override {return SimObject::createWidget();}
  const QIcon* getIcon() const override;
  SimRobotCore2::Renderer* createRenderer() override {return SimObject::createRenderer();}
  bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing) override {return ::GraphicalObject::registerDrawing(drawing);}
  bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing) override {return ::GraphicalObject::unregisterDrawing(drawing);}
};
