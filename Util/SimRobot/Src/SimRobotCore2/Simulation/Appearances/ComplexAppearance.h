/**
* @file Simulation/Appearances/ComplexAppearance.h
* Declaration of class ComplexAppearance
* @author Colin Graf
*/

#pragma once

#include <vector>

#include "Simulation/Appearances/Appearance.h"

/**
* @class ComplexAppearance
* The graphical representation of a complex shape
*/
class ComplexAppearance : public Appearance
{
public:
  /**
  * @class Vertex
  * A class encapsulating the three components of a vertex
  */
  class Vertex
  {
  public:
    float x; /**< The x-component of the vertex */
    float y; /**< The y-component of the vertex */
    float z; /**< The z-component of the vertex */

    /**
    * Default constructor; Sets all components to zero
    */
    Vertex() : x(0), y(0), z(0) {}

    /**
    * Constructs a vertex
    * @param x The x-component of the vertex
    * @param y The y-component of the vertex
    * @param z The z-component of the vertex
    */
    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
  };

  /**
  * The normal vector of a vertex
  */
  class Normal : public Vertex
  {
  public:
    unsigned int length; /**< The length of the normal (used to normalize accumulated normals) */

    /**
    * @class Normal
    * Default constructor; Sets all components to zero
    */
    Normal() : length(0) {}

    /**
    * Constructs a vertex
    * @param x The x-component of the vertex
    * @param y The y-component of the vertex
    * @param z The z-component of the vertex
    * @param length The length of the normal
    */
    Normal(float x, float y, float z, unsigned int length) : Vertex(x, y, z), length(length) {}

    /** Addition of another normal to this one.
    * @param other The other normal that will be added to this one
    */
    void operator+=(const Normal& other)
    {
      x += other.x;
      y += other.y;
      z += other.z;
      length += other.length;
    }
  };

  /**
  * @class TexCoord
  * A point on a texture
  */
  class TexCoord
  {
  public:
    float x; /**< The x-component of the point */
    float y; /**< The y-component of the point */

    /**
    * Constructs a point of a texture
    * @param x The x-component of the point
    * @param y The y-component of the point
    */
    TexCoord(float x, float y) : x(x), y(y) {}
  };

  /**
  * A vertex library
  */
  class Vertices : public Element
  {
  public:
    float unit;
    std::vector<Vertex> vertices; /**< Available vertices */

  private:
    /**
    * Registers an element as parent
    * @param element The element to register
    */
    void addParent(Element& element) override;
  };

  /**
  * A normals library
  */
  class Normals : public Element
  {
  public:
    std::vector<Normal> normals; /**< Available normals */

  private:
    /**
    * Registers an element as parent
    * @param element The element to register
    */
    void addParent(Element& element) override;
  };

  /**
  * @class TexCoords
  * A texture point library
  */
  class TexCoords : public Element
  {
  public:
    std::vector<TexCoord> coords; /**< Available points */

  private:
    /**
    * Registers an element as parent
    * @param element The element to register
    */
    void addParent(Element& element) override;
  };

  /**
  * @class PrimitiveGroup
  * A primitive (aka. face, like triangle or quad...) or a group of primitives
  */
  class PrimitiveGroup : public Element
  {
  public:
    int mode; /**< The OpenGL primitive group type (\c GL_TRIANGLES, \c GL_QUADS, ...) */
    std::list<unsigned int> vertices; /**< The indices of the vertices used to draw the primitive */

    /**
    * Constructor
    * @param mode The OpenGL primitive group type (\c GL_TRIANGLES, \c GL_QUADS, ...)
    */
    PrimitiveGroup(int mode) : mode(mode) {}

  private:
    /**
    * Registers an element as parent
    * @param element The element to register
    */
    void addParent(Element& element) override;
  };

  Vertices* vertices; /**< The vertex library used for drawing the primitives */
  Normals* normals; /**< The normals library used for drawing the primitives */
  TexCoords* texCoords; /**< Optional texture points for textured primitives */
  std::list<PrimitiveGroup*> primitiveGroups; /** The primitives that define the complex shape */
  bool normalsDefined; /**< Normals were manually defined */

  /** Default constructor */
  ComplexAppearance() : vertices(0), normals(0), texCoords(0), normalsDefined(false) {}

private:
  /**
  * Prepares the object and the currently selected OpenGL context for drawing the object.
  * Loads textures and creates display lists. Hence, this function is called for each OpenGL
  * context the object should be drawn in.
  */
  void createGraphics() override;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;
};
