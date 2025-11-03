/**
 * @file PaintMethods3DOpenGL.h
 *
 * This file declares a class that draws 3D debug drawings using OpenGL.
 *
 * @author Arne Hasselbring
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Eigen.h"
#include <qopengl.h>
#include <SimRobotCore3.h>
#include <unordered_map>
#include <vector>

struct CameraImage;
class DebugDrawing3D;
class QOpenGLContext;
class QOpenGLFunctions_3_3_Core;

class PaintMethods3DOpenGL : public SimRobotCore3::Controller3DDrawingManager
{
public:
  /** Constructor. */
  PaintMethods3DOpenGL();

  /** Register a new OpenGL context. */
  void registerContext() override;

  /** Unregister an OpenGL context. */
  void unregisterContext() override;

  /** (Re)set members for a new frame in the current context. */
  void beforeFrame() override;

  /** Upload data to the VBOs and EBO. */
  void uploadData() override;

  /** Reset draw indices. */
  void beforeDraw() override;

  /** Deletes some resources that were used in the frame. */
  void afterFrame() override;

  /**
   * Creates resources for a drawing.
   * @param drawing The drawing.
   * @param projection The projection matrix.
   * @param view The view matrix.
   * @param model The model matrix.
   * @param flip Whether the drawing must be flipped.
   */
  void beforeFrame(const DebugDrawing3D& drawing, const Matrix4f& projection, const Matrix4f& view, const Matrix4f& model, bool flip);

  /**
   * Issues draw calls for a drawing.
   * @param drawing The drawing.
   */
  void draw(const DebugDrawing3D& drawing);

private:
  /**
   * Returns a newly allocated copy of the camera image in the RGB format.
   * @param srcImage A YUYV image.
   * @return Pointer to the beginning of an RGB image with the same size as the input. Must be freed by the caller.
   */
  static char* copyImage(const CameraImage& srcImage);

  /** Helper for temporarily changing the alpha value for drawing. */
  struct Alpha
  {
    float blendColor[4]; /**< The original blend color currently set. */
    QOpenGLFunctions_3_3_Core& f; /**< The OpenGL functions. */

    /**
     * The constructor integrates the alpha value into the current blend color.
     * @param f The OpenGL functions.
     * @param alpha The alpha value in the range of 0 ... 255.
     */
    Alpha(QOpenGLFunctions_3_3_Core& f, unsigned char alpha);

    /** The destructor resets the blend color to its previous value. */
    ~Alpha();
  };

  struct VertexP
  {
    VertexP(const Vector3f& position) :
      position(position)
    {}

    Vector3f position;
  };

  struct VertexPC
  {
    VertexPC(const Vector3f& position, const ColorRGBA& color) :
      position(position),
      color(color.r, color.g, color.b, 0)
    {}

    Vector3f position;
    Eigen::Vector4<unsigned char> color; // 1 byte padding
  };

  struct PerContextData
  {
    QOpenGLFunctions_3_3_Core* f = nullptr; /**< OpenGL functions for this share group. */

    GLuint ldqVBO = 0; /**< Vertex buffer object for lines, dots and quads. */
    GLuint ldqVAO = 0; /**< Vertex attribute object for lines, dots and quads. */
    GLuint ldqProgram = 0; /**< Shader for lines, dots and quads. */
    GLint ldqPVMLocation = -1; /**< Location of the pvm uniform in the shader for lines, dots and quads. */

    GLuint secVBO = 0; /**< Vertex buffer object for spheres, ellipsoids and cylinders. */
    GLuint secEBO = 0; /**< Element buffer object for spheres, ellipsoids and cylinders. */
    GLuint secVAO = 0; /**< Vertex attribute object for spheres, ellipsoids and cylinders. */
    GLuint secProgram = 0; /**< Shader for spheres, ellipsoids and cylinders. */
    GLint secPVMLocation = -1; /**< Location of the pvm uniform in the shader for spheres, ellipsoids and cylinders. */
    GLint secColorLocation = -1; /**< Location of the color uniform in the shader for spheres, ellipsoids and cylinders. */

    GLuint imageVAO = 0; /**< Vertex attribute object for images. */
    GLuint imageProgram = 0; /**< Shader for images. */
    GLint imagePVMLocation = -1; /**< Location of the pvm uniform in the shader for images. */

    std::size_t referenceCounter = 1; /**< Reference counter for this context. */
    std::size_t referenceCounterIndex; /**< Index in \c referenceCounters for the counter of this context's share group. */
  };

  static constexpr unsigned int sphereSlices = 16; /**< Number of slices of the unit sphere. */
  static constexpr unsigned int sphereStacks = 16; /**< Number of stacks of the unit sphere. */
  static constexpr unsigned int cylinderSlices = 16; /**< Number of slices for cylinders. */

  // Context handling:
  std::vector<unsigned> referenceCounters; /**< References counters per share group. */
  std::unordered_map<const QOpenGLContext*, PerContextData> perContextData; /**< Data for each OpenGL context. */

  // Constant data:
  std::vector<VertexP> sphereVertexData; /**< Vertex positions for a unit sphere. */
  std::vector<std::uint16_t> sphereIndexData; /**< Indices for a unit sphere. */

  // Valid for the current draw pass (between \c beginFrame and \c endFrame):
  PerContextData* data = nullptr; /**< Data for the OpenGL context in this pass. */
  QOpenGLFunctions_3_3_Core* f = nullptr; /**< OpenGL functions for the context in this pass. */
  std::unordered_map<const DebugDrawing3D*, Matrix4f> transformations; /**< Map from debug drawing to transformation matrix. */
  std::vector<VertexPC> ldqVertexData; /**< Vertex data for lines, dots and quads. */
  std::vector<VertexP> secVertexData; /**< Vertex data for spheres, ellipsoids and cylinders. */
  std::vector<std::uint16_t> secIndexData; /**< Index data for spheres, ellipsoids and cylinders. */
  GLint ldqFirst = 0; /**< First vertex for the next draw call of lines/dots/quads. */
  GLint secBaseVertex = 0; /**< Base vertex for the next draw call of cylinders (the unit sphere is always at 0). */
  GLsizeiptr secIndexOffset = 0; /**< Index offset for the next draw call of cylinders (the units sphere is always at 0). */
  bool includeSphere = false; /**< Whether the sphere must be part of the vertex buffer. */
  std::vector<GLuint> textures; /**< The textures (in order) for 3D images. */
  unsigned int textureIndex = 0; /**< The texture index for the next image draw call. */
};
