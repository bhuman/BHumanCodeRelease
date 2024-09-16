/**
 * @file PaintMethods3DOpenGL.cpp
 *
 * This file implements a class that draws 3D debug drawings using OpenGL.
 *
 * @author Arne Hasselbring
 * @author Thomas Röfer
 */

#include "PaintMethods3DOpenGL.h"
#include "SimulatedNao/Visualization/DebugDrawing3D.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/CameraImage.h"
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>

const char* ldqVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inColor;
uniform mat4 pvm;

out vec3 Color;

void main()
{
  Color = inColor;
  gl_Position = pvm * vec4(inPos, 1.0);
}
)glsl";

const char* secVertexShaderSource = R"glsl(
#version 330 core

layout(location = 0) in vec3 inPos;
uniform mat4 pvm;

void main()
{
  gl_Position = pvm * vec4(inPos, 1.0);
}

)glsl";

const char* imageVertexShaderSource = R"glsl(
#version 330 core

const vec3 vertices[] = vec3[]
(
  vec3(0.0, -1.0, 1.0), vec3(0.0, 1.0, 1.0),
  vec3(0.0, -1.0, -1.0), vec3(0.0, 1.0, -1.0)
);

const vec2 texCoords[] = vec2[]
(
  vec2(1, 1), vec2(0, 1),
  vec2(1, 0), vec2(0, 0)
);

uniform mat4 pvm;

out vec2 TexCoords;

void main()
{
  TexCoords = texCoords[gl_VertexID];
  gl_Position = pvm * vec4(vertices[gl_VertexID], 1.0);
}
)glsl";

const char* ldqFragmentShaderSource = R"glsl(
#version 330 core
in vec3 Color;
out vec4 outFragColor;
void main()
{
  outFragColor = vec4(Color, 1.0);
}
)glsl";

const char* secFragmentShaderSource = R"glsl(
#version 330 core
uniform uvec3 color;
out vec4 outFragColor;
void main()
{
  outFragColor = vec4(vec3(color) / 255.0, 1.0);
}
)glsl";

const char* imageFragmentShaderSource = R"glsl(
#version 330 core
uniform sampler2D tex;
in vec2 TexCoords;
out vec4 outFragColor;
void main()
{
  outFragColor = vec4(texture(tex, TexCoords).rgb, 1.0);
}
)glsl";

PaintMethods3DOpenGL::Alpha::Alpha(QOpenGLFunctions_3_3_Core& f, unsigned char alpha) :
  f(f)
{
  if(alpha < 255)
  {
    f.glGetFloatv(GL_BLEND_COLOR, blendColor);
    f.glBlendColor(blendColor[0], blendColor[1], blendColor[2], blendColor[3] * alpha / 255.f);
  }
  else
    blendColor[0] = -1.f;
}

PaintMethods3DOpenGL::Alpha::~Alpha()
{
  if(blendColor[0] != -1.f)
    f.glBlendColor(blendColor[0], blendColor[1], blendColor[2], blendColor[3]);
}

PaintMethods3DOpenGL::PaintMethods3DOpenGL()
{
  sphereVertexData.reserve(1 + (sphereStacks - 1) * sphereSlices + 1);
  sphereVertexData.emplace_back(Vector3f(0.f, 0.f, -1.f));
  for(unsigned int i = 1; i < sphereStacks; ++i)
  {
    const float z = -std::cos(static_cast<float>(i) * pi / sphereStacks);
    const float r = std::sin(static_cast<float>(i) * pi / sphereStacks);
    for(unsigned int j = 0; j < sphereSlices; ++j)
    {
      const float x = r * std::cos(static_cast<float>(j) * 2.f * pi / sphereSlices);
      const float y = r * std::sin(static_cast<float>(j) * 2.f * pi / sphereSlices);
      sphereVertexData.emplace_back(Vector3f(x, y, z));
    }
  }
  sphereVertexData.emplace_back(Vector3f(0.f, 0.f, 1.f));
  sphereIndexData.reserve(3 * (sphereSlices + 2 * (sphereStacks - 2) * sphereSlices + sphereSlices));
  for(unsigned short i = 0; i < sphereSlices; ++i)
  {
    sphereIndexData.push_back(0);
    sphereIndexData.push_back(((i + 1) % sphereSlices) + 1);
    sphereIndexData.push_back(i + 1);
  }
  for(unsigned short i = 0; i < sphereStacks - 2; ++i)
  {
    for(unsigned short j = 0; j < sphereSlices; ++j)
    {
      sphereIndexData.push_back(j + i * sphereSlices + 1);
      sphereIndexData.push_back(((j + 1) % sphereSlices) + i * sphereSlices + 1);
      sphereIndexData.push_back(((j + 1) % sphereSlices) + (i + 1) * sphereSlices + 1);
      sphereIndexData.push_back(((j + 1) % sphereSlices) + (i + 1) * sphereSlices + 1);
      sphereIndexData.push_back(j + (i + 1) * sphereSlices + 1);
      sphereIndexData.push_back(j + i * sphereSlices + 1);
    }
  }
  for(unsigned short i = 0; i < sphereSlices; ++i)
  {
    sphereIndexData.push_back((sphereStacks - 1) * sphereSlices + 1);
    sphereIndexData.push_back(i + (sphereStacks - 2) * sphereSlices + 1);
    sphereIndexData.push_back(((i + 1) % sphereSlices) + (sphereStacks - 2) * sphereSlices + 1);
  }
}

void PaintMethods3DOpenGL::registerContext()
{
  const auto* context = QOpenGLContext::currentContext();

  if(perContextData.find(context) != perContextData.end())
  {
    ++perContextData[context].referenceCounter;
    return;
  }

  const auto shareDataIt = std::find_if(perContextData.begin(), perContextData.end(), [context](const auto& data)
  {
    return QOpenGLContext::areSharing(const_cast<QOpenGLContext*>(context), const_cast<QOpenGLContext*>(data.first));
  });
  const PerContextData* shareData = shareDataIt != perContextData.end() ? &shareDataIt->second : nullptr;
  PerContextData& data = perContextData[context];
  data.referenceCounterIndex = shareData ? shareData->referenceCounterIndex : [this]
  {
    for(std::size_t i = 0; i < referenceCounters.size(); ++i)
      if(!referenceCounters[i])
        return i;
    referenceCounters.push_back(0);
    return referenceCounters.size() - 1;
  }();
  ++referenceCounters[data.referenceCounterIndex];

  ASSERT(!f);
  if(shareData)
    data.f = shareData->f;
  else
  {
    data.f = new QOpenGLFunctions_3_3_Core;
    data.f->initializeOpenGLFunctions();
  }
  f = data.f;

  if(shareData)
    data.ldqVBO = shareData->ldqVBO;
  else
    f->glGenBuffers(1, &data.ldqVBO);
  f->glBindBuffer(GL_ARRAY_BUFFER, data.ldqVBO);

  f->glGenVertexArrays(1, &data.ldqVAO);
  f->glBindVertexArray(data.ldqVAO);
  f->glEnableVertexAttribArray(0);
  f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPC), nullptr);
  f->glEnableVertexAttribArray(1);
  f->glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(VertexPC), reinterpret_cast<void*>(3 * sizeof(GLfloat)));

  if(shareData)
    data.secVBO = shareData->secVBO;
  else
    f->glGenBuffers(1, &data.secVBO);
  f->glBindBuffer(GL_ARRAY_BUFFER, data.secVBO);

  if(shareData)
    data.secEBO = shareData->secEBO;
  else
    f->glGenBuffers(1, &data.secEBO);

  f->glGenVertexArrays(1, &data.secVAO);
  f->glBindVertexArray(data.secVAO);
  f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data.secEBO);
  f->glEnableVertexAttribArray(0);
  f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexP), nullptr);

  f->glGenVertexArrays(1, &data.imageVAO);

  if(shareData)
  {
    data.ldqProgram = shareData->ldqProgram;
    data.ldqPVMLocation = shareData->ldqPVMLocation;

    data.secProgram = shareData->secProgram;
    data.secPVMLocation = shareData->secPVMLocation;
    data.secColorLocation = shareData->secColorLocation;

    data.imageProgram = shareData->imageProgram;
    data.imagePVMLocation = shareData->imagePVMLocation;
  }
  else
  {
    auto compileShader = [this](const char* vertexShaderSource, const char* fragmentShaderSource)
    {
#ifndef NDEBUG
      GLint status;
#endif
      const GLuint program = f->glCreateProgram();
      auto vertexShader = f->glCreateShader(GL_VERTEX_SHADER);
      f->glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
      f->glCompileShader(vertexShader);
#ifndef NDEBUG
      f->glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &status);
      if(!status)
      {
        char message[512];
        f->glGetShaderInfoLog(vertexShader, sizeof(message), nullptr, message);
        FAIL(message);
      }
#endif
      f->glAttachShader(program, vertexShader);
      f->glDeleteShader(vertexShader);
      auto fragmentShader = f->glCreateShader(GL_FRAGMENT_SHADER);
      f->glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
      f->glCompileShader(fragmentShader);
#ifndef NDEBUG
      f->glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &status);
      if(!status)
      {
        char message[512];
        f->glGetShaderInfoLog(fragmentShader, sizeof(message), nullptr, message);
        FAIL(message);
      }
#endif
      f->glAttachShader(program, fragmentShader);
      f->glDeleteShader(fragmentShader);
      f->glLinkProgram(program);
#ifndef NDEBUG
      f->glGetProgramiv(program, GL_LINK_STATUS, &status);
      if(!status)
      {
        char message[512];
        f->glGetProgramInfoLog(program, sizeof(message), nullptr, message);
        FAIL(message);
      }
#endif
      return program;
    };

    data.ldqProgram = compileShader(ldqVertexShaderSource, ldqFragmentShaderSource);
    data.ldqPVMLocation = f->glGetUniformLocation(data.ldqProgram, "pvm");

    data.secProgram = compileShader(secVertexShaderSource, secFragmentShaderSource);
    data.secPVMLocation = f->glGetUniformLocation(data.secProgram, "pvm");
    data.secColorLocation = f->glGetUniformLocation(data.secProgram, "color");

    data.imageProgram = compileShader(imageVertexShaderSource, imageFragmentShaderSource);
    data.imagePVMLocation = f->glGetUniformLocation(data.imageProgram, "pvm");
  }

  f = nullptr;
}

void PaintMethods3DOpenGL::unregisterContext()
{
  const auto* context = QOpenGLContext::currentContext();
  auto& data = perContextData[context];
  if(--data.referenceCounter)
    return;

  data.f->glDeleteVertexArrays(1, &data.imageVAO);
  data.f->glDeleteVertexArrays(1, &data.secVAO);
  data.f->glDeleteVertexArrays(1, &data.ldqVAO);
  if(--referenceCounters[data.referenceCounterIndex] == 0)
  {
    data.f->glDeleteProgram(data.imageProgram);
    data.f->glDeleteBuffers(1, &data.secEBO);
    data.f->glDeleteBuffers(1, &data.secVBO);
    data.f->glDeleteProgram(data.secProgram);
    data.f->glDeleteBuffers(1, &data.ldqVBO);
    data.f->glDeleteProgram(data.ldqProgram);
    delete data.f;
  }

  perContextData.erase(context);
}

void PaintMethods3DOpenGL::beforeFrame()
{
  ASSERT(!data);
  ASSERT(!f);
  data = &perContextData[QOpenGLContext::currentContext()];
  f = data->f;

  includeSphere = false;
}

void PaintMethods3DOpenGL::uploadData()
{
  ASSERT(data);
  ASSERT(f);

  if(!ldqVertexData.empty())
  {
    f->glBindBuffer(GL_ARRAY_BUFFER, data->ldqVBO);
    f->glBufferData(GL_ARRAY_BUFFER, ldqVertexData.size() * sizeof(VertexPC), ldqVertexData.data(), GL_STREAM_DRAW);
    ldqVertexData.clear();
  }

  if(includeSphere)
  {
    secVertexData.insert(secVertexData.begin(), sphereVertexData.begin(), sphereVertexData.end());
    secIndexData.insert(secIndexData.begin(), sphereIndexData.begin(), sphereIndexData.end());
  }

  if(!secVertexData.empty())
  {
    f->glBindBuffer(GL_ARRAY_BUFFER, data->secVBO);
    f->glBufferData(GL_ARRAY_BUFFER, secVertexData.size() * sizeof(VertexP), secVertexData.data(), GL_STREAM_DRAW);
    secVertexData.clear();
  }

  if(!secIndexData.empty())
  {
    f->glBindVertexArray(data->secVAO);
    f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, secIndexData.size() * sizeof(std::uint16_t), secIndexData.data(), GL_STREAM_DRAW);
    secIndexData.clear();
  }
}

void PaintMethods3DOpenGL::beforeDraw()
{
  ldqFirst = 0;
  secBaseVertex = includeSphere ? static_cast<GLint>(sphereVertexData.size()) : 0;
  secIndexOffset = includeSphere ? sizeof(std::uint16_t) * sphereIndexData.size() : 0;
  textureIndex = 0;
}

void PaintMethods3DOpenGL::afterFrame()
{
  ASSERT(data);
  ASSERT(f);

  if(!textures.empty())
    f->glDeleteTextures(static_cast<GLsizei>(textures.size()), textures.data());

  data = nullptr;
  f = nullptr;
  transformations.clear();
  textures.clear();
}

void PaintMethods3DOpenGL::beforeFrame(const DebugDrawing3D& drawing, const Matrix4f& projection, const Matrix4f& view, const Matrix4f& model, bool flip)
{
  auto& transformation = transformations[&drawing];
  transformation = projection * view * model;

  if(flip)
    transformation *= (Matrix4f() << -1.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f).finished();

  // Convert mm to m and custom scaling.
  transformation *= (Matrix4f() << 0.001f * drawing.scale.x(), 0.f, 0.f, 0.f, 0.f, 0.001f * drawing.scale.y(), 0.f, 0.f, 0.f, 0.f, 0.001f * drawing.scale.z(), 0.f, 0.f, 0.f, 0.f, 1.f).finished();

  // Custom rotation.
  Matrix3f rotation = Matrix3f::Identity();
  if(drawing.rotate.z() != 0)
    rotation *= RotationMatrix::aroundZ(drawing.rotate.z());
  if(drawing.rotate.y() != 0)
    rotation *= RotationMatrix::aroundY(drawing.rotate.y());
  if(drawing.rotate.x() != 0)
    rotation *= RotationMatrix::aroundX(drawing.rotate.x());

  // Custom translation (and rotation).
  transformation *= (Matrix4f() << rotation, drawing.trans, RowVector3f::Zero(), 1.f).finished();

  for(const DebugDrawing3D::Line& l : drawing.lines)
  {
    ldqVertexData.emplace_back(l.start, l.color);
    ldqVertexData.emplace_back(l.end, l.color);
  }

  for(const DebugDrawing3D::Dot& d : drawing.dots)
    ldqVertexData.emplace_back(d.point, d.color);

  for(const DebugDrawing3D::Quad& q : drawing.quads)
  {
    // This order is used to draw a quad as triangle strip.
    ldqVertexData.emplace_back(q.points[0], q.color);
    ldqVertexData.emplace_back(q.points[1], q.color);
    ldqVertexData.emplace_back(q.points[3], q.color);
    ldqVertexData.emplace_back(q.points[2], q.color);
  }

  includeSphere |= !drawing.spheres.empty() || !drawing.ellipsoids.empty();

  for(const DebugDrawing3D::Cylinder& c : drawing.cylinders)
  {
    if(c.baseRadius <= 0.f && c.topRadius <= 0.f)
      continue;
    secVertexData.emplace_back(Vector3f(0.f, 0.f, -c.height * 0.5f));
    if(c.baseRadius > 0.f)
    {
      for(unsigned int i = 0; i < cylinderSlices; ++i)
      {
        const float x = c.baseRadius * std::cos(static_cast<float>(i) * 2.f * pi / static_cast<float>(cylinderSlices));
        const float y = c.baseRadius * std::sin(static_cast<float>(i) * 2.f * pi / static_cast<float>(cylinderSlices));
        secVertexData.emplace_back(Vector3f(x, y, -c.height * 0.5f));
        secIndexData.push_back(0);
        secIndexData.push_back(((static_cast<unsigned short>(i) + 1) % cylinderSlices) + 1);
        secIndexData.push_back(static_cast<unsigned short>(i) + 1);
      }
    }
    secVertexData.emplace_back(Vector3f(0.f, 0.f, c.height * 0.5f));
    if(c.topRadius > 0.f)
    {
      const unsigned short baseIndex = 1 + (c.baseRadius > 0.f ? cylinderSlices : 0);
      for(unsigned int i = 0; i < cylinderSlices; ++i)
      {
        const float x = c.topRadius * std::cos(static_cast<float>(i) * 2.f * pi / static_cast<float>(cylinderSlices));
        const float y = c.topRadius * std::sin(static_cast<float>(i) * 2.f * pi / static_cast<float>(cylinderSlices));
        secVertexData.emplace_back(Vector3f(x, y, c.height * 0.5f));
        secIndexData.push_back(baseIndex);
        secIndexData.push_back(static_cast<unsigned short>(i) + baseIndex + 1);
        secIndexData.push_back(((static_cast<unsigned short>(i) + 1) % cylinderSlices) + baseIndex + 1);
      }
    }

    if(c.topRadius > 0.f && c.baseRadius > 0.f)
    {
      for(unsigned short i = 0; i < cylinderSlices; ++i)
      {
        secIndexData.push_back(i + 1);
        secIndexData.push_back(((i + 1) % cylinderSlices) + cylinderSlices + 2);
        secIndexData.push_back(i + cylinderSlices + 2);
        secIndexData.push_back(((i + 1) % cylinderSlices) + 1);
        secIndexData.push_back(((i + 1) % cylinderSlices) + cylinderSlices + 2);
        secIndexData.push_back(i + 1);
      }
    }
    else if(c.baseRadius <= 0.f)
    {
      ASSERT(c.topRadius > 0.f);
      for(unsigned short i = 0; i < cylinderSlices; ++i)
      {
        secIndexData.push_back(0);
        secIndexData.push_back(i + 1);
        secIndexData.push_back(((i + 1) % cylinderSlices) + 1);
      }
    }
    else if(c.topRadius <= 0.f)
    {
      for(unsigned short i = 0; i < cylinderSlices; ++i)
      {
        secIndexData.push_back(cylinderSlices + 1);
        secIndexData.push_back(i + 1);
        secIndexData.push_back(((i + 1) % cylinderSlices) + 1);
      }
    }
  }

  for(const DebugDrawing3D::Image3D& i : drawing.images)
  {
    GLuint texture;
    f->glGenTextures(1, &texture);
    f->glBindTexture(GL_TEXTURE_2D, texture);

    char* imageData = copyImage(*i.cameraImage);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, i.cameraImage->width, i.cameraImage->height,
                    0, GL_RGB, GL_UNSIGNED_BYTE, imageData);

    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    delete[] imageData;
    textures.push_back(texture);
  }
}

void PaintMethods3DOpenGL::draw(const DebugDrawing3D& drawing)
{
  ASSERT(data);
  ASSERT(f);

  bool draw = true;
  if(drawing.renderOptions & (Drawings3D::disableTransparency | Drawings3D::disableOpacity))
  {
    GLfloat blendColor[4];
    f->glGetFloatv(GL_BLEND_COLOR, blendColor);
    draw = (blendColor[0] == 1.f) == ((drawing.renderOptions & Drawings3D::disableTransparency) != 0);
  }

  if(draw && drawing.renderOptions & Drawings3D::disableDepth)
  {
    GLfloat blendColor[4];
    f->glGetFloatv(GL_BLEND_COLOR, blendColor);
    f->glDepthMask(GL_FALSE);
  }

  const auto& transformation = transformations[&drawing];

  if(!drawing.lines.empty() || !drawing.dots.empty() || !drawing.quads.empty())
  {
    // Bind VAO and shader and set transformation matrix.
    f->glBindVertexArray(data->ldqVAO);
    f->glUseProgram(data->ldqProgram);
    f->glUniformMatrix4fv(data->ldqPVMLocation, 1, GL_FALSE, transformation.data());

    // Draw all lines.
    if(!drawing.lines.empty() && draw)
    {
      for(const DebugDrawing3D::Line& l : drawing.lines)
      {
        // TODO: My OpenGL driver doesn't support wide lines with OpenGL 3.3.
        // f->glLineWidth(l.width);
        Alpha alpha(*f, l.color.a);
        f->glDrawArrays(GL_LINES, ldqFirst, 2);
        ldqFirst += 2;
      }
    }
    else
      ldqFirst += 2 * static_cast<GLint>(2 * drawing.lines.size());

    // Draw all dots.
    if(!drawing.dots.empty() && draw)
    {
      GLfloat oldPointSize = 1.f;
      f->glGetFloatv(GL_POINT_SIZE, &oldPointSize);
      for(const DebugDrawing3D::Dot& d : drawing.dots)
      {
        f->glPointSize(d.size);
        Alpha alpha(*f, d.color.a);
        f->glDrawArrays(GL_POINTS, ldqFirst, 1);
        ++ldqFirst;
      }
      f->glPointSize(oldPointSize);
    }
    else
      ldqFirst += 2 * static_cast<GLint>(drawing.dots.size());

    // Draw all quads.
    if(draw)
      for(const DebugDrawing3D::Quad& q : drawing.quads)
      {
        Alpha alpha(*f, q.color.a);
        f->glDrawArrays(GL_TRIANGLE_STRIP, ldqFirst, 4);
        ldqFirst += 4;
      }
    else
      ldqFirst += 4 * static_cast<GLint>(drawing.quads.size());
}

  if(!drawing.spheres.empty() || !drawing.ellipsoids.empty() || !drawing.cylinders.empty())
  {
    // Bind VAO and shader (transformation matrix is set per mesh).
    f->glBindVertexArray(data->secVAO);
    f->glUseProgram(data->secProgram);

    // Draw spheres.
    for(const DebugDrawing3D::Sphere& s : drawing.spheres)
    {
      const Matrix4f localTransformation = transformation *
                                           (Matrix4f() << Matrix3f::Identity(), s.point, RowVector3f::Zero(), 1.f).finished() *
                                           (Matrix4f() << Matrix3f::Identity() * s.radius, Vector3f::Zero(), RowVector3f::Zero(), 1.f).finished();
      f->glUniformMatrix4fv(data->secPVMLocation, 1, GL_FALSE, localTransformation.data());
      f->glUniform3ui(data->secColorLocation, s.color.r, s.color.g, s.color.b);
      Alpha alpha(*f, s.color.a);
      f->glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(sphereIndexData.size()), GL_UNSIGNED_SHORT, nullptr);
    }

    // Draw ellipsoids.
    for(const DebugDrawing3D::Ellipsoid& e : drawing.ellipsoids)
    {
      const Matrix4f localTransformation = transformation *
                                           (Matrix4f() << e.pose.rotation, e.pose.translation, RowVector3f::Zero(), 1.f).finished() *
                                           (Matrix4f() << e.radii.x(), 0.f, 0.f, 0.f, 0.f, e.radii.y(), 0.f, 0.f, 0.f, 0.f, e.radii.z(), 0.f, 0.f, 0.f, 0.f, 1.f).finished();
      f->glUniformMatrix4fv(data->secPVMLocation, 1, GL_FALSE, localTransformation.data());
      f->glUniform3ui(data->secColorLocation, e.color.r, e.color.g, e.color.b);
      Alpha alpha(*f, e.color.a);
      f->glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(sphereIndexData.size()), GL_UNSIGNED_SHORT, nullptr);
    }

    // Draw cylinders.
    for(const DebugDrawing3D::Cylinder& c : drawing.cylinders)
    {
      const unsigned int numOfIndices = 6 * cylinderSlices * ((c.baseRadius > 0.f) + (c.topRadius > 0.f));
      if(draw)
      {
        if(c.baseRadius <= 0.f && c.topRadius <= 0.f)
          continue;
        Matrix3f rotation = Matrix3f::Identity();
        if(c.rotation.x() != 0)
          rotation *= RotationMatrix::aroundX(c.rotation.x());
        if(c.rotation.y() != 0)
          rotation *= RotationMatrix::aroundY(c.rotation.y());
        if(c.rotation.z() != 0)
          rotation *= RotationMatrix::aroundZ(c.rotation.z());
        const Matrix4f localTransformation = transformation * (Matrix4f() << rotation, c.point, RowVector3f::Zero(), 1.f).finished();
        f->glUniformMatrix4fv(data->secPVMLocation, 1, GL_FALSE, localTransformation.data());
        f->glUniform3ui(data->secColorLocation, c.color.r, c.color.g, c.color.b);
        Alpha alpha(*f, c.color.a);
      f->glDrawElementsBaseVertex(GL_TRIANGLES, numOfIndices, GL_UNSIGNED_SHORT, reinterpret_cast<void*>(secIndexOffset), secBaseVertex);
      }
      secBaseVertex += 2 + ((c.baseRadius > 0.f) + (c.topRadius > 0.f)) * cylinderSlices;
      secIndexOffset += sizeof(std::uint16_t) * numOfIndices;
    }
  }

  if(!drawing.images.empty() && draw)
  {
    GLboolean cullingWasEnabled;
    if((cullingWasEnabled = f->glIsEnabled(GL_CULL_FACE)))
      f->glDisable(GL_CULL_FACE);

    // Bind VAO and shader (transformation matrix is set per image).
    f->glBindVertexArray(data->imageVAO);
    f->glUseProgram(data->imageProgram);
    f->glActiveTexture(GL_TEXTURE0);

    for(const DebugDrawing3D::Image3D& i : drawing.images)
    {
      Matrix3f rotation = Matrix3f::Identity();
      if(i.rotation.x() != 0)
        rotation *= RotationMatrix::aroundX(i.rotation.x());
      if(i.rotation.y() != 0)
        rotation *= RotationMatrix::aroundY(i.rotation.y());
      if(i.rotation.z() != 0)
        rotation *= RotationMatrix::aroundZ(i.rotation.z());
      const Matrix4f localTransformation = transformation *
                                           (Matrix4f() << rotation, i.point, RowVector3f::Zero(), 1.f).finished() *
                                           (Matrix4f() << 1.f, 0.f, 0.f, 0.f, 0.f, i.width * 0.5f, 0.f, 0.f, 0.f, 0.f, i.height * 0.5f, 0.f, 0.f, 0.f, 0.f, 1.f).finished();
      f->glUniformMatrix4fv(data->imagePVMLocation, 1, GL_FALSE, localTransformation.data());
      f->glBindTexture(GL_TEXTURE_2D, textures[textureIndex++]);
      f->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }

    if(cullingWasEnabled)
      f->glEnable(GL_CULL_FACE);
  }
  else
    textureIndex += static_cast<unsigned int>(drawing.images.size());

  if(draw && drawing.renderOptions & Drawings3D::disableDepth)
    f->glDepthMask(GL_TRUE);
}

char* PaintMethods3DOpenGL::copyImage(const CameraImage& srcImage)
{
  char* imageData = new char[srcImage.width * srcImage.height * 3];
  unsigned char* p = reinterpret_cast<unsigned char*>(imageData);
  for(int y = srcImage.height - 1; y >= 0; y--)
  {
    const CameraImage::PixelType* cur = &srcImage[y][0];
    const CameraImage::PixelType* end = cur + srcImage.width;
    for(; cur < end; cur++, p += 3)
      ColorModelConversions::fromYUVToRGB(cur->y1, cur->u, cur->v, p[0], p[1], p[2]);
  }
  return imageData;
}
