/**
 * @file SimulatedNao/Views/ColorSpaceView.cpp
 *
 * Implementation of class ColorSpaceView
 *
 * @author Thomas Röfer
 * @author Colin Graf
 * @author Arne Hasselbring
 */

#include "ColorSpaceView.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include "SimulatedNao/RobotConsole.h"
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QSettings>

static const char* cubeVertexShader = R"glsl(
#version 330 core
uniform mat4 cameraPV;
const vec3 vertices[] = vec3[]
(
  vec3(-1.0, -1.0, -1.0), vec3(-1.0, -1.0, 1.0),
  vec3(-1.0, 1.0, -1.0), vec3(-1.0, 1.0, 1.0),
  vec3(1.0, -1.0, -1.0), vec3(1.0, -1.0, 1.0),
  vec3(1.0, 1.0, -1.0), vec3(1.0, 1.0, 1.0),
  vec3(-1.0, -1.0, -1.0), vec3(-1.0, 1.0, -1.0),
  vec3(-1.0, 1.0, -1.0), vec3(1.0, 1.0, -1.0),
  vec3(1.0, 1.0, -1.0), vec3(1.0, -1.0, -1.0),
  vec3(1.0, -1.0, -1.0), vec3(-1.0, -1.0, -1.0),
  vec3(-1.0, -1.0, 1.0), vec3(-1.0, 1.0, 1.0),
  vec3(-1.0, 1.0, 1.0), vec3(1.0, 1.0, 1.0),
  vec3(1.0, 1.0, 1.0), vec3(1.0, -1.0, 1.0),
  vec3(1.0, -1.0, 1.0), vec3(-1.0, -1.0, 1.0)
);
void main()
{
  gl_Position = cameraPV * vec4(vertices[gl_VertexID], 1.0);
}
)glsl";

static const char* cubeFragmentShader = R"glsl(
#version 330 core
uniform vec3 color;
out vec4 outFragColor;
void main()
{
  outFragColor = vec4(color, 1.0);
}
)glsl";

static const char* pixelVertexShader = R"glsl(
#version 330 core

uniform mat4 cameraPV;
uniform sampler2D rgbImage;
uniform sampler2D convertedImage;
out vec4 Color;

vec3 getPosition(in vec2 texCoords, in vec4 channels);

void main()
{
  ivec2 imageSize = textureSize(rgbImage, 0);
  vec2 texCoords = (vec2(gl_VertexID % imageSize.x, gl_VertexID / imageSize.x) + 0.5) / imageSize;
  Color = texture(rgbImage, texCoords);
  gl_Position = cameraPV * vec4(getPosition(texCoords, texture(convertedImage, texCoords)), 1.0);
}

#define ELEMENT )glsl"; // This line is completed dynamically.

static const char* pixelVertexShaderAllChannels = R"glsl(
vec3 getPosition(in vec2 texCoords, in vec4 channels)
{
  return -1.0 + channels.ELEMENT * 2.0;
}
)glsl";

static const char* pixelVertexShaderChannel = R"glsl(
vec3 getPosition(in vec2 texCoords, in vec4 channels)
{
  return vec3(-1.0 + texCoords.x * 2.0, 1.0 - texCoords.y * 2.0, -1.0 + channels.ELEMENT * 2.0);
}
)glsl";

static const char* pixelFragmentShader = R"glsl(
#version 330 core
in vec4 Color;
out vec4 outFragColor;
void main()
{
  outFragColor = Color;
}
)glsl";

class ColorSpaceWidget : public QOpenGLWidget, public QOpenGLFunctions_3_3_Core, public SimRobot::Widget
{
public:
  ColorSpaceWidget(ColorSpaceView& view) : view(view)
  {
    setFocusPolicy(Qt::StrongFocus);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view.fullName);
    rotation = QPointF(settings.value("RotationX").toFloat(), settings.value("RotationY").toFloat());
    settings.endGroup();
  }

  ~ColorSpaceWidget()
  {
    makeCurrent();
    if(vao != 0)
      glDeleteVertexArrays(1, &vao);
    if(rgbTextureID != 0)
      glDeleteTextures(1, &rgbTextureID);
    if(convertedTextureID != 0)
      glDeleteTextures(1, &convertedTextureID);
    if(cubeProgram != 0)
      glDeleteProgram(cubeProgram);
    if(pixelProgram != 0)
      glDeleteProgram(pixelProgram);

    QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
    settings.beginGroup(view.fullName);
    settings.setValue("RotationX", rotation.rx());
    settings.setValue("RotationY", rotation.ry());
    settings.endGroup();
  }

private:
  QPointF rotation;
  int width;
  int height;
  ColorSpaceView& view;
  bool dragging = false;
  QPoint dragStart;

  GLuint vao = 0;
  GLuint rgbTextureID = 0;
  GLuint convertedTextureID = 0;
  GLuint cubeProgram = 0;
  GLuint pixelProgram = 0;
  GLuint cubeCameraPVLocation;
  GLuint cubeColorLocation;
  GLuint pixelCameraPVLocation;
  GLuint pixelRGBImageLocation;
  GLuint pixelConvertedImageLocation;
  unsigned int imageWidth = 0;
  unsigned int imageHeight = 0;

  void initializeGL() override
  {
    initializeOpenGLFunctions();
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(1.5f); // required
    glPointSize(2.5f);
    glEnable(GL_DEPTH_TEST);

    glGenVertexArrays(1, &vao);
    // The VAO is intentionally left empty - no shader uses vertex attributes, but it must be bound anyway.
    glGenTextures(1, &rgbTextureID);
    glGenTextures(1, &convertedTextureID);

    auto compileShader = [this](const std::vector<const char*>& vertexShaderSources, const char* fragmentShaderSource)
    {
      const auto vertexShader = glCreateShader(GL_VERTEX_SHADER);
      glShaderSource(vertexShader, static_cast<GLsizei>(vertexShaderSources.size()), vertexShaderSources.data(), nullptr);
      glCompileShader(vertexShader);
      GLint status;
      glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &status);
      if(!status)
      {
        char message[512];
        glGetShaderInfoLog(vertexShader, sizeof(message), nullptr, message);
        FAIL(message);
      }
      const auto fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
      glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
      glCompileShader(fragmentShader);
      glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &status);
      if(!status)
      {
        char message[512];
        glGetShaderInfoLog(fragmentShader, sizeof(message), nullptr, message);
        FAIL(message);
      }
      const auto program = glCreateProgram();
      glAttachShader(program, vertexShader);
      glAttachShader(program, fragmentShader);
      glLinkProgram(program);
      glGetProgramiv(program, GL_LINK_STATUS, &status);
      if(!status)
      {
        char message[512];
        glGetProgramInfoLog(program, sizeof(message), nullptr, message);
        FAIL(message);
      }
      glDeleteShader(vertexShader);
      glDeleteShader(fragmentShader);
      return program;
    };

    cubeProgram = compileShader({cubeVertexShader}, cubeFragmentShader);
    cubeCameraPVLocation = glGetUniformLocation(cubeProgram, "cameraPV");
    cubeColorLocation = glGetUniformLocation(cubeProgram, "color");

    const char* elements[][4] =
    {
      {"zyw", "z", "y", "w"}, // YUV
      {"zyx", "z", "y", "x"}, // RGB
      {"xyz", "x", "y", "z"}  // HSI
    };
    pixelProgram = compileShader({pixelVertexShader, elements[view.colorModel][view.channel], view.channel ? pixelVertexShaderChannel : pixelVertexShaderAllChannels}, pixelFragmentShader);
    pixelCameraPVLocation = glGetUniformLocation(pixelProgram, "cameraPV");
    pixelRGBImageLocation = glGetUniformLocation(pixelProgram, "rgbImage");
    pixelConvertedImageLocation = glGetUniformLocation(pixelProgram, "convertedImage");
  }

  void resizeGL(int newWidth, int newHeight) override
  {
    width = newWidth;
    height = newHeight;
  }

  void paintGL() override
  {
    if(view.needsUpdate())
    {
      SYNC_WITH(view.console);
      DebugImage* image = nullptr;

      RobotConsole::Images& currentImages = view.console.threadData[view.threadName].images;
      RobotConsole::Images::const_iterator i = currentImages.find(view.name);

      if(i != currentImages.end())
        image = i->second.image;
      if(image)
      {
        if(image->width > 0)
        {
          Image<PixelTypes::BGRAPixel> bgraImage(image->getImageWidth(), image->height);
          view.console.debugImageConverter.convertToBGRA(*image, bgraImage[0]);

          glActiveTexture(GL_TEXTURE0);
          glBindTexture(GL_TEXTURE_2D, rgbTextureID);
          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, bgraImage.width, bgraImage.height, 0, GL_BGRA, GL_UNSIGNED_BYTE, bgraImage[0]);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

          union ColorChannels
          {
            unsigned char channels[4];
            unsigned int color;
          };

          ColorChannels* convertedImage;
          bool allocated = false;
          switch(view.colorModel)
          {
            case ColorSpaceView::YUV:
              if(image->type == PixelTypes::YUV)
              {
                convertedImage = const_cast<ColorChannels*>(image->getView<ColorChannels>()[0]);
              }
              else
              {
                convertedImage = new ColorChannels[bgraImage.width * bgraImage.height];
                allocated = true;
                PixelTypes::YUVPixel* dest = reinterpret_cast<PixelTypes::YUVPixel*>(convertedImage);
                switch(image->type)
                {
                  case PixelTypes::BGRA:
                    for(const PixelTypes::BGRAPixel* src = image->getView<PixelTypes::BGRAPixel>()[0], *srcEnd = image->getView<PixelTypes::BGRAPixel>()[image->height]; src < srcEnd; src++, dest++)
                      ColorModelConversions::fromRGBToYUV(src->r, src->g, src->b, dest->y, dest->u, dest->v);
                    break;
                  case PixelTypes::RGB:
                    for(const PixelTypes::RGBPixel* src = image->getView<PixelTypes::RGBPixel>()[0], *srcEnd = image->getView<PixelTypes::RGBPixel>()[image->height]; src < srcEnd; src++, dest++)
                      ColorModelConversions::fromRGBToYUV(src->r, src->g, src->b, dest->y, dest->u, dest->v);
                    break;
                  case PixelTypes::YUYV:
                    for(const PixelTypes::YUYVPixel* src = image->getView<PixelTypes::YUYVPixel>()[0], *srcEnd = image->getView<PixelTypes::YUYVPixel>()[image->height]; src < srcEnd; src++, dest++)
                    {
                      dest->y = src->y0;
                      dest->u = src->u;
                      dest->v = src->v;
                      dest++;
                      dest->y = src->y1;
                      dest->u = src->u;
                      dest->v = src->v;
                    }
                    break;
                  case PixelTypes::Grayscale:
                    for(const PixelTypes::GrayscaledPixel* src = image->getView<PixelTypes::GrayscaledPixel>()[0], *srcEnd = image->getView<PixelTypes::GrayscaledPixel>()[image->height]; src < srcEnd; src++, dest++)
                    {
                      dest->y = *src;
                      dest->u = dest->v = 128;
                    }
                    break;
                  case PixelTypes::Edge2:
                    for(const PixelTypes::Edge2Pixel* src = image->getView<PixelTypes::Edge2Pixel>()[0], *srcEnd = image->getView<PixelTypes::Edge2Pixel>()[image->height]; src < srcEnd; src++, dest++)
                    {
                      dest->y = 255;
                      dest->u = reinterpret_cast<const unsigned char*>(src)[0];
                      dest->v = reinterpret_cast<const unsigned char*>(src)[1];
                    }
                    break;
                  default:
                    ASSERT(false);
                }
              }
              break;
            case ColorSpaceView::RGB:
              convertedImage = reinterpret_cast<ColorChannels*>(bgraImage[0]);
              break;
            case ColorSpaceView::HSI:
            {
              convertedImage = new ColorChannels[bgraImage.width * bgraImage.height];
              allocated = true;
              PixelTypes::HSIPixel* dest = reinterpret_cast<PixelTypes::HSIPixel*>(convertedImage);
              for(const PixelTypes::BGRAPixel* src = bgraImage[0], *srcEnd = bgraImage[bgraImage.height]; src < srcEnd; src++, dest++)
                ColorModelConversions::fromRGBToHSI(src->r, src->g, src->b, dest->h, dest->s, dest->i);
              break;
            }
            default:
              ASSERT(false);
              goto end;
          }

          glActiveTexture(GL_TEXTURE1);
          glBindTexture(GL_TEXTURE_2D, convertedTextureID);
          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, bgraImage.width, bgraImage.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, convertedImage);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

          imageWidth = bgraImage.width;
          imageHeight = bgraImage.height;

          if(allocated)
            delete[] convertedImage;
        end:;
        }
        view.lastTimestamp = i->second.timestamp;
      }
      else
      {
        imageWidth = imageHeight = 0;
        view.lastTimestamp = 0;
      }
    }

    glClearColor(view.background.x(), view.background.y(), view.background.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    view.lastBackground = view.background;

    if(!imageWidth || !imageHeight)
      return;

    const float aspect = height ? static_cast<float>(width) / static_cast<float>(height) : static_cast<float>(width);
    Matrix4f pvMatrix = Matrix4f::Zero();
    pvMatrix(0, 0) = 1.f / std::tan(0.5f * 5.f / 36.f * pi) / aspect;
    pvMatrix(1, 1) = 1.f / std::tan(0.5f * 5.f / 36.f * pi);
    pvMatrix(2, 2) = 101.f / -99.f;
    pvMatrix(3, 2) = -1.f;
    pvMatrix(2, 3) = 200.f / -99.f;
    pvMatrix *= (Matrix4f() << 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, -8.f, 0.f, 0.f, 0.f, 1.f).finished();
    {
      const float c = std::cos(static_cast<float>(rotation.x()) * pi / 180.f);
      const float s = std::sin(static_cast<float>(rotation.x()) * pi / 180.f);
      pvMatrix *= (Matrix4f() << 1.f, 0.f, 0.f, 0.f, 0.f, c, -s, 0.f, 0.f, s, c, 0.f, 0.f, 0.f, 0.f, 1.f).finished();
    }
    {
      const float c = std::cos(static_cast<float>(rotation.y()) * pi / 180.f);
      const float s = std::sin(static_cast<float>(rotation.y()) * pi / 180.f);
      pvMatrix *= (Matrix4f() << c, -s, 0.f, 0.f, s, c, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f).finished();
    }
    if(view.channel)
    {
      const float scaleX = std::max(1.f, static_cast<float>(imageWidth) / imageHeight);
      const float scaleY = std::max(1.f, static_cast<float>(imageHeight) / imageWidth);
      pvMatrix *= (Matrix4f() << scaleX, 0.f, 0.f, 0.f, 0.f, scaleY, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f).finished();
    }

    glViewport(0, 0, static_cast<GLsizei>(width * devicePixelRatio()), static_cast<GLsizei>(height * devicePixelRatio()));

    glBindVertexArray(vao);

    glUseProgram(cubeProgram);
    glUniformMatrix4fv(cubeCameraPVLocation, 1, GL_FALSE, pvMatrix.data());
    glUniform3f(cubeColorLocation, static_cast<float>(static_cast<int>(view.background.x() * 255.f) ^ 0xc0) / 255.f,
                                   static_cast<float>(static_cast<int>(view.background.y() * 255.f) ^ 0xc0) / 255.f,
                                   static_cast<float>(static_cast<int>(view.background.z() * 255.f) ^ 0xc0) / 255.f);
    glDrawArrays(GL_LINES, 0, 24);

    glUseProgram(pixelProgram);
    glUniformMatrix4fv(pixelCameraPVLocation, 1, GL_FALSE, pvMatrix.data());
    glUniform1i(pixelRGBImageLocation, 0);
    glUniform1i(pixelConvertedImageLocation, 1);
    glDrawArrays(GL_POINTS, 0, imageWidth * imageHeight);
  }

  void mousePressEvent(QMouseEvent* event) override
  {
    QWidget::mousePressEvent(event);

    if(event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton)
    {
      dragStart = event->pos();
      dragging = true;
    }
  }

  void mouseReleaseEvent(QMouseEvent* event) override
  {
    QWidget::mouseReleaseEvent(event);

    dragging = false;
  }

  void mouseMoveEvent(QMouseEvent* event) override
  {
    QWidget::mouseMoveEvent(event);

    if(dragging)
    {
      QPoint diff(event->pos() - dragStart);
      dragStart = event->pos();
      rotation.ry() += diff.x();
      rotation.rx() += diff.y();
      QOpenGLWidget::update();
    }
  }

  void mouseDoubleClickEvent(QMouseEvent* event) override
  {
    QWidget::mouseDoubleClickEvent(event);

    rotation = QPointF();
    QOpenGLWidget::update();
  }

  void wheelEvent(QWheelEvent* event) override
  {
    if(!event->angleDelta().isNull())
    {
      rotation.ry() += event->angleDelta().x() * 0.2f;
      rotation.rx() += event->angleDelta().y() * 0.2f;
      QOpenGLWidget::update();
    }
    else
      QOpenGLWidget::wheelEvent(event);
  }

  QSize sizeHint() const override { return QSize(320, 240); }

  QWidget* getWidget() override { return this; }

  void update() override
  {
    if(view.background != view.lastBackground || view.needsUpdate())
      QOpenGLWidget::update();
  }
};

ColorSpaceView::ColorSpaceView(const QString& fullName, RobotConsole& c, const std::string& n, ColorModel cm, int ch, const Vector3f& b, const std::string& threadName) :
  fullName(fullName), icon(":/Icons/icons8-view-50.png"), console(c), name(n), colorModel(cm), channel(ch), background(b), threadName(threadName)
{
  icon.setIsMask(true);
}

bool ColorSpaceView::needsUpdate() const
{
  SYNC_WITH(console);
  DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = console.threadData[threadName].images;
  RobotConsole::Images::const_iterator i = currentImages.find(name);
  if(i != currentImages.end())
    image = i->second.image;
  return ((image && i->second.timestamp != lastTimestamp) ||
          (!image && lastTimestamp));
}

SimRobot::Widget* ColorSpaceView::createWidget()
{
  return new ColorSpaceWidget(*this);
}
