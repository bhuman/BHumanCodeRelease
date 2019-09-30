/**
 * @file Controller/Visualization/OpenGLMethods.cpp
 * Implementation of class OpenGLMethods.
 *
 * @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
 */

#include "OpenGLMethods.h"
#include <Platform/OpenGL.h>
#include "Controller/RobotConsole.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

/** returns (value+offset)/scale */
float transformCoordinates(int scale, int offset, int value)
{
  return (value + offset) / static_cast<float>(scale);
}

void OpenGLMethods::paintCubeToOpenGLList(int xLow, int yLow, int zLow, int xHigh, int yHigh, int zHigh,
    int listID, bool paintFullCube, int scale, int offsetX, int offsetY, int offsetZ,
    int r, int g, int b, bool beginList, bool endList)
{
  GLUquadricObj* pQuadric = gluNewQuadric();

  if(beginList)
    glNewList(listID, GL_COMPILE_AND_EXECUTE);

  float xH = transformCoordinates(scale, offsetX, xHigh);
  float yH = transformCoordinates(scale, offsetY, yHigh);
  float zH = transformCoordinates(scale, offsetZ, zHigh);
  float xL = transformCoordinates(scale, offsetX, xLow);
  float yL = transformCoordinates(scale, offsetY, yLow);
  float zL = transformCoordinates(scale, offsetZ, zLow);

  glBegin(GL_LINE_LOOP);
  glColor3ub(static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
  glVertex3d(xL, yL, zL);
  glVertex3d(xH, yL, zL);
  glVertex3d(xH, yH, zL);
  glVertex3d(xL, yH, zL);
  glEnd();

  if(paintFullCube)
  {
    glBegin(GL_LINE_LOOP);
    glVertex3d(xL, yL, zH);
    glVertex3d(xH, yL, zH);
    glVertex3d(xH, yH, zH);
    glVertex3d(xL, yH, zH);
    glEnd();

    glBegin(GL_LINES);

    glVertex3d(xL, yL, zL);
    glVertex3d(xL, yL, zH);
    glVertex3d(xH, yL, zL);
    glVertex3d(xH, yL, zH);
    glVertex3d(xH, yH, zL);
    glVertex3d(xH, yH, zH);
    glVertex3d(xL, yH, zL);
    glVertex3d(xL, yH, zH);

    glEnd();
  }
  if(endList)
    glEndList();

  gluDeleteQuadric(pQuadric);
}

union ColorChannels
{
  unsigned char channels[4];
  unsigned int color;
};

void OpenGLMethods::paintImagePixelsToOpenGLList(RobotConsole& console, const DebugImage& image, int colorModel, int zComponent, bool polygons, int listID, int x1, int x2, int y1, int y2)
{
  // Build a new list
  ::glNewList(listID, GL_COMPILE_AND_EXECUTE);

  if(polygons)
  {
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    glBegin(GL_QUADS);
  }
  else
    glBegin(GL_POINTS);

  {
    Image<PixelTypes::BGRAPixel> rgbImage(image.getImageWidth(), image.height);
    console.debugImageConverter.convertToBGRA(image, rgbImage[0]);

    ColorChannels* convertedImage;
    bool allocated = false;
    switch(colorModel)
    {
      case 0: //yuv
        if(image.type == PixelTypes::YUV)
        {
          convertedImage = const_cast<ColorChannels*>(image.getView<ColorChannels>()[0]);
        }
        else
        {
          convertedImage = new ColorChannels[rgbImage.width * rgbImage.height];
          allocated = true;
          PixelTypes::YUVPixel* dest = reinterpret_cast<PixelTypes::YUVPixel*>(convertedImage);
          switch(image.type)
          {
            case PixelTypes::BGRA:
              for(const PixelTypes::BGRAPixel* src = image.getView<PixelTypes::BGRAPixel>()[0], *srcEnd = image.getView<PixelTypes::BGRAPixel>()[image.height]; src < srcEnd; src++, dest++)
                ColorModelConversions::fromRGBToYUV(src->r, src->g, src->b, dest->y, dest->u, dest->v);
              break;
            case PixelTypes::RGB:
              for(const PixelTypes::RGBPixel* src = image.getView<PixelTypes::RGBPixel>()[0], *srcEnd = image.getView<PixelTypes::RGBPixel>()[image.height]; src < srcEnd; src++, dest++)
                ColorModelConversions::fromRGBToYUV(src->r, src->g, src->b, dest->y, dest->u, dest->v);
              break;
            case PixelTypes::YUYV:
              for(const PixelTypes::YUYVPixel* src = image.getView<PixelTypes::YUYVPixel>()[0], *srcEnd = image.getView<PixelTypes::YUYVPixel>()[image.height]; src < srcEnd; src++, dest++)
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
            case PixelTypes::Colored:
              for(const PixelTypes::BGRAPixel* src = rgbImage[0], *srcEnd = rgbImage[rgbImage.height]; src < srcEnd; src++, dest++)
                ColorModelConversions::fromRGBToYUV(src->r, src->g, src->b, dest->y, dest->u, dest->v);
              break;
            case PixelTypes::Grayscale:
              for(const PixelTypes::GrayscaledPixel* src = image.getView<PixelTypes::GrayscaledPixel>()[0], *srcEnd = image.getView<PixelTypes::GrayscaledPixel>()[image.height]; src < srcEnd; src++, dest++)
              {
                dest->y = *src;
                dest->u = dest->v = 128;
              }
              break;
            case PixelTypes::Edge2:
              for(const PixelTypes::Edge2Pixel* src = image.getView<PixelTypes::Edge2Pixel>()[0], *srcEnd = image.getView<PixelTypes::Edge2Pixel>()[image.height]; src < srcEnd; src++, dest++)
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
      case 1: //rgb
        convertedImage = reinterpret_cast<ColorChannels*>(rgbImage[0]);
        break;
      case 2: //hsi
      {
        convertedImage = new ColorChannels[rgbImage.width * rgbImage.height];
        allocated = true;
        PixelTypes::HSIPixel* dest = reinterpret_cast<PixelTypes::HSIPixel*>(convertedImage);
        for(const PixelTypes::BGRAPixel* src = rgbImage[0], *srcEnd = rgbImage[rgbImage.height]; src < srcEnd; src++, dest++)
          ColorModelConversions::fromRGBToHSI(src->r, src->g, src->b, dest->h, dest->s, dest->i);
        break;
      }
      default:
        ASSERT(false);
        goto end;
    }

    float tmp = 2.0f / 255.0f;
    float x, y, xn, yn;
    float z00, z01, z10, z11;

    for(int j = y1; j < y2 - 2; j++)
    {
      for(int i = x1; i < x2 - 2; i++)
      {
        if(zComponent == -1)
        {
          unsigned char* channels = convertedImage[j * rgbImage.width + i].channels;
          if(colorModel == 0) //(padding is at 0 by yuv)
            channels++;
          x = -1.0f + channels[2] * tmp;
          y = -1.0f + channels[0] * tmp;
          z00 = -1.0f + channels[1] * tmp;
          glColor3ub(rgbImage[j][i].r, rgbImage[j][i].g, rgbImage[j][i].b);
          glVertex3d(x, y, z00);
        }
        else
        {
          x = (+i - rgbImage.width / 2.f) * tmp;
          y = (-j + rgbImage.height / 2.f) * tmp;
          xn = (+i + 1 - rgbImage.width / 2.f) * tmp;
          yn = (-j - 1 + rgbImage.height / 2.f) * tmp;
          z00 = (-0.5f + convertedImage[j * rgbImage.width + i].channels[zComponent] * tmp / 2);
          if(polygons)
          {
            z01 = -0.5f + convertedImage[(j + 1) * rgbImage.width + i].channels[zComponent] * tmp / 2;
            z10 = -0.5f + convertedImage[j * rgbImage.width + i + 1].channels[zComponent] * tmp / 2;
            z11 = -0.5f + convertedImage[(j + 1) * rgbImage.width + i + 1].channels[zComponent] * tmp / 2;
            glColor3ub(rgbImage[j][i].r, rgbImage[j][i].g, rgbImage[j][i].b);
            glVertex3d(x, y, z00);

            glColor3ub(rgbImage[j][i + 1].r, rgbImage[j][i + 1].g, rgbImage[j][i + 1].b);
            glVertex3d(xn, y, z10);

            glColor3ub(rgbImage[j + 1][i + 1].r, rgbImage[j + 1][i + 1].g, rgbImage[j + 1][i + 1].b);
            glVertex3d(xn, yn, z11);

            glColor3ub(rgbImage[j + 1][i].r, rgbImage[j + 1][i].g, rgbImage[j + 1][i].b);
            glVertex3d(x, yn, z01);
          }
          else
          {
            glColor3ub(rgbImage[j][i].r, rgbImage[j][i].g, rgbImage[j][i].b);
            glVertex3d(x, y, z00);
          }
        }
      }
    }

    if(allocated)
    {
      delete[] convertedImage;
    }
  }

end:
  glEnd();
  glPolygonMode(GL_FRONT, GL_LINE);
  glPolygonMode(GL_BACK, GL_LINE);
  ::glEndList();
}
