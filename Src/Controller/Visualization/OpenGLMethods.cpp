/**
* @file Controller/Visualization/OpenGLMethods.cpp
* Implementation of class OpenGLMethods.
*
* @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
*/

#include "OpenGLMethods.h"
#include <Platform/OpenGL.h>
#include "Tools/ColorModelConversions.h"

void OpenGLMethods::paintCubeToOpenGLList
(
  int xLow, int yLow, int zLow,
  int xHigh, int yHigh, int zHigh,
  int listID, bool paintFullCube,
  int scale,
  int offsetX, int offsetY, int offsetZ,
  int r, int g, int b,
  bool beginList, bool endList
)
{
  GLUquadricObj* pQuadric = gluNewQuadric();

  if(beginList) glNewList(listID, GL_COMPILE_AND_EXECUTE);

  float xL, yL, zL;
  float xH, yH, zH;

  xH = OpenGLMethods::transformCoordinates(scale, offsetX, xHigh);
  yH = OpenGLMethods::transformCoordinates(scale, offsetY, yHigh);
  zH = OpenGLMethods::transformCoordinates(scale, offsetZ, zHigh);
  xL = OpenGLMethods::transformCoordinates(scale, offsetX, xLow);
  yL = OpenGLMethods::transformCoordinates(scale, offsetY, yLow);
  zL = OpenGLMethods::transformCoordinates(scale, offsetZ, zLow);

  glBegin(GL_LINE_LOOP);
  glColor3ub((unsigned char) r, (unsigned char) g, (unsigned char) b);
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
  if(endList) glEndList();

  gluDeleteQuadric(pQuadric);
}

void OpenGLMethods::paintImagePixelsToOpenGLList
(
  const Image& image,
  int colorModel,
  int zComponent,
  bool polygons,
  int listID,
  int x1,
  int x2,
  int y1,
  int y2
)
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
  {
    glBegin(GL_POINTS);
  }

  float tmp = 2.0f / 255.0f;
  float x, y;
  float xn, yn;
  float z00, z01, z10, z11;

  Image rgbImage;
  rgbImage.convertFromYCbCrToRGB(image);

  Image convertedImage;
  switch(colorModel)
  {
    case 0: //yuv
      convertedImage = image;
      break;
    case 1: //rgb
      convertedImage.convertFromYCbCrToRGB(image);
      break;
    case 2: //hsi
      convertedImage.convertFromYCbCrToHSI(image);
      break;
  }

  for(int j = y1; j < y2 - 2; j++)
  {
    for(int i = x1; i < x2 - 2; i++)
    {
      if(zComponent == -1)
      {
        unsigned char* channels = convertedImage[j][i].channels;
        if(colorModel == 0) //(padding is at 0 by yuv)
          channels++;
        x   = (float)(-1.0f + channels[2] * tmp);
        y   = (float)(-1.0f + channels[0] * tmp);
        z00 = (float)(-1.0f + channels[1] * tmp);
        glColor3ub(rgbImage[j][i].r, rgbImage[j][i].g, rgbImage[j][i].b);
        glVertex3d(x, y, z00);
      }
      else
      {
        x = (float)(+i - image.width / 2) * tmp;
        y = (float)(-j + image.height / 2) * tmp;
        xn = (float)(+i + 1 - image.width / 2) * tmp;
        yn = (float)(-j - 1 + image.height / 2) * tmp;
        z00 = (float)(-0.5f + convertedImage[j][i].channels[zComponent] * tmp / 2);
        if(polygons)
        {
          z01 = (float)(-0.5f + convertedImage[j + 1][i].channels[zComponent] * tmp / 2);
          z10 = (float)(-0.5f + convertedImage[j][i + 1].channels[zComponent] * tmp / 2);
          z11 = (float)(-0.5f + convertedImage[j + 1][i + 1].channels[zComponent] * tmp / 2);
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
  glEnd();
  glPolygonMode(GL_FRONT, GL_LINE);
  glPolygonMode(GL_BACK, GL_LINE);
  ::glEndList();
}

void OpenGLMethods::paintColorTable(const ColorTable& colorTable,
                                    ColorClasses::Color color, int listID)
{
  // Build a new list
  ::glNewList(listID, GL_COMPILE_AND_EXECUTE);

  glBegin(GL_POINTS);

  float tmp = 2.0f / 255.0f;
  float x, y;
  float z00;

  Image::Pixel p;
  // subsampling of color table is independent from its real resolution
  for(int colorSpaceY = 0; colorSpaceY < 256; colorSpaceY += 4)
  {
    p.y = (unsigned char) colorSpaceY;
    for(int colorSpaceU = 0; colorSpaceU < 256; colorSpaceU += 4)
    {
      p.cb = (unsigned char) colorSpaceU;
      for(int colorSpaceV = 0; colorSpaceV < 256; colorSpaceV += 4)
      {
        p.cr = (unsigned char) colorSpaceV;
        if(colorTable[p].is(color))
        {
          x   = (float)(-1.0f + colorSpaceV * tmp);
          y   = (float)(-1.0f + colorSpaceU * tmp);
          z00 = (float)(-1.0f + colorSpaceY * tmp);
          unsigned char r, g, b;
          ColorModelConversions::fromYCbCrToRGB(p.y, p.cb, p.cr, r, g, b);
          glColor3ub(r, g, b);
          glVertex3d(x, y, z00);
        }
      }
    }
  }
  glEnd();
  glPolygonMode(GL_FRONT, GL_LINE);
  glPolygonMode(GL_BACK, GL_LINE);
  ::glEndList();
}
