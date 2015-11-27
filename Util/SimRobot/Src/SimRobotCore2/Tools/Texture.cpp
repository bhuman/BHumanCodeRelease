/**
* @file Tools/Texture.cpp
* Implementation of class Texture
* @author <A href="mailto:pachur@informatik.uni-bremen.de">Dennis Pachur</A>
* @author Colin Graf
*/

#include <cstring>
#include <cstdio>
#include "Platform/OpenGL.h"

#include "Platform/Assert.h"
#include "Tools/Texture.h"
#include <QImage>

#ifdef WINDOWS
#define strcasecmp _stricmp
#endif

Texture::~Texture()
{
  glDeleteTextures(1, &textureId);
  if(imageData)
    delete[] imageData;
}

void Texture::createGraphics()
{
  if(imageData)
  {
    //unsigned int textureId;
    //glGenTextures(1, &textureId);
    //ASSERT(textureId > 0);
    //ASSERT(this->textureId == 0 || textureId == this->textureId);
    //this->textureId = textureId;
    glBindTexture(GL_TEXTURE_2D, textureId);

    // mode #1 GL_NEAREST
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, hasAlpha ? 4 : 3, width, height, 0, byteOrder, GL_UNSIGNED_BYTE, imageData);
/*
    // mode #2 GL_LINEAR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, hasAlpha ? 4 : 3, width, height, 0, byteOrder, GL_UNSIGNED_BYTE, imageData);

    // mode #3 GL_NEAREST mipmap
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gluBuild2DMipmaps(GL_TEXTURE_2D, hasAlpha ? 4 : 3, width, height, byteOrder, GL_UNSIGNED_BYTE, imageData);

    // mode #4 GL_LINEAR mipmap
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, hasAlpha ? 4 : 3, width, height, byteOrder, GL_UNSIGNED_BYTE, imageData);
*/
  }
}

bool Texture::load(const std::string& file)
{
  ASSERT(!imageData);
  glGenTextures(1, &textureId);

  if(file.length() >= 4)
  {
    std::string suffix = file.substr(file.length() - 4);
    if(strcasecmp(suffix.c_str(), ".tga") == 0)
      return loadTGA(file);
  }

  QImage image;
  if(image.load(file.c_str()))
  {
    if(image.format() != QImage::Format_ARGB32 &&
       image.format() != QImage::Format_RGB32 &&
       image.format() != QImage::Format_RGB888)
      return false;
    width = image.width();
    height = image.height();
    byteOrder = image.format() == QImage::Format_RGB888 ? GL_BGR : GL_BGRA;
    hasAlpha = image.hasAlphaChannel();
    imageData = new GLubyte[image.byteCount()];
    if(!imageData)
      return false;

    GLubyte* p = imageData;
    for(int y = height; y-- > 0;)
    {
      memcpy(p, image.scanLine(y), image.bytesPerLine());
      p += image.bytesPerLine();
    }

    return true;
  }
  else
    return false;
}

bool Texture::loadTGA(const std::string& name)
{
  GLubyte TGAheader[12] = {0,0,2,0,0,0,0,0,0,0,0,0};   // Uncompressed TGA header
  GLubyte TGAcompare[12];                              // Used to compare TGA header
  GLubyte header[6];                                   // First 6 useful bytes of the header
  GLuint  bytesPerPixel;
  GLuint  imageSize;
  GLuint  bpp;

  FILE *file = fopen(name.c_str(), "rb");               // Open the TGA file

  // Load the file and perform checks
  if(file == nullptr ||                                                      // Does file exist?
    fread(TGAcompare,1,sizeof(TGAcompare),file) != sizeof(TGAcompare) ||  // Are there 12 bytes to read?
    memcmp(TGAheader,TGAcompare,sizeof(TGAheader)) != 0 ||                // Is it the right format?
    fread(header,1,sizeof(header),file) != sizeof(header))                // If so then read the next 6 header bytes
  {
    if(file == nullptr) // If the file didn't exist then return
      return false;
    else
    {
      fclose(file); // If something broke then close the file and return
      return false;
    }
  }

  // Determine the TGA width and height (highbyte*256+lowbyte)
  width  = header[1] * 256 + header[0];
  height = header[3] * 256 + header[2];

  // Check to make sure the targa is valid
  if(width <= 0 || height <= 0)
  {
    fclose(file);
    return false;
  }
  // Only 24 or 32 bit images are supported
  if( (header[4] != 24 && header[4] != 32) )
  {
    fclose(file);
    return false;
  }

  bpp = header[4];  // Grab the bits per pixel
  bytesPerPixel = bpp/8;  // Divide by 8 to get the bytes per pixel
  imageSize = width * height * bytesPerPixel; // Calculate the memory required for the data

  // Allocate the memory for the image data
  imageData = new GLubyte[imageSize];
  if(!imageData)
  {
    fclose(file);
    return false;
  }

  // Load the image data
  if(fread(imageData, 1, imageSize, file) != imageSize)  // Does the image size match the memory reserved?
  {
    delete [] imageData;  // If so, then release the image data
    fclose(file); // Close the file
    return false;
  }

  // We are done with the file so close it
  fclose(file);

  byteOrder = bpp == 24 ? GL_BGR : GL_BGRA;
  hasAlpha = byteOrder == GL_BGRA;
  return true;
}
