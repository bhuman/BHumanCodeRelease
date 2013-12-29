/**
 * @file Image.cpp
 *
 * Implementation of class Image.
 */

#include <cstring>

#include "Image.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Platform/BHAssert.h"

Image::Image(bool initialize, int width, int height) :
  timeStamp(0),
  isReference(false),
  width(width),
  height(height),
  widthStep(width * 2)
{
  // allocate full size image and keep it that way indepentent of resolution
  image = new Pixel[maxResolutionWidth * maxResolutionHeight * 2];
  if(initialize)
    for(int y = 0; y < height; ++y)
      for(Pixel* p = (*this)[y], *pEnd = p + width; p < pEnd; ++p)
        p->color = 0x80008000;
}

Image::Image(const Image& other)
  : isReference(true)
{
  *this = other;
}

Image::~Image()
{
  if(!isReference)
    delete[] image;
}

Image& Image::operator=(const Image& other)
{
  height = other.height;
  width = other.width;
  widthStep = 2 * width;
  timeStamp = other.timeStamp;
  if(isReference)
  {
    // allocate full size image and keep it that way indepentent of resolution
    image = new Pixel[maxResolutionHeight * maxResolutionWidth * 2];
    isReference = false;
  }
  for(int y = 0; y < height; ++y)
    memcpy((*this)[y], other[y], width * sizeof(Image::Pixel));
  return *this;
}

void Image::setImage(const unsigned char* buffer)
{
  if(!isReference)
  {
    delete[] image;
    isReference = true;
  }
  image = (Pixel*) buffer;
}

void Image::convertFromYCbCrToRGB(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromYCbCrToRGB(ycbcrImage[y][x].y,
                                            ycbcrImage[y][x].cb,
                                            ycbcrImage[y][x].cr,
                                            (*this)[y][x].r,
                                            (*this)[y][x].g,
                                            (*this)[y][x].b);
}

void Image::convertFromRGBToYCbCr(const Image& rgbImage)
{
  height = rgbImage.height;
  width = rgbImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromRGBToYCbCr(rgbImage[y][x].r,
                                            rgbImage[y][x].g,
                                            rgbImage[y][x].b,
                                            (*this)[y][x].y,
                                            (*this)[y][x].cb,
                                            (*this)[y][x].cr);
}

void Image::convertFromYCbCrToHSI(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromYCbCrToHSI(ycbcrImage[y][x].y,
                                            ycbcrImage[y][x].cb,
                                            ycbcrImage[y][x].cr,
                                            (*this)[y][x].h,
                                            (*this)[y][x].s,
                                            (*this)[y][x].i);
}

void Image::convertFromHSIToYCbCr(const Image& hsiImage)
{
  height = hsiImage.height;
  width = hsiImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromHSIToYCbCr(hsiImage[y][x].h,
                                            hsiImage[y][x].s,
                                            hsiImage[y][x].i,
                                            (*this)[y][x].y,
                                            (*this)[y][x].cb,
                                            (*this)[y][x].cr);
}

void Image::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);
  STREAM(timeStamp);

  if(out)
    for(int y = 0; y < height; ++y)
      out->write((*this)[y], width * sizeof(Pixel));
  else
  {
    widthStep = width * 2;
    for(int y = 0; y < height; ++y)
      in->read((*this)[y], width * sizeof(Pixel));
  }

  STREAM_REGISTER_FINISH;
}

float Image::getColorDistance(const Image::Pixel& a, const Image::Pixel& b)
{
  int dy = int(a.y) - b.y;
  int dcb = int(a.cb) - b.cb;
  int dcr = int(a.cr) - b.cr;
  dy *= dy;
  dcb *= dcb;
  dcr *= dcr;
  return std::sqrt(float(dy + dcb + dcr));
}
