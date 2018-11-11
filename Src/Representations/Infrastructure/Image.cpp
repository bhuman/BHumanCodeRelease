/**
 * @file Image.cpp
 *
 * Implementation of struct Image.
 */

#include <cstring>

#include "Image.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/DebugDrawings.h"

constexpr int Image::maxResolutionWidth;
constexpr int Image::maxResolutionHeight;

Image::Image(bool initialize, int width, int height) :
  width(width), height(height), widthStep(width * 2)
{
  ASSERT(width <= maxResolutionWidth);
  ASSERT(height <= maxResolutionHeight);

  // allocate full size image and keep it that way indepentent of resolution
  image = new Pixel[maxResolutionWidth * maxResolutionHeight * 2];
  if(initialize)
    for(int y = 0; y < height; ++y)
      for(Pixel* p = (*this)[y], *pEnd = p + width; p < pEnd; ++p)
        p->color = 0x80008000;
}

Image::Image(const Image& other) :
  isReference(true)
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
  isFullSize = other.isFullSize;

  if(isReference)
  {
    // allocate full size image and keep it that way independent of resolution
    image = new Pixel[maxResolutionHeight * maxResolutionWidth * 2];
    isReference = false;
  }

  const int size = width * sizeof(Pixel) * (isFullSize ? 2 : 1);
  for(int y = 0; y < height; ++y)
    memcpy((*this)[y], other[y], size);

  return *this;
}

void Image::fromCameraImage(const CameraImage& other)
{
  height = other.height / 2;
  width = other.width;
  widthStep = 2 * width;
  timeStamp = other.timestamp;
  isFullSize = true;

  if(isReference)
  {
    // allocate full size image and keep it that way independent of resolution
    image = new Pixel[maxResolutionHeight * maxResolutionWidth * 2];
    isReference = false;
  }

  memcpy(image, other[0], other.width * other.height * sizeof(Pixel));
}

void Image::setImage(unsigned char* buffer)
{
  setImage(reinterpret_cast<Pixel*>(buffer));
}

void Image::setImage(Pixel* buffer)
{
  if(!isReference)
  {
    delete[] image;
    isReference = true;
  }
  image = buffer;
}

void Image::getSubImage(int x1, int y1, int x2, int y2, Image& subImage) const
{
  ASSERT(x1 <= x2);
  ASSERT(y1 <= y2);

  int width = x2 - x1;
  int height = y2 - y1;

  subImage.setResolution(width, height);

  for(int y = 0; y < height; ++y)
    if(y + y1 < 0 || y + y1 >= this->height)
      std::memset(subImage[y], 0x80, sizeof(Pixel) * width);
    else
      for(int x = 0; x < width; ++x)
      {
        if(x + x1 < 0 || x + x1 >= this->width)
          subImage[y][x].color = 0x80808080;
        else
          subImage[y][x] = (*this)[y1 + y][x1 + x];
      }
}

void Image::convertFromYCbCrToRGB(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromYUVToRGB(ycbcrImage[y][x].y,
                                          ycbcrImage[y][x].cb,
                                          ycbcrImage[y][x].cr,
                                          (*this)[y][x].r,
                                          (*this)[y][x].g,
                                          (*this)[y][x].b);
}

void Image::convertFromYCbCr422ToRGB(const Image& ycbcrImage)
{
  height = ycbcrImage.height * 2;
  width = ycbcrImage.width * 2;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
    {
      const Pixel pixel = ycbcrImage.getFullSizePixel(y, x);
      ColorModelConversions::fromYUVToRGB(pixel.y,
                                          pixel.cb,
                                          pixel.cr,
                                          (*this)[y][x].r,
                                          (*this)[y][x].g,
                                          (*this)[y][x].b);
    }
}

void Image::setResolution(int newWidth, int newHeight, bool fullSize)
{
  ASSERT(width <= maxResolutionWidth);
  ASSERT(height <= maxResolutionHeight);

  width = newWidth;
  height = newHeight;
  widthStep = width * 2;
  isFullSize = fullSize;
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

void Image::serialize(In* in, Out* out)
{
  STREAM(width);
  STREAM(height);
  if(isFullSize)
    timeStamp |= 1 << 31;
  STREAM(timeStamp);
  isFullSize = (timeStamp & 1 << 31) != 0;
  timeStamp &= ~(1 << 31);

  const int size = width * sizeof(Pixel) * (isFullSize ? 2 : 1);

  if(out)
    for(int y = 0; y < height; ++y)
      out->write((*this)[y], size);
  else
  {
    widthStep = width * 2;
    for(int y = 0; y < height; ++y)
      in->read((*this)[y], size);
  }
}

void Image::reg()
{
  PUBLISH(reg);
  REG_CLASS(Image);
  REG(width);
  REG(height);
  REG(timeStamp);
}

void SegmentedImage::draw() const
{
  SEND_DEBUG_IMAGE("SegmentedImage", *this, PixelTypes::RGB);

  DEBUG_DRAWING("representation:SegmentedImage:image", "drawingOnImage")
  {
    std::vector<Pixel> objects;
    objects.push_back(Pixel());
    objects.back().cb = 1;
    objects.back().y = 225;
    objects.back().cr = 149;
    objects.push_back(Pixel());
    objects.back().cb = 85;
    objects.back().y = 75;
    objects.back().cr = 74;
    objects.push_back(Pixel());
    objects.back().cb = 112;
    objects.back().y = 59;
    objects.back().cr = 177;
    objects.push_back(Pixel());
    objects.back().cb = 255;
    objects.back().y = 29;
    objects.back().cr = 107;
    objects.push_back(Pixel());
    objects.back().cb = 147;
    objects.back().y = 152;
    objects.back().cr = 201;

    for(Pixel& p : objects)
    {
      int xMin = width * 2;
      int xMax = 0;
      int yMin = height * 2;
      int yMax = 0;
      int counter = 0;
      for(int y = 0; y < height * 2; y++)
      {
        for(int x = 0; x < width * 2; x++)
        {
          if(p.cb == this->getFullSizePixel(y, x).cb &&
             p.y == this->getFullSizePixel(y, x).y &&
             p.cr == this->getFullSizePixel(y, x).cr)
          {
            counter++;
            xMin = std::min(xMin, x);
            yMin = std::min(yMin, y);
            xMax = std::max(xMax, x);
            yMax = std::max(yMax, y);
          }
        }
      }
      if(counter > 50)
        RECTANGLE("representation:SegmentedImage:image", xMin, yMin, xMax, yMax, 3, Drawings::solidPen, ColorRGBA::brown);
    }
  }
};
