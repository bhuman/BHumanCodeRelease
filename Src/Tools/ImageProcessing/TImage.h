/**
 * @author Alexis Tsogias
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Platform/BHAssert.h"
#include "Platform/Memory.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/Streamable.h"

#include <cstring>

/**
 * Template class to represent an image of parameterizable pixel type.
 *
 * This was originally written for thumbnail images, but could also be used as a
 * generic image type for other purposes.
 * TODO: unify this with the Image class (which could become a TImage
 * parameterized with the Pixel union)
 */
template<typename Pixel>
struct TImage : public Streamable
{
  using PixelType = Pixel;
  static const int maxWidth = Image::maxResolutionWidth;
  static const int maxHeight = Image::maxResolutionHeight;
  int width;
  int height;
  int yStart = 0; /**< The y coordinate the image starts from. Parts above yStart are left unchanged! */

private:
  void* imagePadding; /**< A pointer to the memory for the image including some padding at the front and back. */
  Pixel* image; /**< A pointer to the memory for the image */

public:
  TImage(int width = maxWidth, int height = maxHeight, int padding = 32);
  TImage(const TImage& other);
  virtual ~TImage();

  TImage& operator=(const TImage& other);

  Pixel* operator[](const int y) { return image + y * width; }
  const Pixel* operator[](const int y) const { return image + y * width; }
  Pixel& operator[](const Vector2s& p) { return image[p.y() * width + p.x()]; }
  const Pixel& operator[](const Vector2s& p) const { return image[p.y() * width + p.x()]; }
  Pixel& operator[](const Vector2i& p) { return image[p.y() * width + p.x()]; }
  const Pixel& operator[](const Vector2i& p) const { return image[p.y() * width + p.x()]; }

  /**
   * Get the pixel at x,y. No boundary check!
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @return The requested pixel.
   */
  PixelType& operator()(int x, int y) { return *(image + (y * width + x)); }

  /**
   * Get the const pixel at x,y. No boundary check!
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @return The const requested pixel.
   */
  inline const PixelType& operator()(int x, int y) const { return *(image + (y * width + x)); }

  void setResolution(int width, int height);

protected:
  virtual void serialize(In* in, Out* out);
};

template<typename Pixel>
TImage<Pixel>::TImage(int width, int height, int padding) :
  width(width), height(height)
{
  ASSERT(width <= maxWidth);
  ASSERT(height <= maxHeight);
  ASSERT(padding % 32 == 0); //we need thisfor alignment
  imagePadding = Memory::alignedMalloc(maxWidth * maxHeight * sizeof(Pixel) + (padding * 2), 32);
  image = reinterpret_cast<Pixel*>(reinterpret_cast<char*>(imagePadding) + padding);
}

template<typename Pixel>
TImage<Pixel>::TImage(const TImage& other) :
  width(other.width), height(other.height)
{
  int padding = static_cast<int>(reinterpret_cast<const char*>(other.image) - reinterpret_cast<const char*>(other.imagePadding));
  imagePadding = Memory::alignedMalloc(maxWidth * maxHeight * sizeof(Pixel) + (padding * 2), 16);
  image = reinterpret_cast<Pixel*>(reinterpret_cast<char*>(imagePadding) + padding);
  memcpy(image, other.image, width * height * sizeof(Pixel));
}

template<typename Pixel>
TImage<Pixel>::~TImage()
{
  Memory::alignedFree(imagePadding);
}

template<typename Pixel>
TImage<Pixel>& TImage<Pixel>::operator=(const TImage& other)
{
  ASSERT(reinterpret_cast<const char*>(image) - reinterpret_cast<const char*>(imagePadding)
         >= reinterpret_cast<const char*>(other.image) - reinterpret_cast<const char*>(other.imagePadding));
  width = other.width;
  height = other.height;
  memcpy(image, other.image, maxWidth * maxHeight * sizeof(Pixel));
  return *this;
}

template<typename Pixel>
void TImage<Pixel>::setResolution(int width, int height)
{
  ASSERT(width <= maxWidth);
  ASSERT(height <= maxHeight);
  this->width = width;
  this->height = height;
}

template<typename Pixel>
void TImage<Pixel>::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);

  if(out)
    out->write(image, width * height * sizeof(Pixel));
  else
    in->read(image, width * height * sizeof(Pixel));

  STREAM_REGISTER_FINISH;
}
