/**
 * @author Alexis Tsogias
 * @author Nils Weller
 * @author Felix Thielke
 * @author Bernd Poppinga
 */

#pragma once

#include "ImageProcessing/PixelTypes.h"
#include "Math/Eigen.h"
#include "Platform/BHAssert.h"
#include "Streaming/Streamable.h"
#include <vector>

/**
 * Template class to represent an image of parameterizable pixel type.
 */
template<typename Pixel>
class Image : public Streamable
{
public:
  using PixelType = Pixel;
  unsigned int width;
  unsigned int height;

private:
  std::vector<unsigned char> allocator;

protected:
  Pixel* image; /**< A pointer to the memory for the image */

public:
  Image() : width(0), height(0) {}
  Image(const unsigned int width, const unsigned int height, const unsigned int padding = 0)
  {
    setResolution(width, height, padding);
  }
  Image(const Image& other)
  {
    (*this) = other;
  }

  Image& operator=(const Image<Pixel>& other)
  {
    setResolution(other.width, other.height);
    memcpy(image, other[0], other.width * other.height * sizeof(Pixel));
    return *this;
  }

  Pixel* operator[](const size_t y) { return image + y * width; }
  const Pixel* operator[](const size_t y) const { return image + y * width; }
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
  PixelType& operator()(const size_t x, const size_t y) { return *(image + (y * width + x)); }

  /**
   * Get the const pixel at x,y. No boundary check!
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @return The const requested pixel.
   */
  const PixelType& operator()(const size_t x, const size_t y) const { return *(image + (y * width + x)); }

  virtual void setResolution(const unsigned int width, const unsigned int height, const unsigned int padding = 0)
  {
    this->width = width;
    this->height = height;

    if(allocator.size() < width * height * sizeof(Pixel) + padding * 2)
    {
      allocator.resize(width * height * sizeof(Pixel) + 31 + padding * 2);
      image = reinterpret_cast<Pixel*>(reinterpret_cast<ptrdiff_t>(allocator.data() + 31 + padding) & (~ptrdiff_t(31)));
    }
  }

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override
  {
    PUBLISH(reg);
    STREAM(width);
    STREAM(height);
    setResolution(width, height, allocator.empty() ? 0 : static_cast<unsigned>(reinterpret_cast<unsigned char*>(image) - allocator.data()));
    stream.read(image, width * height * sizeof(Pixel));
  }

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override
  {
    PUBLISH(reg);
    STREAM(width);
    STREAM(height);
    stream.write(image, width * height * sizeof(Pixel));
  }

private:
  static void reg()
  {
    REG_CLASS(Image);
    REG(width);
    REG(height);
  }
};

using GrayscaledImage = Image<PixelTypes::GrayscaledPixel>;
using YUYVImage = Image<PixelTypes::YUYVPixel>;
using BGRAPixel = Image<PixelTypes::BGRAPixel>;

template<typename Pixel>
class ImageWrapper : public Image<Pixel>
{
public:
  ImageWrapper(const unsigned int width, const unsigned int height, Pixel* theImage)
  {
    this->image = theImage;

    this->width = width;
    this->height = height;
  }

  void setResolution(const unsigned int, const unsigned int, const unsigned int = 0) override
  {
    FAIL("ImageWrapper does not manage its image storage itself.");
  }
};
