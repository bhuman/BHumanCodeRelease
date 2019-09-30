/**
 * @author Alexis Tsogias
 * @author Nils Weller
 * @author Felix Thielke
 * @author Bernd Poppinga
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Platform/File.h"

#ifndef TARGET_ROBOT

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QImage>
#include <QDir>
#include <QFile>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#endif

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

#ifndef TARGET_ROBOT
  static std::string expandImageFileName(const std::string& fileName, int imageNumber)
  {
    std::string name = fileName;
    std::string ext = ".png";
    const std::string::size_type p = name.rfind('.');
    const size_t lastDirectorySeparatorIndex = static_cast<int>(name.find_last_of("\\/"));
    if(static_cast<int>(p) > static_cast<int>(lastDirectorySeparatorIndex))
    {
      ext = name.substr(p);
      name = name.substr(0, p);
    }

    if(imageNumber >= 0)
    {
      char num[12];
      sprintf(num, "%06d", imageNumber);
      if(name != "" && lastDirectorySeparatorIndex != fileName.length() - 1)   // only if nonempty base name supplied
        name += "_";
      name += num + ext;
    }
    else
      name += ext;

    for(unsigned i = 0; i < name.size(); ++i)
      if(name[i] == '\\')
        name[i] = '/';
    if(name[0] != '/' && (name.size() < 2 || name[1] != ':'))
      name = File::getBHDir() + std::string("/Config/") + name;

    const size_t lastDirectorySeparatorIndex2 = static_cast<int>(name.find_last_of("\\/"));
    QDir().mkpath(name.substr(0, lastDirectorySeparatorIndex2).c_str());

    return name;
  }

  enum ExportMode
  {
    raw,
    rgb,
    grayscale,
  };

  /**
   * Save an image to a file.
   * @param image The image to save.
   * @param fileName The intended file name of the image. Its extension determines the file format (e.g. .jpg, .png, ...).
   * @param imageNumber A number that will be integrated into the file name. -1: ignore.
   * @param mode Keep Raw ? Convert to RGB? Convert to Gray?
   * @return Was writing successful?
   */
  bool exportImage(QIODevice& outDevice, const ExportMode mode = raw) const
  {
    QImage::Format format = (mode == grayscale) ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
    int numChannels = (mode == grayscale) ? 1 : 3;

    QImage img(width * Pixel::numPixel(), height, format);
    for(unsigned y = 0; y < height; ++y)
    {
      const Pixel* pSrc = this->operator[](y);
      unsigned char* p = img.scanLine(y);
      const unsigned char* pEnd = p + numChannels * img.width();
      while(p != pEnd)
      {
        std::vector<unsigned char>pixel;
        switch(mode)
        {
          case rgb:
            pixel = pSrc->rgb();
            break;
          case grayscale:
            pixel = pSrc->grayscale();
            break;
          case raw:
            pixel = pSrc->raw();
            break;
        }
        for(unsigned char& pix : pixel)
          *p++ = pix;
        ++pSrc;
      }
    }
    return img.save(&outDevice, "PNG");
  }

  /**
   * Save an image to a file.
   * @param image The image to save.
   * @param fileName The intended file name of the image. Its extension determines the file format (e.g. .jpg, .png, ...).
   * @param imageNumber A number that will be integrated into the file name. -1: ignore.
   * @param mode Keep Raw ? Convert to RGB? Convert to Gray?
   * @return Was writing successful?
   */
  bool exportImage(const std::string& fileName, const int imageNumber = -1, const ExportMode mode = raw) const
  {
    QFile file(expandImageFileName(fileName, imageNumber).c_str());
    file.open(QIODevice::WriteOnly);

    // QFile is closed upon destruction, so close() is not necessary here
    return exportImage(file, mode);
  }

  /**
   * Save part of an image to a file.
   * @param image The image to save.
   * @param fileName The intended file name of the image. Its extension determines the file format (e.g. .jpg, .png, ...).
   * @param left The x coordinate of the leftmost pixel to be exported (inclusive).
   * @param top The y coordinate of the topmost pixel to be exported (inclusive).
   * @param right The x coordinate of the rightmost pixel to be exported (exclusive).
   * @param bottom The y coordinate of the lowest pixel to be exported (exclusive).
   * @param imageNumber A number that will be integrated into the file name. -1: ignore.
   * @param mode Keep Raw ? Convert to RGB? Convert to Gray?
   * @return Was writing successful?
   */
  bool exportImage(const std::string& fileName, const int left, const int top, const int right, const int bottom,
                   const int imageNumber = -1, const ExportMode mode = raw) const
  {
    const std::string name = expandImageFileName(fileName, imageNumber);
    QImage::Format format = (mode == grayscale) ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
    int numChannels = (mode == grayscale) ? 1 : 3;

    QImage img((right - left), bottom - top, format);
    img.fill(0);
    for(int y = std::max(top, 0); y < static_cast<int>(height) && y < bottom; ++y)
    {
      const Pixel* pSrc = this->operator[](y) + std::max(left, 0) / Pixel::numPixel();
      size_t xStart = std::max(left, 0) % Pixel::numPixel() * numChannels;
      unsigned char* p = img.scanLine(y - top) - std::min(left, 0);
      const unsigned char* pEnd = img.scanLine(y - top) + numChannels * std::min(img.width(), static_cast<int>(width * Pixel::numPixel() - left));
      while(p < pEnd)
      {
        std::vector<unsigned char> pixel;
        switch(mode)
        {
          case rgb:
            pixel = pSrc->rgb();
            break;
          case grayscale:
            pixel = pSrc->grayscale();
            break;
          case raw:
            pixel = pSrc->raw();
            break;
        }
        for(auto pix = pixel.begin() + xStart; p < pEnd && pix != pixel.end(); ++pix)
          *p++ = *pix;
        ++pSrc;
        xStart = 0;
      }
    }
    return img.save(name.c_str());
  }
#endif

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
  inline const PixelType& operator()(const size_t x, const size_t y) const { return *(image + (y * width + x)); }

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
  void serialize(In* in, Out* out) override
  {
    PUBLISH(reg);
    STREAM(width);
    STREAM(height);

    if(out)
      out->write(image, width * height * sizeof(Pixel));
    else
    {
      setResolution(width, height, allocator.empty() ? 0 : static_cast<unsigned>(reinterpret_cast<unsigned char*>(image) - allocator.data()));
      in->read(image, width * height * sizeof(Pixel));
    }
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

#ifndef TARGET_ROBOT
template<> inline bool Image<PixelTypes::GrayscaledPixel>::exportImage(const std::string& fileName, const int imageNumber, const ExportMode mode) const
{
  const std::string name = expandImageFileName(fileName, imageNumber);
  QImage::Format format = (mode == grayscale) ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
  int numChannels = (mode == grayscale) ? 1 : 3;

  QImage img(width, height, format);
  for(unsigned y = 0; y < height; ++y)
  {
    const unsigned char* pSrc = this->operator[](y);
    unsigned char* p = img.scanLine(y);
    const unsigned char* pEnd = p + numChannels * img.width();
    while(p != pEnd)
    {
      switch(mode)
      {
        case grayscale:
          *p++ = *pSrc;
          break;
        case rgb:
        case raw:
          *p++ = *pSrc;
          *p++ = *pSrc;
          *p++ = *pSrc;
          break;
      }
      ++pSrc;
    }
  }
  return img.save(name.c_str());
}
#endif

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

  void setResolution(const unsigned int width, const unsigned int height, const unsigned int padding = 0) override
  {
    FAIL("ImageWrapper does not manage its imagestorage itself.");
  }
};
