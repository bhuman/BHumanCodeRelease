/**
 * @file Tools/Debugging/DebugImages.h
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraImage.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/ImageProcessing/Image.h"
#include "Platform/Memory.h"
#include <type_traits>

struct DebugImage : public Streamable
{
private:
  size_t maxSize = 0;

public:
  void* data;
  unsigned int timestamp;
  unsigned short width;
  unsigned short height;
  bool isReference;
  PixelTypes::PixelType type;

  DebugImage() : data(nullptr), isReference(false) {}
  DebugImage(const Image<PixelTypes::RGBPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::RGB) {}
  DebugImage(const Image<PixelTypes::BGRAPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::BGRA) {}
  DebugImage(const CameraImage& cameraImage, const bool copy = false)
    : DebugImage(static_cast<const Image<PixelTypes::YUYVPixel>&>(cameraImage), copy)
  {
    timestamp = cameraImage.timestamp;
  }
  DebugImage(const Image<PixelTypes::YUYVPixel>& image, const bool copy = false)
    : data(copy ? Memory::alignedMalloc(image.width * image.height * sizeof(PixelTypes::YUYVPixel), 32) : const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(!copy), type(PixelTypes::PixelType::YUYV)
  {
    if(copy)
    {
      maxSize = width * height * sizeof(PixelTypes::YUYVPixel);
      memcpy(data, image[0], maxSize);
    }
  }
  DebugImage(const Image<PixelTypes::YUVPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::YUV) {}
  DebugImage(const Image<PixelTypes::GrayscaledPixel>& image, const bool copy = false)
    : data(copy ? Memory::alignedMalloc(image.width * image.height * sizeof(PixelTypes::GrayscaledPixel), 32) : const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(!copy), type(PixelTypes::PixelType::Grayscale)
  {
    if(copy)
    {
      maxSize = width * height * sizeof(PixelTypes::GrayscaledPixel);
      memcpy(data, image[0], maxSize);
    }
  }
  DebugImage(const Image<PixelTypes::ColoredPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::Colored) {}
  DebugImage(const Image<PixelTypes::HuePixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::Hue) {}
  DebugImage(const Image<PixelTypes::BinaryPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::Binary) {}
  template<typename SomeEdge2Pixel>
  DebugImage(const Image<SomeEdge2Pixel>& image, const PixelTypes::PixelType drawAs)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timestamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(drawAs)
  {
    static_assert(sizeof(SomeEdge2Pixel) == PixelTypes::pixelSize(PixelTypes::Edge2), "");
    static_assert(std::is_base_of<PixelTypes::Edge2Pixel, SomeEdge2Pixel>::value, "");
    ASSERT(drawAs == PixelTypes::Edge2 || drawAs == PixelTypes::Edge2MonoAbsAvg || drawAs == PixelTypes::Edge2MonoAvg);
  }

  ~DebugImage()
  {
    if(!isReference && data)
    {
      Memory::alignedFree(data);
      data = nullptr;
    }
  }

  template<typename T> class DebugImageView
  {
  private:
    const DebugImage& image;
  public:
    DebugImageView(const DebugImage& image) : image(image) {}
    const T* operator[](const size_t y) const { return reinterpret_cast<T*>(image.data) + y * image.width; }
  };

  template<typename T> DebugImageView<T> getView() const
  {
    return DebugImageView<T>(*this);
  }

  void from(const CameraImage& cameraImage)
  {
    from(static_cast<const Image<PixelTypes::YUYVPixel>&>(cameraImage));
    timestamp = cameraImage.timestamp;
  }

  void from(const Image<PixelTypes::YUYVPixel>& image)
  {
    size_t size = image.width * image.height * sizeof(PixelTypes::YUYVPixel);
    if(isReference || !data || size > maxSize)
    {
      if(!isReference && data)
        Memory::alignedFree(data);
      isReference = false;
      data = Memory::alignedMalloc(size, 32);
      maxSize = size;
    }
    width = static_cast<unsigned short>(image.width);
    height = static_cast<unsigned short>(image.height);
    type = PixelTypes::PixelType::YUYV;
    memcpy(data, image[0], size);
  }

  void from(const Image<PixelTypes::GrayscaledPixel>& image)
  {
    size_t size = image.width * image.height * sizeof(PixelTypes::GrayscaledPixel);
    if(isReference || !data || size > maxSize)
    {
      if(!isReference && data)
        Memory::alignedFree(data);
      isReference = false;
      data = Memory::alignedMalloc(size, 32);
      maxSize = size;
    }
    width = static_cast<unsigned short>(image.width);
    height = static_cast<unsigned short>(image.height);
    type = PixelTypes::PixelType::Grayscale;
    memcpy(data, image[0], size);
  }

  unsigned short getImageWidth() const
  {
    return type == PixelTypes::YUYV ? width * 2 : width;
  }

protected:
  void serialize(In* in, Out* out)
  {
    STREAM(type);
    STREAM(width);
    STREAM(height);

    const size_t size = width * height * PixelTypes::pixelSize(type);
    if(out)
      out->write(data, size);
    else
    {
      if(isReference || !data || size > maxSize)
      {
        if(!isReference && data)
          Memory::alignedFree(data);
        isReference = false;
        data = Memory::alignedMalloc(size, 32);
        maxSize = size;
      }
      in->read(data, size);
    }
  }

private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(DebugImage);
    REG(type);
    REG(width);
    REG(height);
  }
};

class DebugImageConverter
{
private:
  using ConversionFunction = void (*)(unsigned int, const void*, void*);
  ConversionFunction converters[PixelTypes::numOfPixelTypes];

public:
  DebugImageConverter();
  ~DebugImageConverter();
  void convertToBGRA(const DebugImage& src, void* dest);
};

/**
 * Sends the debug image.
 * @param id The name under which it is sent.
 * @param image The image that is sent.
 * @param method Optionally, a pixel type that defines the drawing method to be used.
 */
#define SEND_DEBUG_IMAGE(id, ...) \
  do \
    _SEND_DEBUG_IMAGE_EXPAND(_SEND_DEBUG_IMAGE_EXPAND(_SEND_DEBUG_IMAGE_THIRD(__VA_ARGS__, _SEND_DEBUG_IMAGE_WITH_METHOD, _SEND_DEBUG_IMAGE_WITHOUT_METHOD))(id, __VA_ARGS__)) \
    while(false)
#define _SEND_DEBUG_IMAGE_WITHOUT_METHOD(id, image) DEBUG_RESPONSE("debug images:" id) OUTPUT(idDebugImage, bin, id << DebugImage(image));
#define _SEND_DEBUG_IMAGE_WITH_METHOD(id, image, method) DEBUG_RESPONSE("debug images:" id) OUTPUT(idDebugImage, bin, id << DebugImage(image, method));

#define _SEND_DEBUG_IMAGE_THIRD(first, second, third, ...) third
#define _SEND_DEBUG_IMAGE_EXPAND(s) s // needed for Visual Studio

// all
/** Generate debug image debug request, can be used for encapsulating the creation of debug images on request */
#define COMPLEX_IMAGE(id) \
  DEBUG_RESPONSE("debug images:" id)
