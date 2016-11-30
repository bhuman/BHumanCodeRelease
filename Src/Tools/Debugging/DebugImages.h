/**
 * @file Tools/Debugging/DebugImages.h
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/ImageProcessing/TImage.h"

struct DebugImage : public Streamable
{
private:
  void* data;
  size_t maxSize = 0;

public:
  unsigned int timeStamp;
  unsigned short width;
  unsigned short height;
  bool isReference;
  PixelTypes::PixelType type;

  DebugImage() : data(nullptr), isReference(false) {}
  DebugImage(const Image& image, const bool copy = false)
    : data(copy ? Memory::alignedMalloc(image.width * image.height * 2 * sizeof(PixelTypes::YUYVPixel), 32) : const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(image.timeStamp), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height * 2)), isReference(!copy), type(PixelTypes::PixelType::YUYV)
  {
    if(copy)
    {
      maxSize = width * height * sizeof(PixelTypes::YUYVPixel);
      memcpy(data, image[0], maxSize);
    }
  }
  DebugImage(const TImage<PixelTypes::RGBPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::RGB) {}
  DebugImage(const TImage<PixelTypes::BGRAPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::BGRA) {}
  DebugImage(const TImage<PixelTypes::YUYVPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::YUYV) {}
  DebugImage(const TImage<PixelTypes::YUVPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::YUV) {}
  DebugImage(const TImage<PixelTypes::GrayscaledPixel>& image, const bool copy = false)
    : data(copy ? Memory::alignedMalloc(image.width * image.height * sizeof(PixelTypes::GrayscaledPixel), 32) : const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(!copy), type(PixelTypes::PixelType::Grayscale)
  {
    if(copy)
    {
      maxSize = width * height * sizeof(PixelTypes::GrayscaledPixel);
      memcpy(data, image[0], maxSize);
    }
  }
  DebugImage(const TImage<PixelTypes::ColoredPixel>& image)
    : data(const_cast<void*>(static_cast<const void*>(image[0]))), timeStamp(0), width(static_cast<unsigned short>(image.width)), height(static_cast<unsigned short>(image.height)), isReference(true), type(PixelTypes::PixelType::Colored) {}

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

  void from(const Image& image)
  {
    size_t size = image.width * image.height * 2 * sizeof(PixelTypes::YUYVPixel);
    if(isReference || !data || size > maxSize)
    {
      if(!isReference && data)
        Memory::alignedFree(data);
      isReference = false;
      data = Memory::alignedMalloc(size, 32);
      maxSize = size;
    }
    timeStamp = image.timeStamp;
    width = static_cast<unsigned short>(image.width);
    height = static_cast<unsigned short>(image.height * 2);
    type = PixelTypes::PixelType::YUYV;
    memcpy(data, image[0], size);
  }

  void from(const TImage<PixelTypes::GrayscaledPixel>& image)
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

  void toImage(Image& image) const
  {
    image.setResolution(width, height / 2, true);
    memcpy(image[0], data, width * height * PixelTypes::pixelSize(type));
  }

  unsigned short getImageWidth() const
  {
    return type == PixelTypes::YUYV ? width * 2 : width;
  }

  void convertToBGRA(void* dest) const;

protected:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(type, PixelTypes);
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

    STREAM_REGISTER_FINISH;
  }
};

/**Sends the debug image with the specified id */
#define SEND_DEBUG_IMAGE(id, image) \
  do \
    DEBUG_RESPONSE("debug images:" id) OUTPUT(idDebugImage, bin, id << DebugImage(image)); \
  while(false)

// all
/** Generate debug image debug request, can be used for encapsulating the creation of debug images on request */
#define COMPLEX_IMAGE(id) \
  DEBUG_RESPONSE("debug images:" id)
