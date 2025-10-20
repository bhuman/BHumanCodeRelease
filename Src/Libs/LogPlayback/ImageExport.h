/**
 * @file ImageExport.h
 *
 * This file defines functions to export images as files.
 *
 * @author Bernd Poppinga
 */

#pragma once

#include "Platform/File.h"
#include "ImageProcessing/Image.h"
#include "ImageProcessing/PixelTypes.h"
#include <QImage>
#include <QDir>
#include <QFile>
#include <cstdio>
#include <string>

namespace ImageExport
{
  inline std::string expandImageFileName(const std::string& fileName, int imageNumber)
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
    if(!File::isAbsolute(name))
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
   * @param outDevice The I/O device to write to.
   * @param fileFormat The name of the target format ("PNG", "JPG", ...)
   * @param mode Keep Raw ? Convert to RGB? Convert to Gray?
   * @return Was writing successful?
   */
  template<typename Pixel>
  inline bool exportImage(const Image<Pixel>& image, QIODevice& outDevice, const char* fileFormat,
                          const ExportMode mode = raw)
  {
    QImage::Format format = (mode == grayscale) ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
    QImage img(image.width * Pixel::numPixel(), image.height, format);
    for(unsigned y = 0; y < image.height; ++y)
    {
      unsigned char* p = img.scanLine(y);
      for(const Pixel* pSrc = image[y], * pEnd = pSrc + image.width; pSrc != pEnd; ++pSrc)
        switch(mode)
        {
          case rgb:
            pSrc->rgb(p);
            break;
          case grayscale:
            pSrc->grayscale(p);
            break;
          case raw:
            pSrc->raw(p);
        }
    }
    return img.save(&outDevice, fileFormat);
  }

  /**
   * Save an image to a file.
   * @param image The image to save.
   * @param fileName The intended file name of the image. Its extension determines the file format (e.g. .jpg, .png, ...).
   * @param imageNumber A number that will be integrated into the file name. -1: ignore.
   * @param mode Keep Raw ? Convert to RGB? Convert to Gray?
   * @return Was writing successful?
   */
  template<typename Pixel>
  inline bool exportImage(const Image<Pixel>& image, const std::string& fileName, const int imageNumber = -1, const ExportMode mode = raw)
  {
    QFile file(expandImageFileName(fileName, imageNumber).c_str());
    file.open(QIODevice::WriteOnly);

    // QFile is closed upon destruction, so close() is not necessary here
    return exportImage(image, file, file.fileName().last(3).toUpper().toStdString().c_str(), mode);
  }

  /**
   * Save a grayscale image to a file.
   * @param image The image to save.
   * @param fileName The intended file name of the image. Its extension determines the file format (e.g. .jpg, .png, ...).
   * @param imageNumber A number that will be integrated into the file name. -1: ignore.
   * @param mode Keep Raw ? Convert to RGB? Convert to Gray?
   * @return Was writing successful?
   */
  template<>
  inline bool exportImage(const Image<PixelTypes::GrayscaledPixel>& image, const std::string& fileName, const int imageNumber, const ExportMode mode)
  {
    const std::string name = expandImageFileName(fileName, imageNumber);
    QImage::Format format = (mode == grayscale) ? QImage::Format_Grayscale8 : QImage::Format_RGB888;
    int numChannels = (mode == grayscale) ? 1 : 3;

    QImage img(image.width, image.height, format);
    for(unsigned y = 0; y < image.height; ++y)
    {
      const unsigned char* pSrc = image[y];
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
}
