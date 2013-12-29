/**
 * @file Image.h
 *
 * Declaration of class Image
 */

#pragma once

#include "Tools/Streams/Streamable.h"

const int maxResolutionWidth = 640;
const int maxResolutionHeight = 480;

/**
* Platform independend definition of an image class
*/
class Image : public Streamable
{
public:
  /**
   * The union defines a pixel in YCbCr space.
   */
  union Pixel
  {
    unsigned color; /**< Representation as single machine word. */
    unsigned char channels[4];  /**< Representation as an array of channels. */
    struct
    {
      unsigned char yCbCrPadding, /**< Ignore. */
      cb, /**< Cb channel. */
      y, /**< Y channel. */
      cr; /**< Cr channel. */
    };
    struct
    {
      unsigned char r, /**< R channel. */
      g, /**< G channel. */
      b, /**< B channel. */
      rgbPadding; /**< Ignore. */
    };
    struct
    {
      unsigned char h, /**< H channel. */
      s, /**< S channel. */
      i, /**< I channel. */
      hsiPadding; /**< Ignore. */
    };
  };

  /**
  * Default constructor.
  * @param initialize Whether to initialize the image in gray or not
  */
  Image(bool initialize = true, int width = maxResolutionWidth, int height = maxResolutionHeight);

  /** destructs an image */
  ~Image();

  /**
  * Copy constructor.
  * @param other The image this is copied from.
  */
  Image(const Image& other);

  /**
  * Assignment operator.
  * @param other The image this is copied from.
  * @return This image.
  */
  Image& operator=(const Image& other);

  inline Pixel* operator[](const int y)
  {
    return image + y * widthStep;
  }

  inline const Pixel* operator[](const int y) const
  {
    return image + y * widthStep;
  }

  /**
  * The method sets an external image.
  * @param buffer The image buffer.
  */
  void setImage(const unsigned char* buffer);

  /** Converts an YCbCr image into an RGB image.
  *  @param ycbcrImage The given YCbCr image
  */
  void convertFromYCbCrToRGB(const Image& ycbcrImage);

  /** Converts an RGB image into an YCbCr image.
  *  @param rgbImage The given RGB image
  */
  void convertFromRGBToYCbCr(const Image& rgbImage);

  /** Converts an YCbCr image into a HSI image.
  *  @param ycrcbImage The given YCbCr image
  */
  void convertFromYCbCrToHSI(const Image& ycrcbImage);

  /** Converts a HSI image into an YCbCr image.
  *  @param hsiImage The given HSI image
  */
  void convertFromHSIToYCbCr(const Image& hsiImage);

  unsigned timeStamp; /**< The time stamp of this image. */
  bool isReference; /**< States whether this class holds the image, or only a reference to an image stored elsewhere. */
  int width; /**< The width of the image in pixel */
  int height; /**< The height of the image in pixel */
  int widthStep; /**< The Distance between the first pixels of subsequent lines. */

  /**
   * Sets the new resolution of the image including the widthStep.
   */
  inline void setResolution(int newWidth, int newHeight)
  {
    width = newWidth;
    height = newHeight;
    widthStep = width * 2;
  }

  /**
   * Calculates the distance between the first three bytes of two colors
   * @param a The first color
   * @param b The second color
   * @return The distance
   */
  static float getColorDistance(const Image::Pixel& a, const Image::Pixel& b);

private:
  void serialize(In* in, Out* out);

  Pixel* image; /**< The image. Please note that the second half of each row must be ignored. */
};

class ImageOther : public Image {};
