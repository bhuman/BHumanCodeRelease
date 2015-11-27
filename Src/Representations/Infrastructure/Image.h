/**
 * @file Image.h
 *
 * Declaration of struct Image
 */

#pragma once

#include "Tools/Streams/Streamable.h"

/**
 * Platform independent definition of an image struct
 */
struct Image : public Streamable
{
public:
  /**
   * The union defines a pixel in YCbCr space.
   */
  union Pixel
  {
    unsigned color; /**< Representation as single machine word. */
    unsigned char channels[4]; /**< Representation as an array of channels. */
    struct
    {
      unsigned char yCbCrPadding; /**< Ignore. */
      unsigned char cb; /**< Cb channel. */
      unsigned char y; /**< Y channel. */
      unsigned char cr; /**< Cr channel. */
    };
    struct
    {
      unsigned char r; /**< R channel. */
      unsigned char g; /**< G channel. */
      unsigned char b; /**< B channel. */
      unsigned char rgbPadding; /**< Ignore. */
    };
    struct
    {
      unsigned char h; /**< H channel. */
      unsigned char s; /**< S channel. */
      unsigned char i; /**< I channel. */
      unsigned char hsiPadding; /**< Ignore. */
    };
  };

  static const int maxResolutionWidth = 640;
  static const int maxResolutionHeight = 480;

  int width; /**< The width of the image in pixel */
  int height; /**< The height of the image in pixel */
  int widthStep; /**< The Distance between the first pixels of subsequent lines. */
  unsigned timeStamp = 0; /**< The time stamp of this image. */
  bool isReference = false; /**< States whether this struct holds the image, or only a reference to an image stored elsewhere. */
  bool isFullSize = false; /**< States that the pixels x = [width ... widthStep] should be preserved. */

private:
  Pixel* image; /**< The image. Please note that the second half of each row must be ignored. */

public:
  /**
   * @param initialize Whether to initialize the image in gray or not
   */
  Image(bool initialize = true, int width = maxResolutionWidth, int height = maxResolutionHeight);

  /**
   * Copy constructor.
   * @param other The image this is copied from.
   */
  Image(const Image& other);

  /** destructs an image */
  virtual ~Image();

  /**
   * Assignment operator.
   * @param other The image this is copied from.
   * @return This image.
   */
  Image& operator=(const Image& other);

  Pixel* operator[](const int y) { return image + y * widthStep; }
  const Pixel* operator[](const int y) const { return image + y * widthStep; }

  Pixel getFullSizePixel(const int y, const int x) const
  {
    Image::Pixel p = *(image + y * width + x / 2);
    if(!(x & 1))
      p.y = p.yCbCrPadding;
    return p;
  }

  /**
   * The method sets an external image.
   * @param buffer The image buffer.
   */
  void setImage(unsigned char* buffer);
  void setImage(Pixel* image);

  /** 
   * Converts an YCbCr image into an RGB image.
   * @param ycbcrImage The given YCbCr image
   */
  void convertFromYCbCrToRGB(const Image& ycbcrImage);

  /**
   * Converts an RGB image into an YCbCr image.
   * @param rgbImage The given RGB image
   */
  void convertFromRGBToYCbCr(const Image& rgbImage);

  /**
   * Converts an YCbCr image into a HSI image.
   * @param ycrcbImage The given YCbCr image
   */
  void convertFromYCbCrToHSI(const Image& ycrcbImage);

  /**
   * Converts a HSI image into an YCbCr image.
   * @param hsiImage The given HSI image
   */
  void convertFromHSIToYCbCr(const Image& hsiImage);

  /**
   * Sets the new resolution of the image including the widthStep.
   */
  void setResolution(int newWidth, int newHeight, bool fullSize = false);

  /**
   * Calculates the distance between the first three bytes of two colors
   * @param a The first color
   * @param b The second color
   * @return The distance
   */
  static float getColorDistance(const Image::Pixel& a, const Image::Pixel& b);

private:
  void serialize(In* in, Out* out);
};
