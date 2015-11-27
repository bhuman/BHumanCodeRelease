/**
 * @file   Sobel.h
 *
 * A class with methods to perform sobel edge detection on images (optimized by SSE instructions).
 * The introduced classes for images reserve 16byte aligned memory for the actual images, due to the usage of SSE.
 *
 * @author Alexis Tsogias
 */

#pragma once

#include "Representations/Infrastructure/Image.h"

namespace Sobel
{
  /**
   * An enum for the three channels of an image.
   */
  enum class Channel
  {
    Y = 0,
    Cb = 1,
    Cr = 2
  };

  /**
   * A class for an image with a single color channel.
   */
  class Image1D
  {
  public:
    typedef unsigned char Pixel;  /**< Pixels are unsigned chars. */
    const int maxWidth;           /**< The maximum width. */
    const int maxHeight;          /**< The maximum height. */
    int width;                    /**< The current width. */
    int height;                   /**< The current height. */
    int yStart;                   /**< The y coordinate the image starts from. Parts above yStart are left unchanged!*/
  private:
    static const int padding = 16;       /**< Some padding needed for the SSE instructions. */
    Pixel* imagePadding;          /**< A pointer to the memory for the image including some padding at the front and back. */
    Pixel* image;                 /**< A pointer to the actual image. */

  public:
    /**
     * Constructor. The memory for the underlying image is allocated 16byte aligned.
     * The allocated memory depends on maxWidth and maxHeight.
     *
     * @param maxWidth The maximum width of the image.
     * @param maxHeight The maximum height of the image.
     */
    Image1D(int maxWidth, int maxHeight);

    /**
     * Copy constructor.
     */
    Image1D(const Image1D& other);

    /**
     * Destructor. This will free the underlying image.
     */
    ~Image1D();

    /**
     * Sets the new resolution of the image. The resolution must be within the
     * maximum resolution.
     *
     * @param width The new width.
     * @param height The new height.
     */
    void setResolution(int width, int height);

    /**
     * Assignment operator.
     */
    Image1D& operator=(const Image1D& other);

    /**
     * Get a pointer to the yth row. No boundary check!
     * Use [y][x] to get the Pixel with the coordinates x,y.
     * The used resolutino is given by width and height!
     *
     * @param y The row to get.
     * @return A Pixel pointer of the yth row.
     */
    inline Pixel* operator[](const int y)
    {
      return image + y * width;
    }

    /**
     * Get a const pointer to the yth row. No boundary check!
     * Use [y][x] to get the Pixel with the coordinates x,y.
     * The used resolutino is given by width and height!
     *
     * @param y The row to get.
     * @return A const Pixel pointer of the yth row.
     */
    inline const Pixel* operator[](const int y) const
    {
      return image + y * width;
    }

    /**
     * Get the Pixel at x,y. No boundary check!
     * The current resolutino is given by width and height!
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The requested Pixel.
     */
    inline Pixel& operator()(int x, int y)
    {
      return *(image + y * width + x);
    }

    /**
     * Get the const Pixel at x,y. No boundary check!
     * The current resolutino is given by width and height!
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The requested const Pixel.
     */
    inline const Pixel& operator()(int x, int y) const
    {
      return *(image + y * width + x);
    }
  };

  /**
   * Sobel pixels consist of signed chars for the two directions x and y.
   */
  union SobelPixel
  {
    unsigned short index; /**< Used as internal table index */
    struct
    {
      char x, y;
    };
  };

  /**
   * A class for sobel images with a single color channel.
   */
  class SobelImage
  {
  public:
    const int maxWidth;   /**< The maximum width. */
    const int maxHeight;  /**< The maximum height. */
    int width;            /**< The current width. */
    int height;           /**< The current height. */
    int yStart;           /**< The y coordinate the image starts from. Parts above yStart are left unchanged!*/
  private:
    SobelPixel* sImage;   /**< A pointer to the actual sobel image. */

  public:
    /**
     * Constructor. The memory for the underlying image is allocated 16byte aligned.
     * The allocated memory depends on maxWidth and maxHeight.
     *
     * @param maxWidth The maximum width of the image.
     * @param maxHeight The maximum height of the image.
     */
    SobelImage(int maxWidth, int maxHeight);

    /**
     * Copy constructor.
     */
    SobelImage(const SobelImage& other);

    /**
     * Destructor. This will free the underlying image.
     */
    ~SobelImage();

    /**
     * Sets the new resolution of the image. The resolution must be within the
     * maximum resolution.
     *
     * @param width The new width.
     * @param height The new height.
     */
    void setResolution(int width, int height);

    /**
     * Assignment operator.
     */
    SobelImage& operator=(const SobelImage& other);

    /**
     * Get a pointer to the yth row. No boundary check!
     * Use [y][x] to get the Pixel with the coordinates x,y.
     *
     * @param y The row to get.
     * @return A SobelPixel pointer of the yth row.
     */
    inline SobelPixel* operator[](const int y)
    {
      return sImage + (y * width);
    }

    /**
     * Get a const pointer to the yth row. No boundary check!
     * Use [y][x] to get the Pixel with the coordinates x,y.
     *
     * @param y The row to get.
     * @return A const SobelPixel pointer of the yth row.
     */
    inline const SobelPixel* operator[](const int y) const
    {
      return sImage + (y * width);
    }

    /**
     * Get the SobelPixel at x,y. No boundary check!
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The requested SobelPixel.
     */
    inline SobelPixel& operator()(int x, int y)
    {
      return *(sImage + (y * width + x));
    }

    inline const SobelPixel* pixelAt(int x, int y) const
    {
      return sImage + (y * width + x);
    }

    /**
     * Get the const SobelPixel at x,y. No boundary check!
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The const requested SobelPixel.
     */
    inline const SobelPixel& operator()(int x, int y) const
    {
      return *(sImage + (y * width + x));
    }
  };

  /**
   * Extraction of a channel from an Image to an Image1D. The extraction starts at yStart.
   * Optimized with SSE instructions.
   *
   * @param srcImage The source image.
   * @param destImage The destination image.
   * @param channel The channel to extract.
   * @param yStart The y coordinate from where to start the extraction.
   */
  void extractSingleChannelSSE(const Image& srcImage, Sobel::Image1D& destImage, Channel channel, int yStart);

  /**
   * Sobel edge detection algorithm optimized with SSE. The algorithm starts at srcImage.yStart.
   *
   * @param srcImage The source image.
   * @param destImage The destination image.
   */
  void sobelSSE(const Image1D& srcImage, SobelImage& destImage);
};
