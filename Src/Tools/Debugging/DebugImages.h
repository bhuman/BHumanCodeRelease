/**
 * @file Tools/Debugging/DebugImages.h
 *
 * Macros to manipulate and send debug images
 *
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 */

#pragma once
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Geometry.h"

/**
 * Declares a debug image
 * @param id An image id
 */
#define DECLARE_DEBUG_IMAGE(id) mutable Image id##Image

/**Gets the y, u and v values of the specified pixel in the specified debug image */
#define DEBUG_IMAGE_GET_PIXEL_Y(id, xx, yy) id##Image[yy][xx].y
#define DEBUG_IMAGE_GET_PIXEL_U(id, xx, yy) id##Image[yy][xx].cb
#define DEBUG_IMAGE_GET_PIXEL_V(id, xx, yy) id##Image[yy][xx].cr

/**Sets the Y, U, and V values of the specified pixel in the specified debug image */
#define DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, Y, U, V) \
  do \
    if(((int)(xx))+1 > 0 && ((int)(xx)) < id##Image.width && \
       ((int)(yy))+1 > 0 && ((int)(yy)) < id##Image.height) \
    { \
      id##Image[yy][xx].y = Y; \
      id##Image[yy][xx].cb = U; \
      id##Image[yy][xx].cr = V; \
    } \
  while(false)

/** Converts a RGB color and sets the Y, U, and V values of the specified pixel in the specified debug image */
#define DEBUG_IMAGE_SET_PIXEL_RGB(id, xx, yy, r, g, b) \
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, \
                            (unsigned char)((306 * int(b) + 601 * int(g) + 117 * int(r)) >> 10), \
                            (unsigned char)((130560 + 512 * int(b) - 429 * int(g) - 83 * int(r)) >> 10), \
                            (unsigned char)((130560 - 173 * int(b) - 339 * int(g) + 512 * int(r)) >> 10))

/**
 * Initializes a debug image with an image
 * @param id An image id
 * @param image The Image.
 */
#define INIT_DEBUG_IMAGE(id, image) \
  DECLARED_DEBUG_RESPONSE("debug images:" #id) id##Image = image

/**
 * Sets the width and height of a debug image
 * @param id An image id.
 * @param width width of the image.
 * @param height height of the image.
 */
#define SET_DEBUG_IMAGE_SIZE(id, rwidth, rheight) \
  id##Image.setResolution(rwidth, rheight); \

/**
 * Initializes a debug image with an image, setting all pixels black afterwards
 * @param id An image id
 */
#define INIT_DEBUG_IMAGE_BLACK(id, rwidth, rheight) \
  do \
    DECLARED_DEBUG_RESPONSE("debug images:" #id) \
    { \
      id##Image.setResolution(rwidth, rheight); \
      for(int y = 0; y < id##Image.height; y++) \
        for(int x = 0; x < id##Image.width; x++) \
          DEBUG_IMAGE_SET_PIXEL_BLACK(id, x, y); \
    } \
  while(false)

/**Sends the debug image with the specified id */
#define SEND_DEBUG_IMAGE(id) \
  do \
    DEBUG_RESPONSE("debug images:" #id) OUTPUT(idDebugImage, bin, #id << id##Image); \
  while(false)

/**Sends the debug image with the specified id as jpeg encoded image */
#define SEND_DEBUG_IMAGE_AS_JPEG(id) \
  do \
    DEBUG_RESPONSE("debug images:" #id) \
    { \
      JPEGImage* temp = new JPEGImage(id##Image); \
      OUTPUT(idDebugJPEGImage, bin, #id << *temp); \
      delete temp; \
    } \
  while(false)

/**Sets the Y, U, and V values of the specified pixel in the specified debug image */
#define DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, Y, U, V) \
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xy.x, xy.y, Y, U, V)

#define DEBUG_IMAGE_SET_PIXEL_BLACK(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 0, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_WHITE(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 255, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_GREEN(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 180, 0, 0)
#define DEBUG_IMAGE_SET_PIXEL_LIGHT_GRAY(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 192, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_GRAY(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 127, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_DARK_GRAY(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 64, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_DARK_GREEN(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 0, 0, 0)
#define DEBUG_IMAGE_SET_PIXEL_ORANGE(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 164, 0, 255)
#define DEBUG_IMAGE_SET_PIXEL_YELLOW(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 255, 0, 170)
#define DEBUG_IMAGE_SET_PIXEL_RED(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 0, 0, 255)
#define DEBUG_IMAGE_SET_PIXEL_BLUE(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 60, 255, 80)
#define DEBUG_IMAGE_SET_PIXEL_PINK(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 255, 255, 255)
#define DEBUG_IMAGE_SET_PIXEL_DARK_BLUE(id, xx, yy)\
  DEBUG_IMAGE_SET_PIXEL_YUV(id, xx, yy, 30, 255, 80)

/** and the same with an vector as input */
#define DEBUG_IMAGE_SET_PIXEL_BLACK_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 0, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_WHITE_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 255, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_GREEN_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 180, 0, 0)
#define DEBUG_IMAGE_SET_PIXEL_LIGHT_GRAY_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 192, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_GRAY_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 127, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_DARK_GRAY_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 64, 127, 127)
#define DEBUG_IMAGE_SET_PIXEL_DARK_GREEN_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 0, 0, 0)
#define DEBUG_IMAGE_SET_PIXEL_ORANGE_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 100, 255, 0)
#define DEBUG_IMAGE_SET_PIXEL_YELLOW_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 180, 255, 0)
#define DEBUG_IMAGE_SET_PIXEL_RED_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 0, 255, 0)
#define DEBUG_IMAGE_SET_PIXEL_MAUVE_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 0, 180, 255)
#define DEBUG_IMAGE_SET_PIXEL_BLUE_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 180, 0, 255)
#define DEBUG_IMAGE_SET_PIXEL_PINK_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 255, 255, 255)
#define DEBUG_IMAGE_SET_PIXEL_DARK_BLUE_AS_VECTOR(id, xy)\
  DEBUG_IMAGE_SET_PIXEL_YUV_AS_VECTOR(id, xy, 100, 0, 255)

// all
/** Generate debug image debug request, can be used for encapsulating the creation of debug images on request */
#define COMPLEX_IMAGE(id) \
  DEBUG_RESPONSE("debug images:" #id)

/**
 * Fill a triangle inside the specified debug image with the specified color
 * @param x1, y1, x2, y2, x3, y3 the three points of the triangle
 */
//FIXME think about inclusive vs. exclusive indices
#define DEBUG_IMAGE_FILL_TRIANGLE_RGB(id, x1, y1, x2, y2, x3, y3, r, g, b) \
  do \
    COMPLEX_IMAGE(id) \
    { \
      const int lowX = std::min(x1, std::min(x2, x3)); \
      const int highX = std::max(x1, std::max(x2, x3)); \
      const int lowY = std::min(y1, std::min(y2, y3)); \
      const int highY = std::max(y1, std::max(y2, y3)); \
      for(int x = lowX; x < highX; ++x) \
      { \
        for(int y = lowY; y < highY; ++y) \
        { \
          if(Geometry::isPointInsideTriangle(static_cast<float>(x1), static_cast<float>(y1), \
                                             static_cast<float>(x2), static_cast<float>(y2), \
                                             static_cast<float>(x3), static_cast<float>(y3), \
                                             static_cast<float>(x), static_cast<float>(y))) \
          { \
            DEBUG_IMAGE_SET_PIXEL_RGB(id, x, y, r, g, b); \
          } \
        } \
      } \
    } \
  while(false)

#define DEBUG_IMAGE_DRAW_LINE_RGB(id, x1, y1, x2, y2, r, g, b) \
  do \
    COMPLEX_IMAGE(id) \
    { \
      const Geometry::PixeledLine line(x1, y1, x2, y2); \
      for(const Vector2i& p : line) \
      { \
        DEBUG_IMAGE_SET_PIXEL_RGB(id, p.x(), p.y(), r, g, b); \
      } \
    } \
  while(false)

#define DEBUG_IMAGE_TRIANGLE_RGB(id, x1, y1, x2, y2, x3, y3, r, g, b) \
  do \
    COMPLEX_IMAGE(id) \
    { \
      DEBUG_IMAGE_DRAW_LINE_RGB(id, x1, y1, x2, y2, r, g, b); \
      DEBUG_IMAGE_DRAW_LINE_RGB(id, x2, y2, x3, y3, r, g, b); \
      DEBUG_IMAGE_DRAW_LINE_RGB(id, x3, y3, x1, y1, r, g, b); \
    } \
  while(false)
