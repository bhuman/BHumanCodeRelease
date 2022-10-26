#pragma once

#include "ImageProcessing/CNS/CNSResponse.h"
#include "ImageProcessing/CNS/ResponseMapping.h"
#include "ImageProcessing/Image.h"

#ifdef WINDOWS
#define __restrict __restrict
#endif

//! A single point on a contour with normal direction
/*! A contour point has a position relative to the
    reference point and a normal direction (angle).
 */
class ContourPoint
{
public:
  //! Position relative to reference point
  int x, y;
  //! direction normal to contour
  double angle;

  ContourPoint() : x(0), y(0), angle(0) {}

  ContourPoint(int x, int y, double angle) : x(x), y(y), angle(angle) {}
};

//! A single point on a contour coded in a 32bit integer
/*! A contour point has a position relative to the
    reference point coded as x and y (int8_t),
    and a vector being normal to the contour coded
    in a way that it most directly can be used by
    the response computation routine.

    The length of the vector (nx,ny) is scaled
    by 1/sqrt(n) with n being the number of points,
    such that the intermediate accumulator does
    not overflow unit16.
 */
using CodedContourPoint = unsigned int;

//! Special contour point indicating "there is none" or invalid
enum {EMPTYCONTOURPOINT = 0xffffffff};

inline CodedContourPoint codeContourPoint(int x, int y, int nx, int ny)
{
  assert(-127 <= x  && x <= 127);
  assert(-127 <= y  && y <= 127);
  assert(-127 <= nx && nx <= 127);
  assert(-127 <= ny && ny <= 127);

  return (x & 0xff) + ((y & 0xff) << 8) + ((nx & 0xff) << 16) + ((ny & 0xff) << 24);
}

//! Determines by which factor to scale \c cos/sin of angle so nothing overflows
/*! n*(factor*CNSResponse::SCALE)^2/0x10000=0xffff is the largest response that can be accumulated
    factor is determined such that this does not overflow. It is asserted that
    factor itself is <128 so the scaled cos/sin fit in 7bit and that factor is >8
    so precision is still reasonable.
 */
inline double factorOfN(int n, bool doClip = false)
{
  double factor =  sqrt(static_cast<double>(0xffff0000) / n) / CNSResponse::SCALE;
  if(doClip)
  {
    if(!(factor >= 9))
      factor = 9; // with ! to clip NaN to 9
    if(!(factor <= 127))
      factor = 127;
  }
  assert(9 <= factor && factor <= 127); // 128 would be an overflow, 8 is the smallest value for reasonable precision
  return factor;
}

//! Codes a contour point at \c (x,y) relative to the reference point with normal angle \c alpha
/*! \c cos and \c sin of \c angle are scaled by \c factor and stored in 8bit signed integers.
 */
inline CodedContourPoint codeContourPointWithAngleAndFactor(int x, int y, double alpha, double factor = 127)
{
  assert(8 < factor && factor < 128); // 128 would be an overflow, 8 is the smallest value for reasonable precision
  double c = cos(alpha), s = sin(alpha);
  return codeContourPoint(x, y, int(factor * c), int(factor * s));
}

//! Codes a contour point at \c (x,y) relative to the reference point with normal angle \c alpha
/*! \c n is the number of points in the contour. This is needed to scale intermediate results
 */
inline CodedContourPoint codeContourPointWithAngle(int x, int y, double alpha, int n)
{return codeContourPointWithAngleAndFactor(x, y, alpha, factorOfN(n));}

//! codes a contour point
/*! \c n is the number of points in the contour. This is needed to scale intermediate results
 */
inline CodedContourPoint code(const ContourPoint& cp, int n) {return codeContourPointWithAngle(cp.x, cp.y, cp.angle, n);}

//! Returns the coded x position
inline int xOfCCP(CodedContourPoint cp) {return static_cast<signed char>(cp & 0xff);}

//! Returns the coded y position
inline int yOfCCP(CodedContourPoint cp) {return static_cast<signed char>((cp >> 8) & 0xff);}

//! Returns the coded nx normal vector component
inline int nxOfCCP(CodedContourPoint cp) {return static_cast<signed char>(((cp >> 16) & 0xff));}

//! Returns the coded ny normal vector component
inline int nyOfCCP(CodedContourPoint cp) {return static_cast<signed char>((cp >> 24));}

//! Returns the coded nx and ny as one 16 bit number
inline unsigned short nOfCCP(CodedContourPoint cp) {return static_cast<unsigned short>(cp >> 16);}

//! Returns the angle coded in nx and ny as a floating point number
inline double angleOfCCP(CodedContourPoint cp) {return atan2(static_cast<double>(nyOfCCP(cp)), static_cast<double>(nxOfCCP(cp)));}

//! decodes a single contour point
inline ContourPoint decode(const CodedContourPoint& ccp)
{
  return ContourPoint(xOfCCP(ccp), yOfCCP(ccp), angleOfCCP(ccp));
}

//! A contour is a set of contourpoints
/*! The points can be ordered arbitrarily and also consist of several subcurves.
    However, the code is more efficient if they are sorted vertically.

    Additional parameters define a linear scaling of the raw response.
    This allows to normalize the response with respect to different
    contours accounting for the fact that shorter contours are more likely
    to receive a large response just by coincidence (see \c responseMapping).
 */
class Contour: public std::vector<ContourPoint>
{
public:
  Contour()  : std::vector<ContourPoint>(), referenceX(0), referenceY(0), mapping()  {}

  Contour(int n) : std::vector<ContourPoint>(n), referenceX(0), referenceY(0), mapping()  {}

  //! The position of the reference point
  /*! There two ways of using a contour. One is to set the reference point here
      for contours which are only applied at/around a certain position in the image.
      The second is to leave them 0 and pass the reference point when using the
      contour.
   */
  int referenceX, referenceY;

  LinearResponseMapping mapping;

  //! post processes a contour to facilitate the detection process later
  /*! contour points are sorted by y (secondary x), duplicate points are either merge
    (if there angle difference is <45deg) or deleted (>45deg). Also the contour is
    clipped to [xLo..xHi]*[yLo..yHi], i.e. points outside are removed.

    If \c xRef,yRef is given \c .referenceX/Y is set and the coordinates shifted
    accordingly. If \c autoShift is given, the reference point is also shifted to
    map the range of x/y offsets to the available range of +/-127.
   */
  void postProcess(int xLo, int xHi, int yLo, int yHi, int xRef = 0, int yRef = 0, bool doAutoShift = true);

  //! Computes the min/max x/y coordinates with the reference point added
  void computeBoundingBox(int& xLo, int& xHi, int& yLo, int& yHi) const;

  //! shifts the reference point if necessary to have coordinates within +/-127
  /*! If this is not possible, an assertion is thrown. */
  void autoShift();
};

//! A contour is a set of contour points
/*! The points can be ordered arbitrarily and also consist of several subcurves.
    However, the code is more efficient if they are sorted vertically.

    Additional parameters define a linear scaling of the raw response.
    This allows to normalize the response with respect to different
    contours accounting for the fact that shorter contours are more likely
    to receive a large response just by coincidence (see \c responseMapping).
 */
class CodedContour: public std::vector<CodedContourPoint>
{
public:
  CodedContour()  : std::vector<CodedContourPoint>(), referenceX(0), referenceY(0), mapping()  {}

  CodedContour(int n) : std::vector<CodedContourPoint>(n), referenceX(0), referenceY(0), mapping()  {}

  int referenceX, referenceY;

  LinearResponseMapping mapping;

  //! Computes the responses for this contour in \c img with the reference point a x,y the 16x16 following
  /*! Precisely, responseBin[i][j] is the binary-response of the contour at \c x+i,y+j */
  void evaluateX16Y16(signed short responseBin[16][16], const Image<CNSResponse>& img, int x = 0, int y = 0) const;

  //! See \c evaluateX16Y16
  void evaluateX8Y8(signed short responseBin[8][8], const Image<CNSResponse>& img, int x = 0, int y = 0) const;

  //! Computes a circular contour of radius \c around \c 0
  static CodedContour circle(int r);
};

inline void code(CodedContour& cct, const Contour& contour)
{
  cct.resize(contour.size());
  for(size_t i = 0; i < contour.size(); i++)
    cct[i] = code(contour[i], static_cast<int>(contour.size()));
  cct.referenceX = contour.referenceX;
  cct.referenceY = contour.referenceY;
  cct.mapping = contour.mapping;
}

inline void decode(Contour& cct, const CodedContour& contour)
{
  cct.resize(contour.size());
  for(int i = 0; i < static_cast<int>(contour.size()); i++)
    cct[i] = decode(contour[i]);
  cct.mapping = contour.mapping;
  cct.referenceX = contour.referenceX;
  cct.referenceY = contour.referenceY;
}

//! Multiplies the normal vector of \c ccp by \c factor/256.0
inline void scaleNormalVector(CodedContourPoint& ccp, int factor)
{
  signed char* ccpX = reinterpret_cast<signed char*>(&ccp);
  ccpX[2] = static_cast<signed char>((static_cast<int>(ccpX[2] * factor + 128)) >> 8);
  ccpX[3] = static_cast<signed char>((static_cast<int>(ccpX[3] * factor + 128)) >> 8);
}

//! Rasterizes a line from \c (x0,y0) to \c (x1,y1) and adds it to \c contour
/*! The line has a light raster, i.e. there is only 1 pixel in each column/row.
    The whole line gets the same angle (no angle interpolation).
 */
void line(Contour& contour, double x0, double y0, double x1, double y1);

//! Adds a line (\c sparse=false) or just its middlepoint (\c sparse=true) to \c contour
void line(Contour& contour, double x0, double y0, double x1, double y1, bool sparse);
