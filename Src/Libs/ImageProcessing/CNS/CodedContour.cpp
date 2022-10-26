#include "CodedContour.h"
#include "CNSSSE.h"
#include <algorithm>
#include <stdexcept>

using namespace std;

void CodedContour::evaluateX16Y16(signed short responseBin[16][16], const Image<CNSResponse>& img, int x, int y) const
{
  responseX16Y16RUsingSSE3(&img(x + referenceX, y + referenceY), img.width * sizeof(CNSResponse), &responseBin[0][0], *this);
}

void CodedContour::evaluateX8Y8(signed short responseBin[8][8], const Image<CNSResponse>& img, int x, int y) const
{
  responseX8Y8RUsingSSE3(&img(x + referenceX, y + referenceY), img.width * sizeof(CNSResponse), &responseBin[0][0], *this);
}

CodedContour CodedContour::circle(int r)
{
  int dMin = (2 * r - 1) * (2 * r - 1);
  int dMax = (2 * r + 1) * (2 * r + 1);
  int n = 0; // count pixels
  for(int y = -r; y <= +r; y++)
    for(int x = -r; x <= +r; x++)
    {
      int d = 4 * (x * x + y * y);
      if(dMin <= d && d < dMax)
        n++;
    }

  CodedContour contour;
  contour.reserve(n);
  for(int y = -r; y <= +r; y++)
    for(int x = -r; x <= +r; x++)
    {
      int d = 4 * (x * x + y * y);
      if(dMin <= d && d < dMax) // The point belongs to the circle
        contour.push_back(codeContourPointWithAngle(x, y, atan2(static_cast<double>(y), static_cast<double>(x)), n));
    }
  return contour;
}

// Lexical by ascending Y (primary) X (secondary)
class LessOnContourPointYX
{
public:
  bool operator()(const ContourPoint& a, const ContourPoint& b)
  { return a.y < b.y || (a.y == b.y && (a.x < b.x));  }
};

double normalizedPI(double angle)
{
  angle = fmod(angle, static_cast<double>(M_PI));
  if(angle < -M_PI / 2)
    return angle + M_PI;
  else if(angle > M_PI / 2)
    return angle - M_PI;
  else
    return angle;
}

void Contour::postProcess(int xLo, int xHi, int yLo, int yHi, int xRef, int yRef, bool doAutoShift)
{
  sort(begin(), end(), LessOnContourPointYX());
  // Remove point outside [xLo..xHi]*[yLo..yHi] and remove/join duplicates
  int j = 0;
  bool deleteJ = false;
  for(int i = 0; i < static_cast<int>(size()); i++)
  {
    const ContourPoint& cp = (*this)[i];
    if(xLo <= cp.x + referenceX && cp.x + referenceX <= xHi && yLo <= cp.y + referenceY && cp.y + referenceY <= yHi) // inside range
    {
      if(j == 0 || cp.x != (*this)[j - 1].x || cp.y != (*this)[j - 1].y) // store in output
      {
        if(deleteJ)
        {
          j--;
          deleteJ = false;
        }
        (*this)[j] = cp;
        j++;
      }
      else //same point twice
      {
        double dAngle = normalizedPI((*this)[j - 1].angle - cp.angle);
        if(fabs(dAngle) < M_PI / 4)
          (*this)[j].angle += dAngle / 2; // join points
        else
          deleteJ = true; // delete both (i directly, remember to delete j)
      }
    }
  }
  if(deleteJ)
    j--;
  resize(j);
  referenceX = xRef;
  referenceY = yRef;
  for(int i = 0; i < static_cast<int>(size()); i++)
  {
    (*this)[i].x -= xRef;
    (*this)[i].y -= yRef;
  }
  if(doAutoShift)
    autoShift();
}

void Contour::autoShift()
{
  if(empty())
    return;
  int xLo, xHi, yLo, yHi;
  computeBoundingBox(xLo, xHi, yLo, yHi);
  if(xHi - xLo >= 256 && yHi - yLo >= 256)
    throw std::range_error("Can not code contour with the available 8 bit coordinates.");
  if(xLo - referenceX < -127 || xHi - referenceX > 127 || yLo - referenceY < -127 || yHi - referenceY > 127) // Move reference point to middle
  {
    int dX = (xLo + xHi) / 2 - referenceX, dY = (yLo + yHi) / 2 - referenceY;
    referenceX += dX;
    referenceY += dY;
    for(int i = 0; i < static_cast<int>(size()); i++)
    {
      (*this)[i].x -= dX;
      (*this)[i].y -= dY;
    }
  }
  computeBoundingBox(xLo, xHi, yLo, yHi);
  assert(xLo - referenceX >= -127 && xHi - referenceX <= 127 && yLo - referenceY >= -127 && yLo - referenceY <= 127);
}

void lineY(Contour& contour, double x0, double x1, double a, double y0, double angle)
{
  assert(x0 <= x1);
  int x0i = static_cast<int>(x0), x1i = static_cast<int>(x1);
  double y = y0 + (x0i - x0) * a;
  for(int x = x0i; x <= x1i; x++)
  {
    contour.push_back(ContourPoint(x, static_cast<int>(round(y)), angle));
    y += a;
  }
}

void lineX(Contour& contour, double y0, double y1, double a, double x0, double angle)
{
  assert(y0 <= y1);
  int y0i = static_cast<int>(y0), y1i = static_cast<int>(y1);
  double x = x0 + (y0i - y0) * a;
  for(int y = y0i; y <= y1i; y++)
  {
    contour.push_back(ContourPoint(static_cast<int>(round(x)), y, angle));
    x += a;
  }
}

void line(Contour& contour, double x0, double y0, double x1, double y1, bool sparse)
{
  if(sparse)
  {
    double angle = atan2(y1 - y0, x1 - x0) + M_PI / 2;
    contour.push_back(ContourPoint(static_cast<int>(round((x0 + x1) / 2)),
                                   static_cast<int>(round((y0 + y1) / 2)), angle));
  }
  else
    line(contour, x0, y0, x1, y1);
}

void line(Contour& contour, double x0, double y0, double x1, double y1)
{
  double dX = x1 - x0, dY = y1 - y0;
  double angle = atan2(dY, dX) + M_PI / 2;
  if(fabs(dX) > fabs(dY))
  {
    if(dX > 0)
      lineY(contour, x0, x1, dY / dX, y0, angle);
    else
      lineY(contour, x1, x0, dY / dX, y1, angle);
  }
  else
  {
    if(dY > 0)
      lineX(contour, y0, y1, dX / dY, x0, angle);
    else
      lineX(contour, y1, y0, dX / dY, x1, angle);
  }
}

void Contour::computeBoundingBox(int& xLo, int& xHi, int& yLo, int& yHi) const
{
  xLo = yLo = numeric_limits<int>::max();
  xHi = yHi = numeric_limits<int>::min();
  if(!empty())
  {
    for(int i = 0; i < static_cast<int>(size()); i++)
    {
      xLo = min(xLo, (*this)[i].x);
      yLo = min(yLo, (*this)[i].y);
      xHi = max(xHi, (*this)[i].x);
      yHi = max(yHi, (*this)[i].y);
    }
    xLo += referenceX;
    xHi += referenceX;
    yLo += referenceY;
    yHi += referenceY;
  }
}
