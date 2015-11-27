/**
* @file Controller/Visualization/PaintMethods.cpp
* Implementation of class PaintMethods.
*
* @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
* @author Colin Graf
*/

#include <QPainter>
#include "PaintMethods.h"

QBrush PaintMethods::brush(Qt::SolidPattern);
QBrush PaintMethods::noBrush(Qt::NoBrush);
QPen PaintMethods::pen;
QPen PaintMethods::noPen(Qt::NoPen);

void PaintMethods::paintDebugDrawing(QPainter& painter, const DebugDrawing& debugDrawing, const QTransform& baseTrans)
{
  for(const DebugDrawing::Element* e = debugDrawing.getFirst(); e; e = debugDrawing.getNext(e))
    switch(e->type)
    {
      case DebugDrawing::Element::POLYGON:
      {
        paintPolygon(*static_cast<const DebugDrawing::Polygon*>(e), painter);
        break;
      }
      case DebugDrawing::Element::GRID_RGBA:
      {
        paintGridRGBA(*static_cast<const DebugDrawing::GridRGBA*>(e), painter);
        break;
      }
      case DebugDrawing::Element::GRID_MONO:
      {
        paintGridMono(*static_cast<const DebugDrawing::GridMono*>(e), painter);
        break;
      }
      case DebugDrawing::Element::ELLIPSE:
      {
        paintEllipse(*static_cast<const DebugDrawing::Ellipse*>(e), painter);
        break;
      }
      case DebugDrawing::Element::ARC:
      {
        paintArc(*static_cast<const DebugDrawing::Arc*>(e), painter);
        break;
      }
      case DebugDrawing::Element::RECTANGLE:
      {
        paintRectangle(*static_cast<const DebugDrawing::Rectangle*>(e), painter);
        break;
      }
      case DebugDrawing::Element::LINE:
      {
        paintLine(*static_cast<const DebugDrawing::Line*>(e), painter);
        break;
      }
      case DebugDrawing::Element::ORIGIN:
      {
        paintOrigin(*static_cast<const DebugDrawing::Origin*>(e), painter, baseTrans);
        break;
      }
      case DebugDrawing::Element::TEXT:
      {
        paintText(*static_cast<const DebugDrawing::Text*>(e), painter);
        break;
      }
      default:
        break;
    }
}

void PaintMethods::paintLine(const DebugDrawing::Line& element, QPainter& painter)
{
  if(element.penStyle != Drawings::noPen)
  {
    setPen(element, painter);
    if(element.xStart == element.xEnd && element.yStart == element.yEnd)
    {
      painter.drawPoint(element.xStart, element.yStart);
    }
    else
    {
      painter.drawLine(element.xStart, element.yStart, element.xEnd, element.yEnd);
    }
  }
}

void PaintMethods::paintPolygon(const DebugDrawing::Polygon& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);

  // copy vector2 to QPoints
  const int* points = (const int*)(&element + 1);
  std::vector<QPoint> qpoints;
  for(int n = element.nCount - 1; n >= 0; --n)
    qpoints.push_back(QPoint(points[2 * n], points[2 * n + 1]));

  painter.drawPolygon(qpoints.data(), element.nCount);
}

void PaintMethods::paintEllipse(const DebugDrawing::Ellipse& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);

  if(element.rotation != 0.0f)
  {
    QTransform trans(painter.transform());
    QTransform transBack(painter.transform());
    trans.translate(qreal(element.x), qreal(element.y));
    trans.rotateRadians(qreal(element.rotation));
    painter.setTransform(trans);
    painter.drawEllipse(-element.radiusX, -element.radiusY, 2 * element.radiusX, 2 * element.radiusY);
    painter.setTransform(transBack);
  }
  else
  {
    painter.drawEllipse(element.x - element.radiusX, element.y - element.radiusY, 2 * element.radiusX, 2 * element.radiusY);
  }
}

void PaintMethods::paintArc(const DebugDrawing::Arc& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);
  painter.drawArc(element.x - element.radius, element.y - element.radius, 2 * element.radius, 2 * element.radius, -element.startAngle, -element.spanAngle);
}

void PaintMethods::paintOrigin(const DebugDrawing::Origin& element, QPainter& painter, const QTransform& baseTrans)
{
  QTransform trans(baseTrans);
  trans.translate(qreal(element.x), qreal(element.y));
  trans.rotateRadians(qreal(element.angle));
  painter.setTransform(trans);
}

void PaintMethods::paintText(const DebugDrawing::Text& element, QPainter& painter)
{
  QFont font("Arial", element.fontSize, QFont::Normal);
  pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
  painter.setPen(pen);
  painter.setFont(font);

  QTransform trans(painter.transform());
  QTransform newTrans;
  newTrans.translate(trans.dx(), trans.dy());
  newTrans.scale(std::abs(trans.m11()), std::abs(trans.m22()));
  painter.setTransform(newTrans);
  painter.drawText(QPoint(element.x * (int)sgn(trans.m11()), element.y * (int)sgn(trans.m22())), QObject::tr((const char*)(&element + 1)));
  painter.setTransform(trans);
}

void PaintMethods::paintGridRGBA(const DebugDrawing::GridRGBA& element, QPainter& painter)
{
  QImage image(element.cellsX, element.cellsY, QImage::Format_ARGB32);

  unsigned* pDst = (unsigned*)image.bits();
  unsigned* pEnd = pDst + element.cellsX * element.cellsY;
  const ColorRGBA* pSrc = (const ColorRGBA*)(&element + 1);
  while(pDst != pEnd)
  {
    *pDst++ = pSrc->a << 24 | pSrc->r << 16 | pSrc->g << 8 | pSrc->b;
    ++pSrc;
  }

  float totalWidth = float(element.cellsX * element.cellSize);
  float totalHeight = float(element.cellsY * element.cellSize);
  painter.drawImage(QRectF(element.x - totalWidth / 2.f, element.y - totalHeight / 2.f, totalWidth, totalHeight), image);
}

void PaintMethods::paintGridMono(const DebugDrawing::GridMono& element, QPainter& painter)
{
  QImage image(element.cellsX, element.cellsY, QImage::Format_ARGB32);

  unsigned colors[256];
  for(int i = 0; i < 256; ++i)
  {
    ColorRGBA col(element.baseColor * (1.0f - (static_cast<float>(i) / 255.0f)));
    colors[i] = element.baseColor.a << 24 | col.r << 16 | col.g << 8 | col.b;
  }

  unsigned* pDst = (unsigned*)image.bits();
  unsigned* pEnd = pDst + element.cellsX * element.cellsY;
  const unsigned char* pSrc = (const unsigned char*)(&element + 1);
  while(pDst != pEnd)
    *pDst++ = colors[*pSrc++];

  float totalWidth = float(element.cellsX * element.cellSize);
  float totalHeight = float(element.cellsY * element.cellSize);
  painter.drawImage(QRectF(element.x - totalWidth / 2.f, element.y - totalHeight / 2.f, totalWidth, totalHeight), image);
}

void PaintMethods::paintRectangle(const DebugDrawing::Rectangle& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);

  const QRect dRect(element.topLX, element.topLY, element.w, element.h);

  if(element.rotation != 0.0f)
  {
    const QPoint center = dRect.center();
    QTransform trans(painter.transform());
    QTransform transBack(painter.transform());
    trans.translate(center.x(), center.y());
    trans.rotateRadians(qreal(element.rotation));
    painter.setTransform(trans);
    painter.drawRect(element.topLX - center.x(), element.topLY - center.y(), dRect.width(), dRect.height());
    painter.setTransform(transBack);
  }
  else
  {
    painter.drawRect(dRect);
  }
}

void PaintMethods::setPen(const DebugDrawing::Element& element, QPainter& painter)
{
  if(element.penStyle != Drawings::noPen)
  {
    pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
    pen.setWidth(element.width);
    switch(element.penStyle)
    {
      case Drawings::dashedPen:
        pen.setStyle(Qt::DashLine);
        break;
      case Drawings::dottedPen:
        pen.setStyle(Qt::DotLine);
        break;
      case Drawings::solidPen:
      default:
        pen.setStyle(Qt::SolidLine);
    }
    painter.setPen(pen);
  }
  else
  {
    painter.setPen(noPen);
  }
}

void PaintMethods::setBrush(const Drawings::BrushStyle brushStyle, const ColorRGBA& brushColor, QPainter& painter)
{
  if(brushStyle == Drawings::solidBrush)
  {
    brush.setColor(QColor(brushColor.r, brushColor.g, brushColor.b, brushColor.a));
    painter.setBrush(brush);
  }
  else
  {
    painter.setBrush(noBrush);
  }
}
