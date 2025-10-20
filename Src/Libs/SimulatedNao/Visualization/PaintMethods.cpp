/**
 * @file Visualization/PaintMethods.cpp
 * Implementation of class PaintMethods.
 *
 * @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
 * @author Colin Graf
 */

#include <QObject>
#include <QPainter>
#include <QPainterPath>
#include "PaintMethods.h"
#include "Platform/File.h"

QBrush PaintMethods::brush(Qt::SolidPattern);
QBrush PaintMethods::noBrush(Qt::NoBrush);
QPen PaintMethods::pen;
QPen PaintMethods::noPen(Qt::NoPen);
QImage PaintMethods::robot = QImage((std::string(File::getBHDir()) + "/Src/Libs/SimulatedNao/Icons/robot.png").c_str());

void PaintMethods::paintDebugDrawing(QPainter& painter, const DebugDrawing& debugDrawing, const QTransform& baseTrans)
{
  for(const DebugDrawing::Element* e = debugDrawing.getFirst(); e; e = debugDrawing.getNext(e))
    switch(e->type)
    {
      case DebugDrawing::ElementType::polygon:
        paintPolygon(*static_cast<const DebugDrawing::Polygon*>(e), painter);
        break;
      case DebugDrawing::ElementType::ellipse:
        paintEllipse(*static_cast<const DebugDrawing::Ellipse*>(e), painter);
        break;
      case DebugDrawing::ElementType::arc:
        paintArc(*static_cast<const DebugDrawing::Arc*>(e), painter);
        break;
      case DebugDrawing::ElementType::rectangle:
        paintRectangle(*static_cast<const DebugDrawing::Rectangle*>(e), painter);
        break;
      case DebugDrawing::ElementType::line:
        paintLine(*static_cast<const DebugDrawing::Line*>(e), painter);
        break;
      case DebugDrawing::ElementType::origin:
        paintOrigin(*static_cast<const DebugDrawing::Origin*>(e), painter, baseTrans);
        break;
      case DebugDrawing::ElementType::text:
        paintText(*static_cast<const DebugDrawing::Text*>(e), painter, baseTrans);
        break;
      case DebugDrawing::ElementType::robot:
        paintRobot(*static_cast<const DebugDrawing::Robot*>(e), painter);
        break;
      case DebugDrawing::ElementType::gridMono:
        paintGridMono(*static_cast<const DebugDrawing::GridMono*>(e), painter);
        break;
      case DebugDrawing::ElementType::gridRGBA:
        paintGridRGBA(*static_cast<const DebugDrawing::GridRGBA*>(e), painter);
        break;
      case DebugDrawing::ElementType::gridRectangleRGBA:
        paintGridRectangleRGBA(*static_cast<const DebugDrawing::GridRectangleRGBA*>(e), painter);
        break;
      default:
        break;
    }
}

void PaintMethods::paintLine(const DebugDrawing::Line& element, QPainter& painter)
{
  if(element.penStyle != Drawings::noPen)
  {
    setPen(element, painter);
    if(element.start == element.end)
      painter.drawPoint(QPointF(element.start.x() + 0.5f, element.start.y() + 0.5f));
    else
      painter.drawLine(QLineF(element.start.x() + 0.5f, element.start.y() + 0.5f, element.end.x() + 0.5f, element.end.y() + 0.5f));
  }
}

void PaintMethods::paintPolygon(const DebugDrawing::Polygon& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);

  // copy vector2 to QPoints
  const int* points = reinterpret_cast<const int*>(&element + 1);
  std::vector<QPoint> qPoints;
  for(int n = element.nCount - 1; n >= 0; --n)
    qPoints.push_back(QPoint(points[2 * n], points[2 * n + 1]));

  painter.drawPolygon(qPoints.data(), element.nCount);
}

void PaintMethods::paintEllipse(const DebugDrawing::Ellipse& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);

  if(element.rotation != 0.0f)
  {
    QTransform trans(painter.transform());
    QTransform transBack(painter.transform());
    trans.translate(qreal(element.center.x()), qreal(element.center.y()));
    trans.rotateRadians(qreal(element.rotation));
    painter.setTransform(trans);
    painter.drawEllipse(-element.radii.x(), -element.radii.y(), 2 * element.radii.x(), 2 * element.radii.y());
    painter.setTransform(transBack);
  }
  else
    painter.drawEllipse(element.center.x() - element.radii.x(), element.center.y() - element.radii.y(), 2 * element.radii.x(), 2 * element.radii.y());
}

void PaintMethods::paintArc(const DebugDrawing::Arc& element, QPainter& painter)
{
  setBrush(element.brushStyle, element.brushColor, painter);
  setPen(element, painter);
  if(element.brushStyle == Drawings::noBrush)
    painter.drawArc(element.center.x() - element.radius, element.center.y() - element.radius,
                    2 * element.radius, 2 * element.radius, -(static_cast<int>(element.startAngle.toDegrees()) * 16), -(static_cast<int>(element.spanAngle.toDegrees()) * 16));
  else
  {
    QPainterPath path;
    path.moveTo(element.center.x(), element.center.y());
    path.arcTo(element.center.x() - element.radius, element.center.y() - element.radius,
               2 * element.radius, 2 * element.radius, -element.startAngle.toDegrees(), -element.spanAngle.toDegrees());
    path.closeSubpath();
    painter.drawPath(path);
  }
}

void PaintMethods::paintOrigin(const DebugDrawing::Origin& element, QPainter& painter, const QTransform& baseTrans)
{
  QTransform trans(baseTrans);
  trans.translate(qreal(element.translation.x()), qreal(element.translation.y()));
  trans.rotateRadians(qreal(element.angle));
  painter.setTransform(trans);
}

void PaintMethods::paintText(const DebugDrawing::Text& element, QPainter& painter, const QTransform& baseTrans)
{
  QFont font("Arial", element.fontSize, QFont::Normal);
  pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
  painter.setPen(pen);
  painter.setFont(font);

  QTransform trans(painter.transform());
  QTransform newTrans(trans);
  newTrans.translate(element.x, element.y);
  newTrans.rotateRadians(std::atan2(trans.m21(), trans.m11()));
  newTrans.scale(sgn(baseTrans.m11()), sgn(baseTrans.m22()));
  painter.setTransform(newTrans);
  painter.drawText(QPoint(), QObject::tr(reinterpret_cast<const char*>(&element + 1)));
  painter.setTransform(trans);
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
    painter.drawRect(dRect);
}

void PaintMethods::paintRobot(const DebugDrawing::Robot& element, QPainter& painter)
{
  QColor colorBody(element.colorBody.r, element.colorBody.g, element.colorBody.b, element.colorBody.a);
  QColor colorDirVec(element.colorDirVec.r, element.colorDirVec.g, element.colorDirVec.b, element.colorDirVec.a);
  QColor colorDirHeadVec(element.colorDirHeadVec.r, element.colorDirHeadVec.g, element.colorDirHeadVec.b, element.colorDirHeadVec.a);

  /** Draws a color on top of a copy the robot (ignored if alpha is 0) */
  QImage mask(element.colorBody.a != 0 ? robot.copy() : robot);
  if(element.colorBody.a != 0)
  {
    QPainter p;
    p.begin(&mask);
    p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
    QRectF rect(0, 0, mask.width(), mask.height());
    p.setOpacity(0.5);
    p.fillRect(rect, colorBody);
    p.end();
  }

  QTransform trans(painter.transform());
  QTransform transBack(painter.transform());
  trans.translate(element.p.translation.x(), element.p.translation.y());
  trans.rotateRadians(qreal(element.p.rotation));
  trans.translate(-robot.width() / 2, -robot.height() / 2);
  painter.setTransform(trans);

  painter.setOpacity(element.alphaRobot);

  painter.drawImage(0, 0, mask);

  painter.setTransform(transBack);

  painter.setOpacity(1.0);

  /** Draws the direction vector of the robot (ignored if alpha is 0) */
  if(element.colorDirVec.a != 0)
  {
    pen.setColor(colorDirVec);
    pen.setWidthF(20);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawLine(QLineF(element.p.translation.x() + 0.5f, element.p.translation.y() + 0.5f, element.dirVec.x() + 0.5f, element.dirVec.y() + 0.5f));
  }

  /** Draws the head rotation of the robot (ignored if alpha is 0) */
  if(element.colorDirHeadVec.a != 0)
  {
    pen.setWidthF(20);
    pen.setColor(colorDirHeadVec);
    painter.setPen(pen);
    painter.drawLine(QLineF(element.p.translation.x() + 0.5f, element.p.translation.y() + 0.5f, element.dirHeadVec.x() + 0.5f, element.dirHeadVec.y() + 0.5f));
  }
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

  unsigned* pDst = reinterpret_cast<unsigned*>(image.bits());
  unsigned* pEnd = pDst + element.cellsX * element.cellsY;
  const unsigned char* pSrc = reinterpret_cast<const unsigned char*>(&element + 1);
  while(pDst != pEnd)
  {
    *pDst++ = colors[*pSrc++];
  }

  float totalWidth = float(element.cellsX * element.cellSize);
  float totalHeight = float(element.cellsY * element.cellSize);
  painter.drawImage(QRectF(element.x - totalWidth / 2.f, element.y - totalHeight / 2.f, totalWidth, totalHeight), image);
}

void PaintMethods::paintGridRGBA(const DebugDrawing::GridRGBA& element, QPainter& painter)
{
  QImage image(element.cellsX, element.cellsY, QImage::Format_ARGB32);

  unsigned* pDst = reinterpret_cast<unsigned*>(image.bits());
  unsigned* pEnd = pDst + element.cellsX * element.cellsY;
  const ColorRGBA* pSrc = reinterpret_cast<const ColorRGBA*>(&element + 1);
  while(pDst != pEnd)
  {
    *pDst++ = pSrc->a << 24 | pSrc->r << 16 | pSrc->g << 8 | pSrc->b;
    ++pSrc;
  }

  float totalWidth = float(element.cellsX * element.cellSize);
  float totalHeight = float(element.cellsY * element.cellSize);
  painter.drawImage(QRectF(element.x - totalWidth / 2.f, element.y - totalHeight / 2.f, totalWidth, totalHeight), image);
}

void PaintMethods::paintGridRectangleRGBA(const DebugDrawing::GridRectangleRGBA& element, QPainter& painter)
{
  QImage image(element.cellsX, element.cellsY, QImage::Format_ARGB32);

  unsigned* pDst = reinterpret_cast<unsigned*>(image.bits());
  unsigned* pEnd = pDst + element.cellsX * element.cellsY;
  const ColorRGBA* pSrc = reinterpret_cast<const ColorRGBA*>(&element + 1);
  while(pDst != pEnd)
  {
    *pDst++ = pSrc->a << 24 | pSrc->r << 16 | pSrc->g << 8 | pSrc->b;
    ++pSrc;
  }

  float totalWidth = float(element.cellsX * element.width);
  float totalHeight = float(element.cellsY * element.height);
  painter.drawImage(QRectF(element.x - totalWidth / 2.f, element.y - totalHeight / 2.f, totalWidth, totalHeight), image);
}

void PaintMethods::setPen(const DebugDrawing::Element& element, QPainter& painter)
{
  if(element.penStyle != Drawings::noPen)
  {
    pen.setColor(QColor(element.penColor.r, element.penColor.g, element.penColor.b, element.penColor.a));
    pen.setWidthF(element.width);
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
    painter.setPen(noPen);
}

void PaintMethods::setBrush(const Drawings::BrushStyle brushStyle, const ColorRGBA& brushColor, QPainter& painter)
{
  if(brushStyle == Drawings::solidBrush)
  {
    brush.setColor(QColor(brushColor.r, brushColor.g, brushColor.b, brushColor.a));
    painter.setBrush(brush);
  }
  else
    painter.setBrush(noBrush);
}
