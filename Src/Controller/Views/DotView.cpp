/**
 * @file Controller/Views/DotView.cpp
 * Implementation of a view that displays a dot graph
 * @author Colin Graf
 */

#include <QGraphicsSvgItem>
#include <QGraphicsRectItem>
#include <QPinchGesture>
#include <QWheelEvent>
#include <QDir>
#include <QTextStream>
#include <QProcess>
#include <QMenu>
#include <QFileDialog>
#include <QSettings>
#include <QScrollBar>
#include <qmath.h>

#include "Platform/File.h"
#include "DotView.h"
#include "Controller/RoboCupCtrl.h"

SimRobot::Widget* DotViewObject::createWidget()
{
  return new DotViewWidget(*this);
}

DotViewWidget::DotViewWidget(DotViewObject& dotViewObject) : dotViewObject(dotViewObject)
{
  setScene(new QGraphicsScene(this));
  setTransformationAnchor(AnchorUnderMouse);
  setDragMode(ScrollHandDrag);
  viewport()->grabGesture(Qt::PinchGesture);
  viewport()->setAttribute(Qt::WA_AcceptTouchEvents);
  setViewportUpdateMode(FullViewportUpdate);
  setFrameStyle(QFrame::NoFrame);
  openDotFileContent(dotViewObject.generateDotFileContent());
}

DotViewWidget::~DotViewWidget()
{
  // save window state
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(dotViewObject.fullName);
  settings.setValue("verticalScrollPosition", verticalScrollBar()->value());
  settings.setValue("horizontalScrollPosition", horizontalScrollBar()->value());
  QTransform transform = this->transform();
  settings.setValue("scale", transform.m11());
  settings.endGroup();
}

bool DotViewWidget::openSvgFile(const QString& fileName)
{
  QGraphicsScene* s = scene();
  s->clear();

  QGraphicsSvgItem* svgItem = new QGraphicsSvgItem(fileName);
  if(!svgItem->boundingRect().isValid())
  {
    delete svgItem;
    return false;
  }
  svgItem->setFlags(QGraphicsItem::ItemClipsToShape);
  svgItem->setZValue(0);
  s->addItem(svgItem);

  // load window state
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(dotViewObject.fullName);
  QVariant scaleVar = settings.value("scale");
  if(scaleVar.isValid())
  {
    float scale = scaleVar.toFloat();
    setTransform(QTransform().scale(scale, scale));
  }
  verticalScrollBar()->setValue(settings.value("verticalScrollPosition").toInt());
  horizontalScrollBar()->setValue(settings.value("horizontalScrollPosition").toInt());
  settings.endGroup();

  return true;
}

bool DotViewWidget::viewportEvent(QEvent* event)
{
  if(event->type() == QEvent::Gesture)
  {
    QPinchGesture* pinch = static_cast<QPinchGesture*>(static_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture));
    if(pinch && (pinch->changeFlags() & QPinchGesture::ScaleFactorChanged))
    {
#ifdef FIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      pinch->setLastScaleFactor(1.f);
#endif
      qreal factor = pinch->scaleFactor() / pinch->lastScaleFactor();
      scale(factor, factor);
      return true;
    }
  }
  return QGraphicsView::viewportEvent(event);
}

void DotViewWidget::wheelEvent(QWheelEvent* event)
{
#ifndef MACOS
  // scroll with mouse wheel
  qreal factor = qPow(1.2, event->delta() / 240.0);
  scale(factor, factor);
  event->accept();
#else
  QGraphicsView::wheelEvent(event);
#endif
}

void DotViewWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  // reset scale
  resetTransform();
  event->accept();
}

void DotViewWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      event->accept();
      {
        qreal factor = qPow(1.2, 60.0 / 240.0);
        scale(factor, factor);
      }
      break;
    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      event->accept();
      {
        qreal factor = qPow(1.2, -60.0 / 240.0);
        scale(factor, factor);
      }
      break;
    default:
      QWidget::keyPressEvent(event);
      break;
  }
}

bool DotViewWidget::convertDotFile(const QString& fmt, const QString& src, const QString& dest)
{
  QString cmd = builtDotCommand(fmt, src, dest);
  int exitCode = QProcess::execute(cmd);
  return exitCode != 0;
}

bool DotViewWidget::openDotFile(const QString& fileName)
{
  // convert dot file into svg
  const QString svgFileName = QDir::temp().filePath("DotView.svg");
  QString cmd = builtDotCommand("svg", fileName, svgFileName);
  int exitCode = QProcess::execute(cmd);
  if(exitCode != 0)
    return false;

  // load svg file
  bool result = openSvgFile(svgFileName);
  QFile::remove(svgFileName);
  return result;
}

bool DotViewWidget::saveDotFileContent(const QString& content, const QString& fileName)
{
  if(content.isEmpty())
    return false;

  QFile dotFile(fileName);
  if(!dotFile.open(QIODevice::WriteOnly | QIODevice::Text))
    return false;
  QTextStream out(&dotFile);
  out << content;
  return true;
}

bool DotViewWidget::openDotFileContent(const QString& content)
{
  if(content.isEmpty())
    return false;

  // generate dot file
  const QString dotFileName = QDir::temp().filePath("DotView.dot");
  if(!saveDotFileContent(content, dotFileName))
    return false;

  // open dot fiile
  bool result = openDotFile(dotFileName);
  QFile::remove(dotFileName);
  return result;
}

void DotViewWidget::update()
{
  if(!dotViewObject.hasChanged())
    return;
  openDotFileContent(dotViewObject.generateDotFileContent());
}

QString DotViewWidget::builtDotCommand(const QString& fmt, const QString& src, const QString& dest) const
{
#ifdef WINDOWS
  return QString("\"%1\\Util\\dot\\dot.exe\" -T%2 -o \"%3\" \"%4\"").arg(File::getBHDir(), fmt, dest, src);
#else
  return QString("\"%1/Util/dot/dot\" -T%2 -o \"%3\" \"%4\"").arg(File::getBHDir(), fmt, dest, src);
#endif
}

QMenu* DotViewWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Graph"));
  QAction* action = menu->addAction(tr("&Export as SVG..."));
  action->setStatusTip(tr("Export the window drawing as svg"));
  connect(action, SIGNAL(triggered()), this, SLOT(exportAsSvg()));
  action = menu->addAction(tr("Export as &DOT..."));
  action->setStatusTip(tr("Export the window drawing as dot file"));
  connect(action, SIGNAL(triggered()), this, SLOT(exportAsDot()));
  action = menu->addAction(tr("Export as &PDF..."));
  action->setStatusTip(tr("Export the window drawing as pdf"));
  connect(action, SIGNAL(triggered()), this, SLOT(exportAsPdf()));
  return menu;
}

void DotViewWidget::exportAsSvg()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Export as SVG"), settings.value("ExportDirectory", "").toString(), tr("Scalable Vector Graphics (*.svg)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  const QString dotFileName = QDir::temp().filePath("DotView.dot");
  if(!saveDotFileContent(dotViewObject.generateDotFileContent(), dotFileName))
    return;
  convertDotFile("svg", dotFileName, fileName);
  QFile::remove(dotFileName);
}

void DotViewWidget::exportAsPdf()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Export as PDF"), settings.value("ExportDirectory", "").toString(), tr("Portable Document Format (*.pdf)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  const QString dotFileName = QDir::temp().filePath("DotView.dot");
  if(!saveDotFileContent(dotViewObject.generateDotFileContent(), dotFileName))
    return;
  convertDotFile("pdf", dotFileName, fileName);
  QFile::remove(dotFileName);
}

void DotViewWidget::exportAsDot()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Export as DOT File"), settings.value("ExportDirectory", "").toString(), tr("Graphviz Graph File (*.dot)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  saveDotFileContent(dotViewObject.generateDotFileContent(), fileName);
}
