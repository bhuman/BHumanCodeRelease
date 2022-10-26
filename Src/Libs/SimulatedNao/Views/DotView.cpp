/**
 * @file SimulatedNao/Views/DotView.cpp
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
#include "SimulatedNao/RoboCupCtrl.h"

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
  // scroll with mouse wheel
  qreal factor = qPow(1.2, event->angleDelta().y() / 240.0);
  scale(factor, factor);
  event->accept();
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
  QString dot = QString(File::getBHDir()) +
#ifdef WINDOWS
    "\\Util\\dot\\dot.exe";
#else
    "/Util/dot/dot";
#endif
  QStringList args;
  args.append("-T" + fmt);
  args.append("-o");
  args.append(dest);
  args.append(src);
  return QProcess::execute(dot, args) == 0;
}

bool DotViewWidget::openDotFile(const QString& fileName)
{
  // convert dot file into svg
  const QString svgFileName = QDir::temp().filePath("DotView.svg");
  if(!convertDotFile("svg", fileName, svgFileName))
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

QMenu* DotViewWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Graph"));
  QAction* action = menu->addAction(tr("&Export as SVG..."));
  action->setStatusTip(tr("Export the window drawing as svg"));
  connect(action, &QAction::triggered, this, &DotViewWidget::exportAsSvg);
  action = menu->addAction(tr("Export as &DOT..."));
  action->setStatusTip(tr("Export the window drawing as dot file"));
  connect(action, &QAction::triggered, this, &DotViewWidget::exportAsDot);
  action = menu->addAction(tr("Export as &PDF..."));
  action->setStatusTip(tr("Export the window drawing as pdf"));
  connect(action, &QAction::triggered, this, &DotViewWidget::exportAsPdf);
  return menu;
}

void DotViewWidget::exportAsSvg()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Export as SVG"), settings.value("ExportDirectory", "").toString(), tr("Scalable Vector Graphics (*.svg)")
#ifdef LINUX
                     , nullptr, QFileDialog::DontUseNativeDialog
#endif
                     );
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
                     tr("Export as PDF"), settings.value("ExportDirectory", "").toString(), tr("Portable Document Format (*.pdf)")
#ifdef LINUX
                     , nullptr, QFileDialog::DontUseNativeDialog
#endif
                     );
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
                     tr("Export as DOT File"), settings.value("ExportDirectory", "").toString(), tr("Graphviz Graph File (*.dot)")
#ifdef LINUX
                     , nullptr, QFileDialog::DontUseNativeDialog
#endif
                     );
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  saveDotFileContent(dotViewObject.generateDotFileContent(), fileName);
}
