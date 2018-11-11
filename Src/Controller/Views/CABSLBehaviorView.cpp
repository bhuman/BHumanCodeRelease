/**
 * @file Controller/Views/CABSLBehaviorView.cpp
 * Implementation of class CABSLBehaviorView
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author Colin Graf
 */

#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QScrollArea>
#include <QSettings>

#include "CABSLBehaviorView.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/RobotConsole.h"

class CABSLBehaviorWidget : public QWidget
{
public:
  CABSLBehaviorWidget(CABSLBehaviorView& cabslBehaviorView, QWidget* parent) :
    QWidget(parent), cabslBehaviorView(cabslBehaviorView)
  {
    setFocusPolicy(Qt::StrongFocus);
    setBackgroundRole(QPalette::Base);

    font = QApplication::font();
    boldFont = font;
    boldFont.setBold(true);
    setFontPointSize(getFontPointSize());

    const QPalette& pal(QApplication::palette());
    altBrush = pal.alternateBase();
    fontPen.setColor(pal.text().color());

    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  void update()
  {
    {
      SYNC_WITH(cabslBehaviorView.console);
      if(cabslBehaviorView.timeStamp == lastCABSLBehaviorDebugInfoTimeStamp)
        return;
      lastCABSLBehaviorDebugInfoTimeStamp = cabslBehaviorView.timeStamp;
    }
    QWidget::update();
  }

  int getFontPointSize()
  {
    return font.pointSize();
  }

  void setFontPointSize(int size)
  {
    font.setPointSize(size);
    boldFont.setPointSize(size);
    QFontMetrics me(font);
    lineSpacing = me.lineSpacing() + 2;
    textOffset = me.descent() + 1;
  }

  void paintEvent(QPaintEvent* event)
  {
    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    char formattedTime[65];

    paintRect = painter.window();
    int defaultLeft = textOffset;
    paintRectField0 = QRect(defaultLeft, 0, paintRect.right() - textOffset * 2, lineSpacing);
    {
      SYNC_WITH(cabslBehaviorView.console);
      const ActivationGraph& info(cabslBehaviorView.activationGraph);

      for(const ActivationGraph::Node& activeOption : info.graph)
      {
        paintRectField0.setLeft(defaultLeft + 10 * activeOption.depth);

        sprintf(formattedTime, "%.02f", float(activeOption.optionTime) / 1000.f);
        print(activeOption.option.c_str(), formattedTime, true, true);

        paintRectField0.setLeft(defaultLeft + 10 * activeOption.depth + 5);
        for(const std::string& parameter : activeOption.parameters)
          print(parameter.c_str(), "", false, false);

        sprintf(formattedTime, "%.02f", float(activeOption.stateTime) / 1000.f);
        print(("state = " + activeOption.state).c_str(), formattedTime, false, true);
        newBlock();
      }
    }
    painter.end();
    int minHeight = paintRectField0.top();
    if(minHeight > 0)
      setMinimumHeight(minHeight);
  }

private:
  CABSLBehaviorView& cabslBehaviorView;
  unsigned int lastCABSLBehaviorDebugInfoTimeStamp; /**< Timestamp of the last painted info. */
  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QFont boldFont;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen = Qt::NoPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;

  void print(const std::string& name, const std::string& value, bool bold = false, bool rightAlign = false)
  {
    if(fillBackground)
    {
      painter.setPen(noPen);
      painter.drawRect(paintRect.left(), paintRectField0.top(), paintRect.width(), paintRectField0.height());
      painter.setPen(fontPen);
    }
    if(bold)
      painter.setFont(boldFont);
    painter.drawText(paintRectField0, Qt::TextSingleLine | Qt::AlignVCenter, name.c_str());
    if(bold)
      painter.setFont(font);

    painter.drawText(paintRectField0, (rightAlign ? Qt::AlignRight : Qt::AlignLeft) | Qt::TextSingleLine | Qt::AlignVCenter, value.c_str());
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
  }

  void newBlock()
  {
    fillBackground = !fillBackground;
  }

  void newSection()
  {
    painter.drawLine(paintRect.left(), paintRectField0.top(), paintRect.width(), paintRectField0.top());
    paintRectField0.moveTop(paintRectField0.top() + 1);
    fillBackground = false;
  }

  void keyPressEvent(QKeyEvent* event)
  {
    switch(event->key())
    {
      case Qt::Key_PageUp:
      case Qt::Key_Plus:
        event->accept();
        if(getFontPointSize() < 48)
          setFontPointSize(getFontPointSize() + 1);
        QWidget::update();
        break;
      case Qt::Key_PageDown:
      case Qt::Key_Minus:
        event->accept();
        if(getFontPointSize() > 3)
          setFontPointSize(getFontPointSize() - 1);
        QWidget::update();
        break;
      default:
        QWidget::keyPressEvent(event);
        break;
    }
  }

  QSize sizeHint() const { return QSize(200, 500); }
};

class CABSLBehaviorScrollingWidget : public QScrollArea, public SimRobot::Widget
{
public:
  CABSLBehaviorScrollingWidget(CABSLBehaviorView& cabslBehaviorView)
  {
    setFrameStyle(QFrame::NoFrame);
    setWidgetResizable(true);

    cabslBehaviorWidget = new CABSLBehaviorWidget(cabslBehaviorView, this);
    setWidget(cabslBehaviorWidget);
    setFocusProxy(cabslBehaviorWidget);
  }

private:
  CABSLBehaviorWidget* cabslBehaviorWidget;
  QWidget* getWidget() override { return this; }
  void update() override { cabslBehaviorWidget->update(); }
};

CABSLBehaviorView::CABSLBehaviorView(const QString& fullName, RobotConsole& console, const ActivationGraph& activationGraph, const unsigned& timeStamp) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), activationGraph(activationGraph), timeStamp(timeStamp)
{}

SimRobot::Widget* CABSLBehaviorView::createWidget()
{
  return new CABSLBehaviorScrollingWidget(*this);
}
