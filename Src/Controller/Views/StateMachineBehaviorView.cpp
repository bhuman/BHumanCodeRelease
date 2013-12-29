/**
* @file Controller/Views/StateMachineBehaviorView.cpp
* Implementation of class StateMachineBehaviorView
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QSettings>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "StateMachineBehaviorView.h"
#include "Platform/Thread.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/HeaderedWidget.h"

class StateMachineBehaviorWidget : public QWidget
{
public:
  StateMachineBehaviorWidget(StateMachineBehaviorView& stateMachineBehaviorView, QWidget* parent) : QWidget(parent),
    stateMachineBehaviorView(stateMachineBehaviorView), noPen(Qt::NoPen)
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
      SYNC_WITH(stateMachineBehaviorView.console);
      if(stateMachineBehaviorView.timeStamp == lastStateMachineBehaviorDebugInfoTimeStamp)
        return;
      lastStateMachineBehaviorDebugInfoTimeStamp = stateMachineBehaviorView.timeStamp;
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

  void paintEvent(QPaintEvent *event)
  {
    painter.begin(this);
    painter.setFont(font);
    painter.setBrush(altBrush);
    painter.setPen(fontPen);
    fillBackground = false;

    char formatedTime[65];

    paintRect = painter.window();
    int defaultLeft = textOffset;
    paintRectField0 = QRect(defaultLeft, 0, paintRect.right() - textOffset * 2, lineSpacing);
    {
      SYNC_WITH(stateMachineBehaviorView.console);
      const ActivationGraph& info(stateMachineBehaviorView.activationGraph);

      for(std::vector<ActivationGraph::Node>::const_iterator i = info.graph.begin(), end = info.graph.end(); i != end; ++i)
      {
        const ActivationGraph::Node& activeOption = *i;
        paintRectField0.setLeft(defaultLeft + 10 * activeOption.depth);

        sprintf(formatedTime, "%.02f", float(activeOption.optionTime) / 1000.f);
        print(activeOption.option.c_str(), formatedTime, true, true);

        paintRectField0.setLeft(paintRectField0.left() + 5);
        sprintf(formatedTime, "%.02f", float(activeOption.stateTime) / 1000.f);
        print(activeOption.state.c_str(), formatedTime, false, true);
        newBlock();
      }
    }
    painter.end();
    int minHeight = paintRectField0.top();
    if(minHeight > 0)
      setMinimumHeight(minHeight);
  }

private:
  StateMachineBehaviorView& stateMachineBehaviorView;
  unsigned int lastStateMachineBehaviorDebugInfoTimeStamp; /**< Timestamp of the last painted info. */
  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QFont boldFont;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;

  void print(const char* name, const char* value, bool bold = false, bool rightAlign = false)
  {
    if(fillBackground)
    {
      painter.setPen(noPen);
      painter.drawRect(paintRect.left(), paintRectField0.top(), paintRect.width(), paintRectField0.height());
      painter.setPen(fontPen);
    }
    if(name)
    {
      if(bold)
        painter.setFont(boldFont);
      painter.drawText(paintRectField0, (!value ? Qt::TextDontClip : 0) | Qt::TextSingleLine | Qt::AlignVCenter, tr(name));
      if(bold)
        painter.setFont(font);
    }
    if(value)
      painter.drawText(paintRectField0, (rightAlign ? Qt::AlignRight : Qt::AlignLeft) | Qt::TextSingleLine | Qt::AlignVCenter, tr(value));
    paintRectField0.moveTop(paintRectField0.top() + lineSpacing);
  }

  void newBlock()
  {
    fillBackground = fillBackground ? false : true;
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

  QSize sizeHint () const { return QSize(200, 500); }
};

class StateMachineBehaviorScrollingWidget : public QScrollArea, public SimRobot::Widget
{
public:
  StateMachineBehaviorScrollingWidget(StateMachineBehaviorView& cabslView)
  {
    setFrameStyle(QFrame::NoFrame);
    setWidgetResizable(true);

    stateMachineBehaviorWidget = new StateMachineBehaviorWidget(cabslView, this);
    setWidget(stateMachineBehaviorWidget);
    setFocusProxy(stateMachineBehaviorWidget);
  }

private:
  StateMachineBehaviorWidget* stateMachineBehaviorWidget;
  virtual QWidget* getWidget() {return this;}
  virtual void update() {stateMachineBehaviorWidget->update();}
};

StateMachineBehaviorView::StateMachineBehaviorView(const QString& fullName, RobotConsole& console, const ActivationGraph& activationGraph, const unsigned& timeStamp) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), activationGraph(activationGraph), timeStamp(timeStamp) {}

SimRobot::Widget* StateMachineBehaviorView::createWidget()
{
  return new StateMachineBehaviorScrollingWidget(*this);
}
