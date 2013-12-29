/**
* @file Controller/Visualization/HeaderedWidget.h
* Declaration of class HeaderedWidget.
*
* @author Colin Graf
*/

#pragma once

#include <QScrollArea>

class QHeaderView;
class QScrollArea;
class QBoxLayout;
class QStandardItemModel;

/**
* @class HeaderedWidget
*
* Defines a QWidget that contains a QHeaderView and another QWidget
*/

class HeaderedWidget : public QScrollArea
{
public:
  /**
  * Constructor.
  * @param parent The parent widget.
  */
  HeaderedWidget(QWidget* parent = 0);



  virtual ~HeaderedWidget()
   {
     /*
      * This dtor was introduced to fix the following linking bug that appeared when building Debug:
      *
      *
      * `.text._ZN14HeaderedWidgetD2Ev' referenced in section `.text._ZN14HeaderedWidgetD1Ev[non-virtual thunk to
      *  HeaderedWidget::~HeaderedWidget()]' of ../../Build/Controller/Linux/Debug/libController.a(HeaderedWidget.o):
      *   defined in discarded section `.text._ZN14HeaderedWidgetD2Ev[_ZN14HeaderedWidgetD5Ev]'
      *    of ../../Build/Controller/Linux/Debug/libController.a(HeaderedWidget.o)
      */

   };


  /**
  * Sets the content Widget of this HeaderedWidget.
  * @param widget The widget.
  */
  void setWidget(QWidget* widget);

  /**
  * Returns the header view of this widget.
  * @return The header view.
  */
  QHeaderView* getHeaderView();

  /**
  * Sets the header labels of the header view.
  * @param headerLabels A list of column descriptions.
  * @param aligns The align of each label (e.g. "lrrcrr") or 0.
  */
  void setHeaderLabels(const QStringList& headerLabels, const char* aligns = 0);

protected:
  QHeaderView* headerView; /**< The header view. */
  QStandardItemModel* headerItemModel; /**< A simple item model for the header view. */

  virtual void resizeEvent(QResizeEvent* event);

  virtual QSize sizeHint() const;
};
