/**
 * @file Controller/Visualization/HeaderedWidget.cpp
 * Implementation of class HeaderedWidget.
 *
 * @author Colin Graf
 */

#include <QHeaderView>
#include <QVBoxLayout>
#include <QStandardItemModel>
#include <QScrollArea>
#include <QResizeEvent>

#include "HeaderedWidget.h"

HeaderedWidget::HeaderedWidget(QWidget* parent) : QScrollArea(parent)
{
  setFrameStyle(QFrame::NoFrame);
  setWidgetResizable(true);

  headerView = new QHeaderView(Qt::Horizontal, this);
  headerView->setStretchLastSection(true);
  headerView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  headerItemModel = new QStandardItemModel(headerView);
  headerView->setModel(headerItemModel);
  headerView->move(frameWidth(), frameWidth());
}

void HeaderedWidget::setWidget(QWidget* widget)
{
  QScrollArea::setWidget(widget);
  setFocusProxy(widget);
}

QHeaderView* HeaderedWidget::getHeaderView()
{
  return headerView;
}

void HeaderedWidget::setHeaderLabels(const QStringList& headerLabels, const char* aligns)
{
  headerItemModel->setHorizontalHeaderLabels(headerLabels);
  int headerHeight = headerView->sizeHint().height();
  headerView->setFixedHeight(headerHeight);
  setViewportMargins(0, headerHeight, 0, 0);
  for(int i = headerLabels.count() - 1; i >= 0; i--)
    headerItemModel->horizontalHeaderItem(i)->setTextAlignment((aligns == 0 || aligns[i] == 'l') ? Qt::AlignLeft : (aligns[i] == 'r' ? Qt::AlignRight : Qt::AlignCenter));
}

void HeaderedWidget::resizeEvent(QResizeEvent* event)
{
  QSize headerSize = event->size();
  headerSize.setHeight(headerView->sizeHint().height());
  headerView->resize(headerSize);

  QScrollArea::resizeEvent(event);
}

QSize HeaderedWidget::sizeHint() const
{
  QWidget* widget = this->widget();
  QSize size;
  if(widget)
    size = widget->sizeHint();
  size.rheight() += headerView->sizeHint().height();
  return size;
}
