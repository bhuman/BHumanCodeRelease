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
   * @param parent The parent widget.
   */
  HeaderedWidget(QWidget* parent = nullptr);

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
  void setHeaderLabels(const QStringList& headerLabels, const char* aligns = nullptr);

protected:
  QHeaderView* headerView; /**< The header view. */
  QStandardItemModel* headerItemModel; /**< A simple item model for the header view. */

  void resizeEvent(QResizeEvent* event) override;

  QSize sizeHint() const override;
};
