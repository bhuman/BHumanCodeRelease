#pragma once

#include <QListWidget>
#include <QStyledItemDelegate>

class QWidget;
class QListWidget;
class TeamSelector;
class RobotView;
class RobotPool;

class RobotPoolDelegate : public QStyledItemDelegate
{
  RobotPool* robotPool;
public:
  RobotPoolDelegate(RobotPool* pool);
  void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
  QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};

class RobotPool : public QListWidget
{
  Q_OBJECT

  TeamSelector* teamSelector;
  QMap<QString, RobotView*> robotViews;
  RobotView* toBeDeletedLater;
public:
  RobotPool(TeamSelector* team);
public slots:
  void update();
protected:
  void mouseMoveEvent(QMouseEvent* me) override;
  void dragEnterEvent(QDragEnterEvent* e) override;
  void dragMoveEvent(QDragMoveEvent* e) override;
  void dropEvent(QDropEvent* e) override;
  void paintEvent(QPaintEvent* e) override;

  friend class RobotPoolDelegate;
};
