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
  virtual void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const;
  virtual QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const;
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
  void mouseMoveEvent(QMouseEvent* me);
  void dragEnterEvent(QDragEnterEvent* e);
  void dragMoveEvent(QDragMoveEvent* e);
  void dropEvent(QDropEvent* e);

  friend class RobotPoolDelegate;
};
