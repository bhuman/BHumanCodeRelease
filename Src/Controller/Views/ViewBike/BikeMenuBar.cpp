/**
* @file Controller/Views/ViewBike/BikeMenuBar.cpp
*
* Implementation of class BikeMenuBar
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#include "BikeMenuBar.h"

BikeMenuBar::BikeMenuBar()
{
  createActions();
  createMenus();
}

void BikeMenuBar::createActions()
{
  dragPlaneMenu = new QMenu(tr("&Drag Plane"), this);

  //drag menu
  xy_plane = new QAction(tr("XY-Plane"), this);
  xy_plane->setShortcut(QKeySequence(Qt::Key_Z));
  xy_plane->setCheckable(true);
  xz_plane = new QAction(tr("XZ-Plane"), this);
  xz_plane->setShortcut(QKeySequence(Qt::Key_Y));
  xz_plane->setCheckable(true);
  yz_plane = new QAction(tr("YZ-Plane"), this);
  yz_plane->setShortcut(QKeySequence(Qt::Key_X));
  yz_plane->setCheckable(true);

  dragPlaneActionGroup = new QActionGroup(this);
  dragPlaneActionGroup->addAction(xy_plane);
  dragPlaneActionGroup->addAction(xz_plane);
  dragPlaneActionGroup->addAction(yz_plane);
}

void BikeMenuBar::createMenus()
{
  dragPlaneMenu->addAction(xy_plane);
  dragPlaneMenu->addAction(xz_plane);
  dragPlaneMenu->addAction(yz_plane);
}

