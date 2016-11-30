/**
 * @file Controller/Views/KickView/KickMenuBar.cpp
 *
 * Implementation of class KickMenuBar
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#include "KickMenuBar.h"

KickMenuBar::KickMenuBar()
{
  createActions();
  createMenus();
}

void KickMenuBar::createActions()
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

void KickMenuBar::createMenus()
{
  dragPlaneMenu->addAction(xy_plane);
  dragPlaneMenu->addAction(xz_plane);
  dragPlaneMenu->addAction(yz_plane);
}
