/**
 * @file Controller/Views/ViewBike/ViewBikeWidget.h
 *
 * Declaration of class ViewBikeWidget
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include <QWidget>
#include <QTreeView>
#include <QStandardItem>
#include <QPushButton>
#include <QSignalMapper>
#include <QCheckBox>

#include "ViewBike.h"
#include "ViewBikeGLWidget.h"
#include "TabWidget.h"
#include "BikeMenuBar.h"
#include "Tools/Math/Vector3.h"
#include "Modules/MotionControl/BIKEParameters.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/MotionRequest.h"

class ViewBikeHeaderedWidget;

class Selected
{
public:
  int phaseNumber;
  unsigned int pointNumber;
  int limb;
  bool xzRot;

  Selected() {};
};

class ViewBikeWidget : public QWidget
{
  Q_OBJECT

public:
  ViewBikeWidget(ViewBike& viewBike, BIKEParameters& parameters, ViewBikeHeaderedWidget* parent);
  virtual ~ViewBikeWidget();
  void updateEditorView();
  void fillModelWithPhaseData(int i);
  void updateCommon();
  void updateGL() {glWidget->update();}
  void update() {QWidget::update(); glWidget->update();}

private slots:
  //Buttons
  void setDrawings(bool value);
  void setSingleDrawing(bool value);
  void setReachedDrawing(bool value);
  void setEditor(bool value);
  void setTra2d(bool value);
  void setTra1d(bool value);
  void setVelocity(bool value);
  void setAccel(bool value);
  void setFollowMode(bool value);

  void removePhase();
  void addPhaseAfterActual();
  void playWholeMotion();
  void playMotionTilActive();
  void resetRobot();
  void standRobot();
  void setMirrored(int state);;
  void recordPose();
  void setHardness(int limb);
  //Sliders
  void transparencyChanged(const int& i);
  //Tab
  void updateCommonParameters(QStandardItem* item);
  void updatePhaseParameters(QStandardItem* item);
  void movePhase(const int fromIndex, int toIndex);
  void setSelectedFromEditor(const int&);
  //Contextmenu
  void setDragPlane(const int& plane);

private:
  ViewBikeHeaderedWidget* parent;
  ViewBikeGLWidget* glWidget;

  //TreeView, StandardModel and Editor
  QList <QStandardItem*> makeStringAndValueRow(QString string, QVariant value);
  void addStringAndValueToList(QList <QStandardItem*> &list, QString string, QVariant value);
  QStandardItemModel* makeNewModelWithLabels();

private:

  bool deleteKids(QStandardItem* rootItem);
  void addControlPoints(QStandardItem* item);
  void makeNewPhaseWithModelAndTree(const int& phaseNumber);
  QStandardItem* makeLabel(QString string);
  QStandardItem* makeValue(QVariant var);
  QList <QStandardItem*> addXYZLabel(QStandardItem* item);
  void playMotion(int phase);

  ViewBike& viewBike;

  std::string floatToStr(const float& f);
  std::string boolToStr(const bool& b);
  std::string intToStr(const int& i);

  //Editor
  TabWidget* tabber;
  QTreeView* treeViewCommon; //TreeView for the Common Parameters
  std::vector<QTreeView*> treeView; //TreeView for the PhaseParameters
  QStandardItemModel* modelCommon; //Model for the Common Parameters
  std::vector<QStandardItemModel*> phaseTab; //Model for the PhaseParameters
  QPushButton* addPhase, //add one phase after the actual phase
               * deletePhase; //delete actual phase
  QAction* lftra,
           * rftra,
           * lfrot,
           * rfrot,
           * lhtra,
           * rhtra,
           * lhrot,
           * rhrot;
  QString fileName; //the actual fileName
  BIKEParameters& parameters; //the actual Parameters

  //Drawings and openGl
  bool phaseDrawings, //show phaseDrawings
       singleDraw, //show only the drawings of the actual Phase
       reachedDraw,
       tra2dWindows, //show 2D-View of phaseDrawings for actual Phase and Limb
       tra1dWindows, //show 1D-View of phaseDrawings for actual Phase and Limb
       velocityWindows,//show one curves velocity
       accelWindows, //show one curves acceleration
       followMode;
  Vector3<> dragPlane; //plane for the 3D View where a selected point is moved
  Selected selectedPoint; //Infos about the actual selected Point

  int getString,
      ghost; //set the opacitiy of the robot model

  std::vector<std::string> commands;

  MotionRequest::Motion lastMotion;

  //Contextmenu
  BikeMenuBar* bikeMenuBar; //the contextMenubar
  QSignalMapper dragPlaneMapper, softenMapper;

  bool mirror;

  friend class ViewBikeGLWidget;
};
