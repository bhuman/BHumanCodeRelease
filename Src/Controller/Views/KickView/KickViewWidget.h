/**
 * @file Controller/Views/KickView/KickViewWidget.h
 *
 * Declaration of class KickViewWidget
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include "KickView.h"
#include "KickViewGLWidget.h"
#include "TabWidget.h"
#include "KickMenuBar.h"
#include "Tools/Math/Eigen.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Representations/MotionControl/MotionRequest.h"

#include <QWidget>
#include <QTreeView>
#include <QStandardItem>
#include <QPushButton>
#include <QSignalMapper>
#include <QCheckBox>

class KickViewHeaderedWidget;

class Selected
{
public:
  int phaseNumber;
  unsigned int pointNumber;
  int limb;
  bool xzRot;
};

class KickViewWidget : public QWidget
{
  Q_OBJECT

public:
  KickViewWidget(KickView& kickView, KickEngineParameters& parameters, KickViewHeaderedWidget* parent);
  ~KickViewWidget();

  void updateEditorView();
  void fillModelWithPhaseData(int i);
  void updateCommon();
  void updateGL() { glWidget->update(); }
  void update() { QWidget::update(); glWidget->update(); }

  bool getDrawings() const {return phaseDrawings;}
  bool getSingleDrawing() const {return singleDraw;}
  bool getReachedDrawing() const {return reachedDraw;}
  bool getEditor() const {return !tabber->isHidden();}
  bool getTra2d() const {return tra2dWindows;}
  bool getTra1d() const {return tra1dWindows;}
  bool getVelocity() const {return velocityWindows;}
  bool getAccel() const {return accelWindows;}
  bool getFollowMode() const {return followMode;}

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
  void setMirrored(int state);
  void setArmsBackFix(int state);
  void recordPose();
  void setStiffness(int limb);
  //Sliders
  void transparencyChanged(int i);
  //Tab
  void updateCommonParameters(QStandardItem* item);
  void updatePhaseParameters(QStandardItem* item);
  void movePhase(int fromIndex, int toIndex);
  void setSelectedFromEditor(int index);
  //Contextmenu
  void setDragPlane(int plane);

private:
  KickViewHeaderedWidget* parent;
  KickViewGLWidget* glWidget;

  KickView& kickView;

  //Editor
  TabWidget* tabber;
  QTreeView* treeViewCommon; //TreeView for the Common Parameters
  std::vector<QTreeView*> treeView; //TreeView for the PhaseParameters
  QStandardItemModel* modelCommon; //Model for the Common Parameters
  std::vector<QStandardItemModel*> phaseTab; //Model for the PhaseParameters
  QPushButton* addPhase; //add one phase after the actual phase
  QPushButton* deletePhase; //delete actual phase
  QAction* lftra;
  QAction* rftra;
  QAction* lfrot;
  QAction* rfrot;
  QAction* lhtra;
  QAction* rhtra;
  QAction* lhrot;
  QAction* rhrot;
  QString fileName; //the actual fileName
  KickEngineParameters& parameters; //the actual Parameters

  //Drawings and openGl
  bool phaseDrawings = true; //show phaseDrawings
  bool singleDraw = false; //show only the drawings of the actual Phase
  bool reachedDraw = false;
  bool tra2dWindows = false; //show 2D-View of phaseDrawings for actual Phase and Limb
  bool tra1dWindows = false; //show 1D-View of phaseDrawings for actual Phase and Limb
  bool velocityWindows = false; //show one curves velocity
  bool accelWindows = false; //show one curves acceleration
  bool followMode = false;
  Vector3f dragPlane = Vector3f::Zero(); //plane for the 3D View where a selected point is moved
  Selected selectedPoint; //Infos about the actual selected Point

  int getString = 4;
  int ghost = 0; //set the opacity of the robot model

  std::vector<std::string> commands;

  MotionRequest::Motion lastMotion;

  //Contextmenu
  KickMenuBar* kickMenuBar; //the contextMenubar
  QSignalMapper dragPlaneMapper, softenMapper;

  bool mirror = false,
       armsBackFix = false;

  friend class KickViewGLWidget;

  //TreeView, StandardModel and Editor
  QList<QStandardItem*> makeStringAndValueRow(QString string, QVariant value);
  void addStringAndValueToList(QList<QStandardItem*>& list, QString string, QVariant value);
  QStandardItemModel* makeNewModelWithLabels();

  bool deleteKids(QStandardItem* rootItem);
  void addControlPoints(QStandardItem* item);
  void makeNewPhaseWithModelAndTree(int phaseNumber);
  QStandardItem* makeLabel(QString string);
  QStandardItem* makeValue(QVariant var);
  QList<QStandardItem*> addXYZLabel(QStandardItem* item);
  void playMotion(int phase);

  std::string floatToStr(float f);
  std::string boolToStr(bool b);
};
