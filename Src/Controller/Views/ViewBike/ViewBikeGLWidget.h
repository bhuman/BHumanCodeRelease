/**
 * @file Controller/Views/ViewBike/ViewBikeGLWidget.h
 *
 * Declaration of class ViewBikeGLWidget
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include <QGLWidget>

#include "ViewBike.h"
#include "Tools/Math/RotationMatrix.h"
#include "Modules/MotionControl/BIKEParameters.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/MotionRequest.h"

class ViewBikeWidget;

class ViewBikeGLWidget : public QGLWidget
{
  Q_OBJECT

public:
  ViewBikeGLWidget(ViewBike& viewBike, BIKEParameters& parameters, ViewBikeWidget* parent);
  virtual ~ViewBikeGLWidget();

private:
  ViewBikeWidget& widget;
  ViewBike& viewBike;
  SimRobotCore2::Renderer& renderer;
  Vector3<> targetPosOffset,
          cameraPosOffset;
  RotationMatrix originRot;
  BIKEParameters& parameters; //the actual Parameters
  std::vector<Vector3<> > reachedPositions[Phase::numOfLimbs];

  bool moveDrag,  //a Point in the 3DView is moved or not
       moveViewOfViewDragXY, //a Point in the XY(t)View (2D) is moved or not
       moveViewOfViewDragXZ, //a Point in the XZ(t)View (2D)is moved or not
       moveViewOfViewDragYZ, //a Point in the YZ(t)View (2D)is moved or not
       moveViewOfViewDragXP, //a Point in the X:Phase View (1D) is moved or not
       moveViewOfViewDragYP, //a Point in the Y:Phase View (1D) is moved or not
       moveViewOfViewDragZP;  //a Point in the Z:Phase View (1D) is moved or not

  float actualX, //the captured mousePosition when contextMenu is open
        actualY;

  GLubyte stippleMask[17][128]; //The opacity mask

  void initializeGL();
  void resizeGL(int newWidth, int newHeight);
  void paintGL();
  GLvoid setMatrix(const float* translation, const RotationMatrix& rotation);
  void drawPhases();
  void drawBezierCurves(const int& phaseNumber);
  void calculateControlPointRot(const Vector3 <>& point0, const Vector3<>& point1, const Vector3<> point2, Vector3<>& rotation);
  void drawArrow(const Vector3 <>& point, const Vector3<>& rotation);
  void drawControlPoint(const Vector3 <>& point, const float& cubeFaktor);
  void draw2dCurves(const float& minA, const float& minB, const float& maxA, const float& maxB, const float& scaleFactorA,
                    const float& scaleFactorB, const float& transA, const float& transB, float* colorA, float* colorB,
                    const float& window, const std::vector<Vector2<> >& cp, const std::vector<Vector2<> >& point);
  bool clickControlPoint(const int& x, const int& y);
  void gluUnProjectClick(int x, int y, Vector3<>& vecFar, Vector3<>& vecNear);

  void setSteadyDiff();
  void showPlane();
  void clipCurve(Vector3<>& translationVec);
  bool clipLegJointsWithLimits(float& leg1, float& leg2, float& leg3, const JointData::Joint& joint);

  //QT-Widget-Events
  void keyPressEvent(QKeyEvent* event);
  void wheelEvent(QWheelEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void contextMenuEvent(QContextMenuEvent* event);
};
