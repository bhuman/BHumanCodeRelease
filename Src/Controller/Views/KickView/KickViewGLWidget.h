/**
 * @file Controller/Views/KickView/KickViewGLWidget.h
 *
 * Declaration of class KickViewGLWidget
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include "KickView.h"
#include "Tools/Math/RotationMatrix.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Joints.h"
#include "Tools/Math/Eigen.h"

#include <QGLWidget>

class KickViewWidget;

class KickViewGLWidget : public QGLWidget
{
  Q_OBJECT

public:
  KickViewGLWidget(KickView& kickView, KickEngineParameters& parameters, KickViewWidget* parent);
  virtual ~KickViewGLWidget();

private:
  KickViewWidget& widget;
  KickView& kickView;
  SimRobotCore2::Renderer& renderer;
  Vector3f targetPosOffset = Vector3f::Zero(),
           cameraPosOffset = Vector3f::Zero();
  RotationMatrix originRot;
  KickEngineParameters& parameters; //the actual Parameters
  std::vector<Vector3f> reachedPositions[Phase::numOfLimbs];

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
  void calculateControlPointRot(const Vector3f& point0, const Vector3f& point1, const Vector3f& point2, Vector3f& rotation);
  void drawArrow(const Vector3f& point, const Vector3f& rotation);
  void drawControlPoint(const Vector3f& point, const float& cubeFaktor);
  void draw2dCurves(const float& minA, const float& minB, const float& maxA, const float& maxB, const float& scaleFactorA,
                    const float& scaleFactorB, const float& transA, const float& transB, float* colorA, float* colorB,
                    const float& window, const std::vector<Vector2f>& cp, const std::vector<Vector2f>& point);
  bool clickControlPoint(const int& x, const int& y);
  void gluUnProjectClick(int x, int y, Vector3f& vecFar, Vector3f& vecNear);

  void setSteadyDiff();
  void showPlane();
  void clipCurve(Vector3f& translationVec);
  bool clipLegJointsWithLimits(float& leg1, float& leg2, float& leg3, const Joints::Joint& joint);

  //QT-Widget-Events
  void keyPressEvent(QKeyEvent* event);
  bool event(QEvent* event);
  void wheelEvent(QWheelEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void contextMenuEvent(QContextMenuEvent* event);
};
