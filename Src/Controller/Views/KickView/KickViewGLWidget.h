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
#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/Joints.h"

#include <QGLWidget>

class KickViewWidget;

class KickViewGLWidget : public QGLWidget
{
  Q_OBJECT

public:
  KickViewGLWidget(KickView& kickView, KickEngineParameters& parameters, KickViewWidget* parent);
  ~KickViewGLWidget();

private:
  KickViewWidget& widget;
  KickView& kickView;
  SimRobotCore2::Renderer& renderer;
  Vector3f targetPosOffset = Vector3f::Zero();
  Vector3f cameraPosOffset = Vector3f::Zero();
  RotationMatrix originRot;
  KickEngineParameters& parameters; //the actual Parameters
  std::vector<Vector3f> reachedPositions[Phase::numOfLimbs];

  bool moveDrag = false;  //a Point in the 3DView is moved or not
  bool moveViewOfViewDragXY = false; //a Point in the XY(t)View (2D) is moved or not
  bool moveViewOfViewDragXZ = false; //a Point in the XZ(t)View (2D)is moved or not
  bool moveViewOfViewDragYZ = false; //a Point in the YZ(t)View (2D)is moved or not
  bool moveViewOfViewDragXP = false; //a Point in the X:Phase View (1D) is moved or not
  bool moveViewOfViewDragYP = false; //a Point in the Y:Phase View (1D) is moved or not
  bool moveViewOfViewDragZP = false;  //a Point in the Z:Phase View (1D) is moved or not

  float actualX; //the captured mousePosition when contextMenu is open
  float actualY;

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
  void draw2dCurves(float minA, float minB, float maxA, float maxB, float scaleFactorA,
                    float scaleFactorB, float transA, float transB, float* colorA, float* colorB,
                    float window, const std::array<Vector2f, Phase::numOfPoints>& cp, const std::vector<Vector2f>& point);
  bool clickControlPoint(int x, int y);
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
