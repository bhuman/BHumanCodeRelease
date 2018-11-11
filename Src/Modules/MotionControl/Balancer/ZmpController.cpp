#include "ZmpController.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Motion/OptimalControl.h"

bool ZmpControllerParameters::operator==(const ZmpControllerParameters& other) const
{
  return (numOfZmpPreviews == other.numOfZmpPreviews
          && R == other.R
          && Qe == other.Qe
          && Qx == other.Qx
          && useIntegrator == other.useIntegrator);
}

bool ZmpControllerParameters::operator!=(const ZmpControllerParameters& other) const
{
  return !operator==(other);
}
ZmpController::ZmpController()
{}

void ZmpController::init(float cycleTime)
{
  const float gtt_2h = Constants::g * cycleTime * cycleTime / (2.f * comHeight);
  const float gt_h = Constants::g * cycleTime / comHeight;
  A << 1.f + gtt_2h, cycleTime, -gtt_2h,
  gt_h, 1, -gt_h,
  gtt_2h, cycleTime, 1.f - gtt_2h;

  const float ttt_6 = cycleTime * cycleTime * cycleTime / 6.f;
  B << ttt_6, 0.5f * cycleTime* cycleTime, ttt_6 - comHeight* cycleTime / Constants::g;

  C << 0.f, 0.f, 1.f;

  calculateGains();
}

void ZmpController::reset(float comHeight, float cycleTime, ZmpControllerParameters params)
{
  this->params = params;
  this->lastParams = params;
  this->comHeight = comHeight;
  init(cycleTime);
  integrationError = 0.f;
}

Vector3f ZmpController::control(const Vector3f& inputState, const VectorXf& Zmps, float comHeight, ZmpControllerParameters params)
{
  this->lastParams = this->params;
  this->params = params;

  ASSERT(static_cast<unsigned>(Zmps.size()) == params.numOfZmpPreviews);
  if(this->comHeight != comHeight || params != lastParams)
  {
    this->comHeight = comHeight;
    lastParams = params;
    calculateGains();
  }

  if(operational)
  {
    const float error = C* inputState - Zmps(0);

    if(params.useIntegrator)
      integrationError = integrationError + error; //integrationError * 0.5f + error;
    else
      integrationError = error; // correct? or just 0?

    const float u = -(GI * integrationError) - (Gx* inputState) - (Gd* Zmps);

    return A* inputState + B* u;
  }
  else
    return Vector3f::Zero();
}

void ZmpController::calculateGains()
{
  DECLARE_PLOT("module:ZmpWalkingEngine:ZmpController:previewGains");
  ASSERT(params.numOfZmpPreviews > 0);

  const Vector4f B = (Vector4f() << this->C* this->B, this->B).finished();
  const Vector4f I = Vector4f::Identity();
  const Matrix4x3f F = (Matrix4x3f() << this->C* this->A, this->A).finished();
  const Matrix4f A = (Matrix4f() << I, F).finished();
  const Matrix4f Q = (Matrix4f() << params.Qe, RowVector3f::Zero(),
                      Vector3f::Zero(), Matrix3f(params.Qx.asDiagonal())).finished();

  Matrix4f K;
  operational = OptimalControl::dare<4, 1>(A, B, Q, Eigen::Matrix<float, 1, 1>(params.R), K);
  if(operational)
  {
    const RowVector4f Bt = B.transpose();
    const RowVector4f BtK = Bt* K;
    const float rBtKBinv = 1.f / (params.R + BtK* B);

    GI = rBtKBinv * BtK* I;
    Gx = rBtKBinv * BtK* F;

    const Matrix4f Act = (A - B* rBtKBinv * BtK* A).transpose();
    const Vector4f KI = K* I;

    Gd.resize(params.numOfZmpPreviews);
    Gd(0) = 0.f;
    Gd(1) = -GI;
    Vector4f X = -Act* KI;
    for(unsigned i = 2; i < params.numOfZmpPreviews; ++i)
    {
      Gd(i) = rBtKBinv * Bt* X;
      X = Act* X;
    }
  }
  else
  {
    GI = 0.f;
    Gx = RowVector3f::Zero();
    Gd = RowVectorXf::Zero(params.numOfZmpPreviews);
  }

  for(int i = 0; i < Gd.size(); ++i)
  {
    PLOT("module:ZmpWalkingEngine:ZmpController:previewGains", Gd(i));
  }
}

void ZmpController::plot() const
{
  DECLARE_PLOT("module:ZmpWalkingEngine:ZmpController:previewGains");

  DEBUG_RESPONSE_ONCE("module:ZmpWalkingEngine:ZmpController:plotPreviewGains")
  {
    for(int i = 0; i < Gd.size(); ++i)
    {
      PLOT("module:ZmpWalkingEngine:ZmpController:previewGains", Gd(i));
    }
  }
}
