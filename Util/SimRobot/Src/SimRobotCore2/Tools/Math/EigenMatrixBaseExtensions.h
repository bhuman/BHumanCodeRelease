//Do NOT directly include this file, instead include "Tools/Math/Eigen.h"

inline PlainObject& normalize(RealScalar l)
{
  const RealScalar vl = norm();
  if(vl > 1e-9)
  {
    derived() *= (l / vl);
  }
  return derived();
}

inline PlainObject normalized(RealScalar l) const
{
  return PlainObject(derived()).normalize(l);
}
