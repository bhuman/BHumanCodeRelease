/**
 * @file WalkingEngineTools.h
 * Declaration of tools utilized by the WalkingEngine
 * @author Colin Graf
 */

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(VectorYZ,
{
  VectorYZ() = default;
  VectorYZ(float y, float z),

  (float)(0) y,
  (float)(0) z,
});

/**
* An one-dimensional downhill simplex optimizer
*/
class FunctionMinimizer
{
public:
  virtual float func(float pos) const = 0;
  float minimize(float minPos, float maxPos, float startPos, float startPosDelta, float minVal, bool& clipped, const char* debugstr) const;
};
