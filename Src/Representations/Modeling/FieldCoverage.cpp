/**
 * FieldCoverage.cpp
 *
 * Contains implementations of FieldCoverage and FieldCoverage grid methods.
 *
 * Author: Felix Wenk
 */

#include "Platform/BHAssert.h"
#include "FieldCoverage.h"

FieldCoverage::Target::Target(float x, float y, unsigned short coverage, bool valid)
: target(x, y), coverage(coverage), isValid(valid) {}

FieldCoverage::Target::Target(const Vector2<>& target, unsigned short coverage, bool valid)
: target(target), coverage(coverage), isValid(valid) {}

unsigned char FieldCoverage::coverage(int cellIdx, unsigned time) const
{
  int sub = (time - cells[cellIdx]) / GridInterval::tick;
  ASSERT(sub >= 0);
  return sub >= GridInterval::maxCoverage ? 0 : static_cast<unsigned char>(GridInterval::maxCoverage - sub);
}
