#pragma once

#include "Tools/Math/Approx.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Random.h"
#include "Tools/Range.h"
#ifdef IS_TESTED
#include "gtest/gtest.h"
#endif

namespace Optimizer
{
  enum class Convergence
  {
    notConverged,
    localMinimum,
    converged
  };

  template<typename ResultType>
  struct Result
  {
    ResultType best;
    Convergence convergence;
    unsigned iterations;
  };
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
class DownhillSimplexOptimizer
{
public:
  using Vector = Eigen::Matrix<T, Dimensions, 1>;

private:
  struct Data
  {
    std::array<T, Dimensions> data;

    Data() = default;
    Data(const Vector& other);

    Data operator+(const Data& other) const;
    Data operator-(const Data& other) const;
    Data operator*(T scalar) const;

    Data& operator+=(const Data& other);
    Data& operator/=(T scalar);

    T& operator[](size_t index) { return data[index]; };
    const T& operator[](size_t index) const { return data[index]; };

    Vector toVector() const { return Eigen::Map<const Vector>(data.data()); }

    static Data Zero() { Data res; res.data.fill(T(0)); return res; }
  };

  class Point
  {
  private:
    const DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>* ds;

  public:
    Data values;
    T error;

    Point() = default;
    Point(const DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>* ds) : ds(ds) {}

    bool operator<(const Point& other) const { return error < other.error; }

    void operator=(const Data& vals);
  };

  std::array<Point, Dimensions + 1> simplex;

  ErrorFunction& function;
  const std::array<Range<T>, Dimensions>& ranges;

  const T alpha = 1;
  const T gamma = 2;
  const T rho = -0.5;
  const T sigma = 0.5;
#ifdef IS_TESTED
  FRIEND_TEST(DownhillSimplex, bench1);
  FRIEND_TEST(DownhillSimplex, bench2);
#endif

public:
  DownhillSimplexOptimizer(ErrorFunction& function, const std::array<Range<T>, Dimensions>& ranges);

  void initRandom(const Vector& start);
  void initDelta(const Vector& start, const Vector& delta);

  Optimizer::Result<Vector> optimize(T terminationCriterion, unsigned maxIterations);

private:
  friend Point;
  Data limit(const Data& vals) const;
  T applyErrorFunction(const Data& vals) const;
};

template<typename T, unsigned Dimensions, typename ErrorFunction>
DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data::Data(const Vector& other)
{
  for(unsigned i = 0; i < Dimensions; ++i)
    data[i] = other(i);
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data::operator+(const Data& other) const
{
  Data res;
  for(unsigned i = 0; i < Dimensions; ++i)
    res[i] = data[i] + other[i];
  return res;
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data::operator-(const Data& other) const
{
  Data res;
  for(unsigned i = 0; i < Dimensions; ++i)
    res[i] = data[i] - other[i];
  return res;
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data::operator*(T scalar) const
{
  Data res;
  for(unsigned i = 0; i < Dimensions; ++i)
    res[i] = data[i] * scalar;
  return res;
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data& DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data::operator+=(const Data& other)
{
  for(unsigned i = 0; i < Dimensions; ++i)
    data[i] += other[i];
  return *this;
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data& DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data::operator/=(T scalar)
{
  for(unsigned i = 0; i < Dimensions; ++i)
    data[i] /= scalar;
  return *this;
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
inline void DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Point::operator=(const Data& vals)
{
  values = ds->limit(vals);
  error = ds->applyErrorFunction(values);
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::DownhillSimplexOptimizer(ErrorFunction& function, const std::array<Range<T>, Dimensions>& ranges) :
  function(function), ranges(ranges)
{
  simplex.fill(Point(this));
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
void DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::initRandom(const Vector& start)
{
  simplex[0] = start;
  for(size_t i = 1; i < simplex.size(); ++i)
  {
    Point& point = simplex[i];
    Data& vals = point.values;
    for(unsigned j = 0; j < Dimensions; ++j)
    {
      vals[j] = Random::uniform(ranges[j].min, ranges[j].max);
    }
    point.error = applyErrorFunction(vals);
  }
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
void DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::initDelta(const Vector& start, const Vector& delta)
{
  simplex[0] = start;
  for(size_t i = 1; i < simplex.size(); ++i)
  {
    Vector values = start;
    values(i - 1) += delta(i - 1) * (Random::bernoulli() ? 1.f : -1.f);
    simplex[i] = values;
  }
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
typename Optimizer::Result<typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Vector>
DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::optimize(T terminationCriterion, unsigned maxIterations)
{
  unsigned i = 0;
  Optimizer::Convergence conv = Optimizer::Convergence::notConverged;
  for(; i < maxIterations; ++i)
  {
    std::sort(simplex.begin(), simplex.end());
    const Point& best = simplex[0];
    const Point& secondWorst = simplex[simplex.size() - 2];
    Point& worst = simplex[simplex.size() - 1];

    // check termination
    if(best.error < terminationCriterion)
    {
      conv = Optimizer::Convergence::converged;
      break;
    }

    Data midSum = Data::Zero();
    for(unsigned i = 1; i < Dimensions; ++i)
      midSum += simplex[i].values;

    // check for local minimum
    const Data simplexExtent = best.values - ((midSum + worst.values) /= Dimensions);
    T norm = T(0);
    for(unsigned i = 0; i < Dimensions; ++i)
      norm = simplexExtent[i] * simplexExtent[i];
    norm = std::sqrt(norm);
    if(norm < 0.0001f)
    {
      conv = Optimizer::Convergence::localMinimum;
      break;
    }

    const Data centroid = (best.values + midSum) /= Dimensions;

    // reflection
    Point reflectionPoint(this);
    reflectionPoint = centroid + (centroid - worst.values) * alpha;
    if(best.error <= reflectionPoint.error && reflectionPoint.error < secondWorst.error)
    {
      worst = reflectionPoint;
      continue;
    }

    // expansion
    if(reflectionPoint.error < best.error)
    {
      Point expansionPoint(this);
      expansionPoint = centroid + (centroid - worst.values) * gamma;
      if(expansionPoint.error < reflectionPoint.error)
        worst = expansionPoint;
      else
        worst = reflectionPoint;
      continue;
    }

    // contraction
    Point contractionPoint(this);
    contractionPoint = centroid + (centroid - worst.values) * rho;
    if(contractionPoint.error < worst.error)
    {
      worst = contractionPoint;
      continue;
    }

    // reduction
    const Data& bestVal = simplex[0].values;
    for(size_t i = 1; i < simplex.size(); ++i)
      simplex[i] = bestVal + (simplex[i].values - bestVal) * sigma;
  }

  return Optimizer::Result<Vector>{simplex[0].values.toVector(), conv, i};
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
inline typename DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::Data DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::limit(const Data& vals) const
{
  Data values;
  for(unsigned i = 0; i < Dimensions; ++i)
    values[i] = ranges[i].limit(vals[i]);
  return values;
}

template<typename T, unsigned Dimensions, typename ErrorFunction>
T DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>::applyErrorFunction(const Data& vals) const
{
  return std::abs(function(vals.toVector()));
}

namespace Optimizer
{
  template<typename T, std::size_t Dimensions, typename ErrorFunction>
  DownhillSimplexOptimizer<T, Dimensions, ErrorFunction> makeDownhillSimplexOptimizer(ErrorFunction& function, const std::array<Range<T>, Dimensions>& ranges)
  {
    return DownhillSimplexOptimizer<T, Dimensions, ErrorFunction>(function, ranges);
  }
}

template<typename T, typename ErrorFunction>
class DownhillSimplexOptimizer1
{
private:
  class Point
  {
    const DownhillSimplexOptimizer1* ds;
  public:
    T value;
    T error;

    Point() = default;
    Point(const DownhillSimplexOptimizer1* ds) : ds(ds) {}

    bool operator<(const Point& other) const { return error < other.error; }

    void operator=(const T val);
  };

  std::array<Point, 2> simplex;

  ErrorFunction& function;
  const Range<T>& range;

  const T alpha = 1;
  const T gamma = 2;
  const T ro = -0.5f;
  const T sigma = 0.5f;

public:
  DownhillSimplexOptimizer1(ErrorFunction& function, const Range<T>& valueRange);

  void initRandom(const T start);
  void initDelta(const T start, const T delta);

  Optimizer::Result<T> optimize(T terminationCriterion, unsigned maxIterations);

private:
  void sort();
  T applyErrorFunction(const T val) const;
#ifdef IS_TESTED
  FRIEND_TEST(DownhillSimplex, bench1);
#endif
};

template<typename T, typename ErrorFunction>
inline void DownhillSimplexOptimizer1<T, ErrorFunction>::Point::operator=(const T val)
{
  value = ds->range.limit(val);
  error = ds->applyErrorFunction(value);
}

template<typename T, typename ErrorFunction>
DownhillSimplexOptimizer1<T, ErrorFunction>::DownhillSimplexOptimizer1(ErrorFunction& function, const Range<T>& range) :
  function(function), range(range)
{
  simplex.fill(Point(this));
}

template<typename T, typename ErrorFunction>
void DownhillSimplexOptimizer1<T, ErrorFunction>::initRandom(const T start)
{
  simplex[0] = start;
  simplex[1] = Random::uniform(range.min, range.max);
}

template<typename T, typename ErrorFunction>
void DownhillSimplexOptimizer1<T, ErrorFunction>::initDelta(const T start, const T delta)
{
  simplex[0] = start;
  simplex[1] = delta * (Random::bernoulli() ? 1.f : -1.f);
}

template<typename T, typename ErrorFunction>
typename Optimizer::Result<T> DownhillSimplexOptimizer1<T, ErrorFunction>::optimize(T terminationCriterion, unsigned maxIterations)
{
  unsigned i = 0;
  Optimizer::Convergence conv = Optimizer::Convergence::notConverged;
  for(; i < maxIterations; ++i)
  {
    sort();
    const Point& best = simplex[0];
    Point& worst = simplex[1];

    // check termination
    if(best.error < terminationCriterion)
    {
      conv = Optimizer::Convergence::converged;
      break;
    }
    else if(Approx::isEqual(best.value, worst.value))
    {
      conv = Optimizer::Convergence::localMinimum;
      break;
    }

    const T centroid = best.value;

    // reflection
    Point reflectionPoint(this);
    reflectionPoint = centroid + alpha * (centroid - worst.value);
    if(best.error <= reflectionPoint.error && reflectionPoint.error < worst.error)
    {
      worst = reflectionPoint;
      continue;
    }

    // expansion
    if(reflectionPoint.error < best.error)
    {
      Point expansionPoint(this);
      expansionPoint = centroid + gamma * (centroid - worst.value);
      if(expansionPoint.error < reflectionPoint.error)
        worst = expansionPoint;
      else
        worst = reflectionPoint;
      continue;
    }

    // contraction
    Point contractionPoint(this);
    contractionPoint = centroid + ro * (centroid - worst.value);
    if(contractionPoint.error < worst.error)
    {
      worst = contractionPoint;
      continue;
    }

    // reduction
    const T bestVal = simplex[0].value;
    simplex[1] = bestVal + sigma * (simplex[1].value - bestVal);
  }
  
  return Optimizer::Result<T>{simplex[0].value, conv, i};
}

template<typename T, typename ErrorFunction>
inline void DownhillSimplexOptimizer1<T, ErrorFunction>::sort()
{
  if(simplex[1] < simplex[0])
    std::swap(simplex[0], simplex[1]);
}

template<typename T, typename ErrorFunction>
T DownhillSimplexOptimizer1<T, ErrorFunction>::applyErrorFunction(const T val) const
{
  return std::abs(function(val));
}

namespace Optimizer
{
  template<typename T, typename ErrorFunction>
  DownhillSimplexOptimizer1<T, ErrorFunction> makeDownhillSimplexOptimizer1(ErrorFunction& function, const Range<T>& valueRange)
  {
    return DownhillSimplexOptimizer1<T, ErrorFunction>(function, valueRange);
  }
}