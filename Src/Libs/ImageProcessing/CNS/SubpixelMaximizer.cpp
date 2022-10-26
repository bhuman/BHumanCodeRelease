#include "SubpixelMaximizer.h"
#include <limits>
#include <cstring>
#include <cmath>
#include <cassert>

SubpixelMaximizer::FitMatrix SubpixelMaximizer::fitMatrix;

void SubpixelMaximizer::max(float& value, float arg[3], const signed short data[3][3][3]) const
{
  float coef[10];
  fitUsingSSE3(coef, data);
  max(value, arg, coef);
}

SubpixelMaximizer::FitMatrix::FitMatrix()
{
  int rawData[10][27] =
  {
    { -8,   4, -8,   4,  16,   4, -8,   4, -8,   4,  16,   4,  16,  28,  16,   4,  16,   4, -8,   4, -8,   4,  16,   4, -8,   4, -8},
    { -6,  -6, -6,  -6,  -6,  -6, -6,  -6, -6,   0,   0,   0,   0,   0,   0,   0,   0,   0,  6,   6,  6,   6,   6,   6,  6,   6,  6},
    { -6,  -6, -6,   0,   0,   0,  6,   6,  6,  -6,  -6,  -6,   0,   0,   0,   6,   6,   6, -6,  -6, -6,   0,   0,   0,  6,   6,  6},
    { -6,   0,  6,  -6,   0,   6, -6,   0,  6,  -6,   0,   6,  -6,   0,   6,  -6,   0,   6, -6,   0,  6,  -6,   0,   6, -6,   0,  6},
    {  6,   6,  6,   6,   6,   6,  6,   6,  6, -12, -12, -12, -12, -12, -12, -12, -12, -12,  6,   6,  6,   6,   6,   6,  6,   6,  6},
    {  6,   6,  6, -12, -12, -12,  6,   6,  6,   6,   6,   6, -12, -12, -12,   6,   6,   6,  6,   6,  6, -12, -12, -12,  6,   6,  6},
    {  6, -12,  6,   6, -12,   6,  6, -12,  6,   6, -12,   6,   6, -12,   6,   6, -12,   6,  6, -12,  6,   6, -12,   6,  6, -12,  6},
    {  9,   9,  9,   0,   0,   0, -9,  -9, -9,   0,   0,   0,   0,   0,   0,   0,   0,   0, -9,  -9, -9,   0,   0,   0,  9,   9,  9},
    {  9,   0, -9,   0,   0,   0, -9,   0,  9,   9,   0,  -9,   0,   0,   0,  -9,   0,   9,  9,   0, -9,   0,   0,   0, -9,   0,  9},
    {  9,   0, -9,   9,   0,  -9,  9,   0, -9,   0,   0,   0,   0,   0,   0,   0,   0,   0, -9,   0,  9,  -9,   0,   9, -9,   0,  9}
  };
  scale = 1.0f / 108.f; // rawData must be scaled by 1/108 to give true matrix
  memset(data, 0, sizeof(data));
  for(int i = 0; i < 10; i++)
    for(int j = 0; j < 27; j++)
      reinterpret_cast<short*>(&data[i][0])[j] = static_cast<short>(rawData[i][j]);
  assert(aligned16(data));
}

void SubpixelMaximizer::fitUsingSSE3(float coef[FitMatrix::ROWS], const signed short data[3][3][3]) const
{
  assert(FitMatrix::PADDEDCOLS == 32);
  __m128 localFitMatrixScale = _mm_set_ss(fitMatrix.scale);
  const short* localFitMatrix = fitMatrix();
  // Load data into four SSE Registers
  __m128i x[4];
  signed short* dataFlat = const_cast<signed short*>(reinterpret_cast<const signed short*>(data)); // flat array of 27 signed shorts
  x[0] = _mm_loadu_si128((__m128i*)(dataFlat + 0));
  x[1] = _mm_loadu_si128((__m128i*)(dataFlat + 8));
  x[2] = _mm_loadu_si128((__m128i*)(dataFlat + 16));
  x[3] = _mm_loadu_si128((__m128i*)(dataFlat + 24));
  x[3] = _mm_srli_si128(_mm_slli_si128(x[3], 10), 10);   // Clear dataFlat[27..31]

  for(int i = 0; i < FitMatrix::ROWS; i++)
  {
    // Compute scalar product between ((float*)x)[0..31] and localFitMatrix
    __m128i sum =             _mm_madd_epi16(x[0], *(__m128i*)(localFitMatrix + 0));
    sum = _mm_add_epi32(sum, _mm_madd_epi16(x[1], *(__m128i*)(localFitMatrix + 8)));
    sum = _mm_add_epi32(sum, _mm_madd_epi16(x[2], *(__m128i*)(localFitMatrix + 16)));
    sum = _mm_add_epi32(sum, _mm_madd_epi16(x[3], *(__m128i*)(localFitMatrix + 24)));
    sum = _mm_hadd_epi32(sum, sum);
    sum = _mm_hadd_epi32(sum, sum);
    _mm_store_ss(coef + i, _mm_mul_ss(_mm_cvtepi32_ps(sum), localFitMatrixScale));
    localFitMatrix += 32;
  }
}

void SubpixelMaximizer::fitUsingC(float coef[10], const signed short data[3][3][3]) const
{
  for(int i = 0; i < 10; i++)
  {
    int sum = 0;
    int j = 0;
    for(int j1 = -1; j1 <= 1; j1++)
      for(int j2 = -1; j2 <= 1; j2++)
        for(int j3 = -1; j3 <= 1; j3++)
        {
          sum += int(fitMatrix(i, j) * data[1 + j1][1 + j2][1 + j3]);
          j++;
        }
    coef[i] = static_cast<float>(sum);
  }
}

void SubpixelMaximizer::fitUsingC(float coef[10], const float data[3][3][3]) const
{
  for(int i = 0; i < 10; i++)
  {
    float sum = 0;
    int j = 0;
    for(int j1 = -1; j1 <= 1; j1++)
      for(int j2 = -1; j2 <= 1; j2++)
        for(int j3 = -1; j3 <= 1; j3++)
        {
          sum += fitMatrix(i, j) * data[1 + j1][1 + j2][1 + j3];
          j++;
        }
    coef[i] = sum;
  }
}

float SubpixelMaximizer::eval(const float coef[10], const float arg[3])
{
  float x = arg[0], y = arg[1], z = arg[2];
  return coef[0]
         + coef[1] * x + coef[2] * y + coef[3] * z
         + coef[4] * x * x + coef[5] * y * y + coef[6] * z * z + coef[7] * x * y + coef[8] * y * z + coef[9] * x * z;
}

void SubpixelMaximizer::max(float& value, float arg[3], const float coef[10]) const
{
  // Derivative of x^TAx+b^Tx+c is 0=2Ax+b ==> -Ax=b/2
  // For a maximum, -A will be SPD.

  float A[3][3];  // extract matrix of 2nd order, make negative, so A is SPD
  A[0][0] = -coef[4];
  A[1][1] = -coef[5];
  A[2][2] = -coef[6];
  A[0][1] = A[1][0] = -coef[7] / 2;
  A[1][2] = A[2][1] = -coef[8] / 2;
  A[0][2] = A[2][0] = -coef[9] / 2;

  float b[3];  // extract first order vector, also negate
  b[0] = coef[1] / 2;
  b[1] = coef[2] / 2;
  b[2] = coef[3] / 2;

  float L[3][3];
  float y[3];
  if(cholesky(A, L))
  {
    solveLxb(L, y, b);
    solveLTxb(L, arg, y);
    arg[0] = clip(arg[0], -1, +1);
    arg[1] = clip(arg[1], -1, +1);
    arg[2] = clip(arg[2], -1, +1);
    value = eval(coef, arg);
  }
  else
  {
    value = std::numeric_limits<float>::infinity();
    arg[0] = arg[1] = arg[2] = std::numeric_limits<float>::quiet_NaN();
  }
}

bool SubpixelMaximizer::cholesky(const float A[3][3], float L[3][3])
{
  float sum;
  sum = A[0][0];
  if(sum <= 0)
    return false;
  if(sum > 0)
  {
    // (0,0)
    L[0][0] = std::sqrt(sum);

    // (1,0)
    sum = A[1][0];
    L[1][0] = sum / L[0][0];

    // (2,0)
    sum = A[2][0];
    L[2][0] = sum / L[0][0];
  }
  else
    L[0][0] = L[1][0] = L[2][0] = 0;

  // (0,1)
  L[0][1] = 0;

  sum = A[1][1] - L[1][0] * L[1][0];
  if(sum <= 0)
    return false;
  if(sum > 0)
  {
    // (1,1)
    L[1][1] = std::sqrt(sum);

    // (2,1)
    sum = A[2][1] - L[1][0] * L[2][0];
    L[2][1] = sum / L[1][1];
  }
  else
    L[1][1] = L[2][1] = 0;

  // (0,2)
  L[0][2] = 0;

  // (1,2)
  L[1][2] = 0;

  sum = A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1];
  if(sum <= 0)
    return false;
  if(sum > 0)
  {
    // (2,2)
    L[2][2] = std::sqrt(sum);
  }
  else
    L[2][2] = 0;

  return true;
}

void SubpixelMaximizer::solveLxb(const float L[3][3], float x[3], const float b[3])
{
  x[0] = b[0] / L[0][0];
  x[1] = (b[1] - x[0] * L[1][0]) / L[1][1];
  x[2] = (b[2] - x[0] * L[2][0] - x[1] * L[2][1]) / L[2][2];
}

void SubpixelMaximizer::solveLTxb(const float L[3][3], float x[3], const float b[3])
{
  x[2] = b[2] / L[2][2];
  x[1] = (b[1] - x[2] * L[2][1]) / L[1][1];
  x[0] = (b[0] - x[2] * L[2][0] - x[1] * L[1][0]) / L[0][0];
}
