/*! \file subpixelMaximizer.h
    Implementation of \c SubpixelMaximizer.
    \author Udo Frese
    \author Christoph Hertzberg
 */

#pragma once

#include "Tools/ImageProcessing/SIMD.h"

//! The algorithm for finding a subpixel maximum of an 3*3*3 area of a 3D function
/*! See \c max for usage. Actually it could be a simple function, but the algorithms
    needs to precompute a 27*10 matrix, which is done in the constructor and should
    be reused.
 */
class SubpixelMaximizer
{
public:
  //! Computes a subpixel maximum from a 3*3*3 window around the discrete maximum \c data[1][1][1]
  /*! The value of the maximum is returned in \c value, the
      coordinates in \c arg, where 0 corresponds to the center \c
      data[1][1][1]. This must be the discrete maximum.

      The function works by fitting an order 2 function to \c data and
      finding the max of that function. If either the order 2 function
      has no maximum, i.e. an indefinite Hessian a value of -infinity
      and an arg of NaN is returned.

      At the moment, if the arg max is outside [-1..+1]^3, it is
      simply clipped. This is not 100% correct and should be later
      replace by computing the maximum on [-1..+1]^3.
   */
  void max(float& value, float arg[3], const signed short data[3][3][3]) const;

protected:
  //! Matrix mapping from the 27 data entries to the coefficient of the fitted quadratic function
  /*! The matrix is 10*27 extended with a 0 columns to
      10*32. Multiplied with a vector of data as in \c max, and then by \c fitMatrixScale
      the result is the list of coefficients \f$ c, b_1, b_2, b_3, a_1, \ldots, a_6 \f$

      of the function f

      \f[
           f =   c
               + b1*x + b2*y + b3*z
               + a1*x*x + a2*y*y + a3*z*z + a4*x*y + a5*y*z + a6*x*z
      \f].

      Fitted. Each entry is represented as a signed short (16bit).

      It is necessary to make a helper class out of it statically copy our fixed matrix into a properly
      scaled, 0-padded and aligned representation  */
  class FitMatrix
  {
  public:
    //! Build up the matrix from constant values
    FitMatrix();

    //! Returns a pointer to the numerical data
    /*! The result points to the row 0 col 0 and each row is aligned with 16 bytes.
      The shorts must be multiplied by \c scale to obtain the floating value that
      is actually meant. See \c operator (i, j).
     */
    const short* operator()() const {return reinterpret_cast<const short*>(data);};

    //! Logical access operator to element \c (i,j) of the represented matrix
    float operator()(int i, int j) const {return scale * reinterpret_cast<const short*>(data)[i * PADDEDCOLS + j];}

    //! The short coefficient need to be multiplied with \c scale to get the exact results
    /*! This allows to do the multiplications and additions in short and
      then convert and scale the sums in float.
     */
    float scale;

    //! Matrix format
    /*! \c ROWS and \c COLS is the logical format. To align rows with 16 bytes boundaries
      the matrix is 0-padded to have \c PADDEDCOLS columns.
     */
    enum {ROWS = 10, COLS = 27, PADDEDCOLS = 32};

  protected:
    //! The actual data
    /*! Aligned to 16byte boundary and padded with 0 entries to 8 word, i.e. 16 bytes rows */
    __m128i data[ROWS][PADDEDCOLS / 8];
  };

  //! The constant matrix describing the fit of a quadratic polynomial to 3*3*3 data.
  /*! See \c FitMatrix. */
  static FitMatrix fitMatrix;

  //! Fits a 2nd order function to \c data
  /*! \c data as a flat 27 vector is multiplied with by multiplying
      with \c fitMatrix and \c fitMatrixScale.  The order of the
      coefficients is documented in \c fitMatrix.

      For technical reasons, the function ignores the lsb of data.
   */
  void fitUsingSSE3(float coef[10], const signed short data[3][3][3]) const;

  //! C reference implementation
  /*! Warning slow, just needed for unit tests. */
  void fitUsingC(float coef[10], const signed short data[3][3][3]) const;

  //! float data variant of \c fitUsingC
  /*! Warning slow, just needed for unit tests. */
  void fitUsingC(float coef[10], const float data[3][3][3]) const;

  //! Computes the maximum of the quadratic function described by \c coef
  /*! If the function has no maximum, +inf is returned. */
  void max(float& value, float arg[3], const float coef[10]) const;

  //! Evaluates the quadratic function defined by \c coef on \c arg
  static float eval(const float coef[10], const float arg[3]);

  //! Computes a cholesky decomposition \c A=LLt for a symmetric positive semidefinite A.
  /*! If the computation fails, the matrix is not S.P.D. and 'false' is returned. */
  static bool cholesky(const float A[3][3], float L[3][3]);

  //! Solves L*x=b for a lower-triangular L
  static void solveLxb(const float L[3][3], float x[3], const float b[3]);

  //! Solves L^T*x=b for a lower-triangular L, i.e. upper triangular L^T
  static void solveLTxb(const float L[3][3], float x[3], const float b[3]);

  //! Clips x to [min..max]
  static float clip(float x, float min, float max)
  {
    if(x < min)
      return min;
    else if(x <= max)
      return x;
    else
      return max;
  }
};
