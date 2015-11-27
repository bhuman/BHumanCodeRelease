/**
 * File:   QuarticPolynomial.h
 * Author: arne
 *
 * Created on May 1, 2014, 10:34 AM
 */

#pragma once

/**
 * This class provides methods for solving polynomials of small degrees.
 * I.e. polynomials of degrees one, two, three and four.
 *
 * @note This code has been taken from the Graphics Gems repository
 *       http://tog.acm.org/resources/GraphicsGems/
 *       Specifically from
 *       http://tog.acm.org/resources/GraphicsGems/gemsiv/vec_mat/ray/solver.c
 *
 * @copyright The Graphics Gems code is copyright-protected.
 *            In other words, you cannot claim the text of the code as your
 *            own and resell it. Using the code is permitted in any program,
 *            product, or library, non-commercial or commercial.
 *            Giving credit is not required, though is a nice gesture.
 *            The code comes as-is, and if there are any flaws or problems
 *            with any Gems code, nobody involved with Gems - authors,
 *            editors, publishers, or webmasters - are to be held responsible.
 *            Basically, don't be a jerk, and remember that anything
 *            free comes with no guarantee.
 *
 */
class PolynomialSolver
{
public:
  /**
   * This function determines the roots of a quadric equation. It takes as
   * parameters a pointer to the three coefficient of the quadric equation
   * (the c[2] is the coefficient of x2 and so on) and a pointer to the
   * two element array in which the roots are to be placed. It outputs the
   * number of roots found.
   */
  static int solveQuadric(float c[3], float s[2]);

  /**
   * This function determines the roots of a cubic equation. It takes as
   * parameters a pointer to the four coefficient of the cubic equation
   * (the c[3] is the coefficient of x3 and so on) and a pointer to the
   * three element array in which the roots are to be placed. It outputs
   * the number of roots found.
   */
  static int solveCubic(float c[4], float s[3]);

  /**
   * This function determines the roots of a quartic equation. It takes as
   * parameters a pointer to the five coefficient of the quartic equation
   * (the c[4] is the coefficient of x4 and so on) and a pointer to the
   * four element array in which the roots are to be placed. It outputs the
   * number of roots found.
   */
  static int solveQuartic(float c[5], float s[4]);
  static int solveLinear(float c[2], float s[1]);

private:
  /**
   * This function determines if a float is small enough to be zero. The purpose
   * of the subroutine is to try to overcome precision problems in math routines.
   */
  static int isZero(float x);
};
