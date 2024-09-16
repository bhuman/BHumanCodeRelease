/**
 * @file SE3Tools.h
 *
 * Declaration of some tools for the SE3 Lie-group and the associated algebra se3
 *
 * The vector representation of elements from the algebra has the translational parameters as the first three elements and the rotational as the next three
 * [translation, rotation].transpose
 * this is important to construct the se3 element from the vector as well as for the construction of the adjoint matrix
 *
 * Sources:
 * [1] J.-L. Blanco, “A tutorial on SE(3) transformation parameterizations and on-manifold optimization,” MAPIR: Grupo de Percepci ́ on y Rob ́ otica Dpto. de Ingenier ́ıa de Sistemas y Autom ́atica, Málaga, Technical #012010, Oct. 2014. Accessed: May 09, 2023. [Online]. Available: http://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
 * [2] L. Meyer, K. H. Strobl, and R. Triebel, “The Probabilistic Robot Kinematics Model and its Application to Sensor Fusion,” in 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct. 2022, pp. 3263–3270. doi: 10.1109/IROS47612.2022.9981399.
 *
 * @author Yannik Meinken
 */
#pragma once


#include "Pose3f.h"

namespace SE3
{
  /**
   * Creates a 3x3 matrix representing the skew symmetric matrix for a Vector3f
   * @param vec vector from which to create the matrix
   * @return skew symmetric matrix
   */
  Matrix3f skewMatrix(const Vector3f& vec);

  /**
   * Inverse of skewMatrix()
   * @param m skew symmetric matrix
   * @return vector which corresponds to the given skew symmetric matrix
   */
  Vector3f skewToVector(const Matrix3f& m);

  /**
   * This creates the adjoint matrix for the current Pose.
   *  The adjoint matrix can be used to change reference frames,
   *  it moves an element from one tangent space to another tangent space of the same group.
   * @param pose point at which the tangent space touches the manifold
   * @return adjoint matrix to transform from the global algebra to the local one
   */
  Matrix6f adjoint(const Pose3f& pose);

  /**
   * Transforms a cartesian vector to an element of the se3 Lie algebra
   * @param s vector representation of an element from se3
   * @return element on se3 corresponding to the vector
   */
  Matrix4f hat(const Vector6f& s);

  /**
   * Inverse of hat()
   * @param se element on se3, needs to follow the constraints
   * @return vector representation of s
   */
  Vector6f vee(const Matrix4f& se);

  /**
   * Exponential map to transform vector representation of elements from the se3 algebra to the group
   * uses the matrix exponential
   * @param s vector representation of an element from se3
   * @return element on the group/manifold
   */
  Pose3f exp(const Vector6f& s);

  /**
   * Inverse of the exponential map
   * maps an element from SE3 to the vector representation of the tangent space se3
   * @param p element from the group
   * @return vector representation of the screw displacement necessary to get to the pose x
   */
  Vector6f ln(const Pose3f& p);
}
