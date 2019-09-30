#pragma once

#include "Platform/BHAssert.h" // Our Eigen extensions use ASSERT

// Extend the Eigen classes with our own methods (see: http://eigen.tuxfamily.org/dox-devel/TopicCustomizingEigen.html)
#define EIGEN_MATRIXBASE_PLUGIN "Tools/Math/EigenMatrixBaseExtensions.h"
#define EIGEN_ARRAY_PLUGIN "Tools/Math/EigenArrayExtensions.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include "Tools/Streams/Eigen.h"

class Angle;

using Array2a = Eigen::Array<Angle, 2, 1>;
using Array2d = Eigen::Array2d;
using Array2f = Eigen::Array2f;
using Array3f = Eigen::Array3f;

using Vector2a = Eigen::Matrix<Angle, 2, 1>;
using Vector2d = Eigen::Vector2d;
using Vector2f = Eigen::Vector2f;
using Vector2i = Eigen::Vector2i;
using Vector2s = Eigen::Matrix<short, 2, 1>;
using Vector2ui = Eigen::Matrix<unsigned int, 2, 1>;
using Vector3a = Eigen::Matrix<Angle, 3, 1>;
using Vector3d = Eigen::Vector3d;
using Vector3f = Eigen::Vector3f;
using Vector3i = Eigen::Vector3i;
using Vector4a = Eigen::Matrix<Angle, 4, 1>;
using Vector4d = Eigen::Vector4d;
using Vector4f = Eigen::Vector4f;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector5f = Eigen::Matrix<float, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using VectorXd = Eigen::VectorXd;
using VectorXf = Eigen::VectorXf;

using RowVector2d = Eigen::RowVector2d;
using RowVector2f = Eigen::RowVector2f;
using RowVector3d = Eigen::RowVector3d;
using RowVector3f = Eigen::RowVector3f;
using RowVector4d = Eigen::RowVector4d;
using RowVector4f = Eigen::RowVector4f;
using RowVectorXd = Eigen::RowVectorXd;
using RowVectorXf = Eigen::RowVectorXf;

using Matrix2d = Eigen::Matrix2d;
using Matrix2f = Eigen::Matrix2f;
using Matrix2x3d = Eigen::Matrix<double, 2, 3>;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;
using Matrix2x4d = Eigen::Matrix<double, 2, 4>;
using Matrix2x4f = Eigen::Matrix<float, 2, 4>;
using Matrix3d = Eigen::Matrix3d;
using Matrix3f = Eigen::Matrix3f;
using Matrix3x2d = Eigen::Matrix<double, 3, 2>;
using Matrix3x2f = Eigen::Matrix<float, 3, 2>;
using Matrix4f = Eigen::Matrix4f;
using Matrix4d = Eigen::Matrix4d;
using Matrix4x2d = Eigen::Matrix<double, 4, 2>;
using Matrix4x2f = Eigen::Matrix<float, 4, 2>;
using Matrix4x3d = Eigen::Matrix<double, 4, 3>;
using Matrix4x3f = Eigen::Matrix<float, 4, 3>;
using Matrix2xXd = Eigen::Matrix<double, 2, Eigen::Dynamic>;
using Matrix2xXf = Eigen::Matrix<float, 2, Eigen::Dynamic>;
using Matrix5f = Eigen::Matrix<float, 5, 5>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
using Matrix6x2f = Eigen::Matrix<float, 6, 2>;
using Matrix6x4f = Eigen::Matrix<float, 6, 4>;
using MatrixXx2d = Eigen::Matrix<double, Eigen::Dynamic, 2>;
using MatrixXx2f = Eigen::Matrix<float, Eigen::Dynamic, 2>;
using MatrixXx3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using MatrixXx3f = Eigen::Matrix<float, Eigen::Dynamic, 3>;
using MatrixXd = Eigen::MatrixXd;
using MatrixXf = Eigen::MatrixXf;

using Quaterniond = Eigen::Quaterniond;
using Quaternionf = Eigen::Quaternionf;
using AngleAxisf = Eigen::AngleAxisf;
