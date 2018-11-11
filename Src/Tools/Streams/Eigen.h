/**
 * @file Tools/Streams/Eigen.h
 *
 * The file defines streaming operators for the Eigen matrix class.
 * Currently, only fixed-sized matrices are supported.
 * Note that this file does not include Eigen matrices. They have
 * to be included before this file is.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "AutoStreamable.h"

/**
 * Helper class to stream a fixed-sized row or column of an Eigen matrix.
 * @tparam T The element type of the row or column.
 * @tparam N The number of elements in the row or column.
 */
template<typename T, int N> struct EigenMatrixRow
{
  T elems[N]; /**< The elements of the row or column. */
};

/**
 * Register an Eigen matrix row.
 * @tparam T The type of the elements.
 * @tparam N The number of elements of the row.
 */
template<typename T, int N> void regEigenMatrixRow()
{
  REG_CLASS(EigenMatrixRow<T, N>);
  REG(T[N], elems);
}

/**
 * Writing a row or column of an Eigen matrix to a stream.
 * @tparam T The element type of the row or column.
 * @tparam N The number of elements in the row or column.
 * @param stream The stream to write to.
 * @param row The row or column to write.
 */
template<typename T, int N>
Out& operator<<(Out& stream, const EigenMatrixRow<T, N>& row)
{
  PUBLISH(regEigenMatrixRow<T, N>);
  STREAM_EXT(stream, row.elems);
  return stream;
}

/**
 * Reading a row or column of an Eigen matrix from a stream.
 * @tparam T The element type of the row or column.
 * @tparam N The number of elements in the row or column.
 * @param stream The stream to read from.
 * @param row The row or column to read.
 */
template<typename T, int N>
In& operator>>(In& stream, EigenMatrixRow<T, N>& row)
{
  PUBLISH(regEigenMatrixRow<T, N>);
  STREAM_EXT(stream, row.elems);
  return stream;
}

/**
 * Register an Eigen matrix.
 * @tparam T The type of the elements.
 * @tparam ROWS The number of rows of the matrix.
 * @tparam COLS The number of columns of the matrix.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @tparam MAX_ROWS The maximum number of rows of the matrix. Must equal ROWS.
 * @tparam MAX_COLS The maximum number of columns of the matrix. Must equal COLS.
 */
template<typename T, int ROWS, int COLS, int OPTIONS>
void regMatrix()
{
  REG_CLASS(Eigen::Matrix<T, ROWS, COLS, OPTIONS, ROWS, COLS>);
  if(OPTIONS & Eigen::RowMajor)
    REG(EigenMatrixRow<T COMMA COLS> (*)[ROWS], rows);
  else
    REG(EigenMatrixRow<T COMMA COLS> (*)[ROWS], cols);
}

/**
 * Writing an Eigen matrix to a stream.
 * @tparam T The type of the elements.
 * @tparam ROWS The number of rows of the matrix.
 * @tparam COLS The number of columns of the matrix.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @tparam MAX_ROWS The maximum number of rows of the matrix. Must equal ROWS.
 * @tparam MAX_COLS The maximum number of columns of the matrix. Must equal COLS.
 * @param stream The stream to write to.
 * @param matrix The matrix to write.
 */
template<typename T, int ROWS, int COLS, int OPTIONS, int MAX_ROWS, int MAX_COLS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, ROWS, COLS, OPTIONS, MAX_ROWS, MAX_COLS>& matrix)
{
  static_assert(ROWS != Eigen::Dynamic && COLS != Eigen::Dynamic,
                "Streaming dynamic Eigen matrix not supported yet");
  static_assert(ROWS == MAX_ROWS && COLS == MAX_COLS,
                "Setting _MaxRows or _MaxCols is not supported yet");

  PUBLISH(regMatrix<T, ROWS, COLS, OPTIONS>);
  if(OPTIONS & Eigen::RowMajor)
  {
    EigenMatrixRow<T, COLS> (*const rows)[ROWS] = (EigenMatrixRow<T, COLS> (*const)[ROWS]) matrix.data();
    STREAM_EXT(stream, rows);
  }
  else
  {
    EigenMatrixRow<T, COLS> (*const cols)[ROWS] = (EigenMatrixRow<T, COLS> (*const)[ROWS]) matrix.data();
    STREAM_EXT(stream, cols);
  }

  return stream;
}

/**
 * Reading an Eigen matrix from a stream.
 * @tparam T The type of the elements.
 * @tparam ROWS The number of rows of the matrix.
 * @tparam COLS The number of columns of the matrix.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @tparam MAX_ROWS The maximum number of rows of the matrix. Must equal ROWS.
 * @tparam MAX_COLS The maximum number of columns of the matrix. Must equal COLS.
 * @param stream The stream to read from.
 * @param matrix The matrix to read.
 */
template<typename T, int ROWS, int COLS, int OPTIONS, int MAX_ROWS, int MAX_COLS>
In& operator>>(In& stream, Eigen::Matrix<T, ROWS, COLS, OPTIONS, MAX_ROWS, MAX_COLS>& matrix)
{
  static_assert(ROWS != Eigen::Dynamic && COLS != Eigen::Dynamic,
                "Streaming dynamic Eigen matrix not supported yet");
  static_assert(ROWS == MAX_ROWS && COLS == MAX_COLS,
                "Setting _MaxRows or _MaxCols is not supported yet");

  PUBLISH(regMatrix<T, ROWS, COLS, OPTIONS>);
  if(OPTIONS & Eigen::RowMajor)
  {
    EigenMatrixRow<T, COLS> (*rows)[ROWS] = (EigenMatrixRow<T, COLS> (*)[ROWS]) matrix.data();
    STREAM_EXT(stream, rows);
  }
  else
  {
    EigenMatrixRow<T, COLS> (*cols)[ROWS] = (EigenMatrixRow<T, COLS> (*)[ROWS]) matrix.data();
    STREAM_EXT(stream, cols);
  }

  return stream;
}

/**
 * Register an Eigen vector.
 * @tparam T The type of the elements.
 * @tparam ROWS The number of rows of the vector.
 * @tparam COLS The number of columns of the vector.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 */
template<typename T, int ROWS, int COLS, int OPTIONS> void regMatrix1()
{
  REG_CLASS(Eigen::Matrix<T, ROWS, COLS, OPTIONS, ROWS, COLS>);
  REG(T (*)[ROWS > COLS ? ROWS : COLS], elems);
}

/**
 * Writing a horizontal Eigen vector to a stream.
 * @tparam T The type of the elements.
 * @tparam ELEMS The number of elements of the vector.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @tparam MAX_ELEMS The maximum number of elements of the vector. Must equal ELEMS.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int ELEMS, int OPTIONS, int MAX_ELEMS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 1, ELEMS, OPTIONS, 1, MAX_ELEMS>& vector)
{
  static_assert(ELEMS != Eigen::Dynamic,
                "Streaming dynamic Eigen matrix not supported yet");
  static_assert(ELEMS == MAX_ELEMS,
                "Setting _MaxCols is not supported yet");

  PUBLISH(regMatrix1<T, 1, ELEMS, OPTIONS>);
  T (*const elems)[ELEMS] = (T (*const)[ELEMS]) vector.data();
  STREAM_EXT(stream, elems);

  return stream;
}

/**
 * Reading a horizontal Eigen vector from a stream.
 * @tparam T The type of the elements.
 * @tparam ELEMS The number of elements of the vector.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @tparam MAX_ELEMS The maximum number of elements of the vector. Must equal ELEMS.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int ELEMS, int OPTIONS, int MAX_ELEMS>
In& operator>>(In& stream, Eigen::Matrix<T, 1, ELEMS, OPTIONS, 1, MAX_ELEMS>& vector)
{
  static_assert(ELEMS != Eigen::Dynamic,
                "Streaming dynamic Eigen matrix not supported yet");
  static_assert(ELEMS == MAX_ELEMS,
                "Setting _MaxCols is not supported yet");

  PUBLISH(regMatrix1<T, 1, ELEMS, OPTIONS>);
  T (*elems)[ELEMS] = (T (*)[ELEMS]) vector.data();
  STREAM_EXT(stream, elems);

  return stream;
}

/**
 * Writing a vertical Eigen vector to a stream.
 * @param T The type of the elements.
 * @param ELEMS The number of elements of the vector.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param MAX_ELEMS The maximum number of elements of the vector. Must equal ELEMS.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int ELEMS, int OPTIONS, int MAX_ELEMS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, ELEMS, 1, OPTIONS, MAX_ELEMS, 1>& vector)
{
  static_assert(ELEMS != Eigen::Dynamic,
                "Streaming dynamic Eigen matrix not supported yet");
  static_assert(ELEMS == MAX_ELEMS,
                "Setting _MaxRows is not supported yet");

  PUBLISH(regMatrix1<T, ELEMS, 1, OPTIONS>);
  T (*const elems)[ELEMS] = (T (*const)[ELEMS]) vector.data();
  STREAM_EXT(stream, elems);

  return stream;
}

/**
 * Reading a vertical Eigen vector from a stream.
 * @param T The type of the elements.
 * @param ELEMS The number of elements of the vector.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param MAX_ELEMS The maximum number of elements of the vector. Must equal ELEMS.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int ELEMS, int OPTIONS, int MAX_ELEMS>
In& operator>>(In& stream, Eigen::Matrix<T, ELEMS, 1, OPTIONS, MAX_ELEMS, 1>& vector)
{
  static_assert(ELEMS != Eigen::Dynamic,
                "Streaming dynamic Eigen matrix not supported yet");
  static_assert(ELEMS == MAX_ELEMS,
                "Setting _MaxRows is not supported yet");

  PUBLISH(regMatrix1<T, ELEMS, 1, OPTIONS>);
  T (*elems)[ELEMS] = (T (*)[ELEMS]) vector.data();
  STREAM_EXT(stream, elems);

  return stream;
}

/**
 * Register a two dimensional Eigen vector.
 * @tparam T The type of the elements.
 * @tparam ROWS The number of rows of the vector.
 * @tparam COLS The number of columns of the vector.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 */
template<typename T, int ROWS, int COLS, int OPTIONS> void regMatrixXY()
{
  REG_CLASS(Eigen::Matrix<T, ROWS, COLS, OPTIONS, ROWS, COLS>);
  REG(T, x);
  REG(T, y);
}

/**
 * Writing a horizontal two dimensional Eigen vector to a stream.
 * @tparam T The type of the elements.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 1, 2, OPTIONS, 1, 2>& vector)
{
  PUBLISH(regMatrixXY<T, 1, 2, OPTIONS>);
  const T& x = vector.x();
  const T& y = vector.y();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);

  return stream;
}

/**
 * Reading a horizontal two dimensional Eigen vector from a stream.
 * @tparam T The type of the elements.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Matrix<T, 1, 2, OPTIONS, 1, 2>& vector)
{
  PUBLISH(regMatrixXY<T, 1, 2, OPTIONS>);
  T& x = vector.x();
  T& y = vector.y();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);

  return stream;
}

/**
 * Writing a vertical two dimensional Eigen vector to a stream.
 * @tparam T The type of the elements.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 2, 1, OPTIONS, 2, 1>& vector)
{
  PUBLISH(regMatrixXY<T, 2, 1, OPTIONS>);
  const T& x = vector.x();
  const T& y = vector.y();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);

  return stream;
}

/**
 * Reading a vertical two dimensional Eigen vector from a stream.
 * @tparam T The type of the elements.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Matrix<T, 2, 1, OPTIONS, 2, 1>& vector)
{
  PUBLISH(regMatrixXY<T, 2, 1, OPTIONS>);
  T& x = vector.x();
  T& y = vector.y();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);

  return stream;
}

/**
 * Register a three dimensional Eigen vector.
 * @tparam T The type of the elements.
 * @tparam ROWS The number of rows of the vector.
 * @tparam COLS The number of columns of the vector.
 * @tparam OPTIONS Mainly describes whether the matrix is row major or column major.
 */
template<typename T, int ROWS, int COLS, int OPTIONS> void regMatrixXYZ()
{
  REG_CLASS(Eigen::Matrix<T, ROWS, COLS, OPTIONS, ROWS, COLS>);
  REG(T, x);
  REG(T, y);
  REG(T, z);
}

/**
 * Writing a horizontal three dimensional Eigen vector to a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 1, 3, OPTIONS, 1, 3>& vector)
{
  PUBLISH(regMatrixXYZ<T, 1, 3, OPTIONS>);
  const T& x = vector.x();
  const T& y = vector.y();
  const T& z = vector.z();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);

  return stream;
}

/**
 * Reading a horizontal three dimensional Eigen vector from a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Matrix<T, 1, 3, OPTIONS, 1, 3>& vector)
{
  PUBLISH(regMatrixXYZ<T, 1, 3, OPTIONS>);
  T& x = vector.x();
  T& y = vector.y();
  T& z = vector.z();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);

  return stream;
}

/**
 * Writing a vertical three dimensional Eigen vector to a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 3, 1, OPTIONS, 3, 1>& vector)
{
  PUBLISH(regMatrixXYZ<T, 3, 1, OPTIONS>);
  const T& x = vector.x();
  const T& y = vector.y();
  const T& z = vector.z();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);

  return stream;
}

/**
 * Reading a vertical Eigen vector from a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Matrix<T, 3, 1, OPTIONS, 3, 1>& vector)
{
  PUBLISH(regMatrixXYZ<T, 1, 3, OPTIONS>);
  T& x = vector.x();
  T& y = vector.y();
  T& z = vector.z();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);

  return stream;
}

/**
 * Register a vertical two dimensional Eigen array.
 * @tparam T The type of the elements.
 * @tparam OPTIONS Mainly describes whether the vector is row major or column major.
 */
template<typename T, int OPTIONS> void regArrayXY()
{
  REG_CLASS(Eigen::Array<T, 2, 1, OPTIONS, 2, 1>);
  REG(T, x);
  REG(T, y);
}

/**
 * Writing a vertical two dimensional Eigen array to a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param array The array to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Array<T, 2, 1, OPTIONS, 2, 1>& array)
{
  PUBLISH(regArrayXY<T, OPTIONS>);
  const T& x = array.x();
  const T& y = array.y();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);

  return stream;
}

/**
 * Reading a vertical Eigen array from a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param array The array to read.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Array<T, 2, 1, OPTIONS, 2, 1>& array)
{
  PUBLISH(regArrayXY<T, OPTIONS>);
  T& x = array.x();
  T& y = array.y();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);

  return stream;
}

/**
 * Register an Eigen quaternion.
 * @tparam T The type of the elements.
 * @tparam OPTIONS Mainly describes whether the vector is row major or column major.
 */
template<typename T, int OPTIONS> void regQuaternion()
{
  REG_CLASS(Eigen::Quaternion<T, OPTIONS>);
  REG(T, x);
  REG(T, y);
  REG(T, z);
  REG(T, w);
}

/**
 * Writing an Eigen quaternion to a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param quaternion The quaternion to write.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Quaternion<T, OPTIONS>& quaternion)
{
  PUBLISH(regQuaternion<T, OPTIONS>);
  T& x = quaternion.x();
  T& y = quaternion.y();
  T& z = quaternion.z();
  T& w = quaternion.w();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_EXT(stream, w);

  return stream;
}

/**
 * Reading an Eigen quaternion from a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param quaternion The quaternion to read.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Quaternion<T, OPTIONS>& quaternion)
{
  PUBLISH(regQuaternion<T, OPTIONS>);
  const T& x = quaternion.x();
  const T& y = quaternion.y();
  const T& z = quaternion.z();
  const T& w = quaternion.w();

  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_EXT(stream, w);

  return stream;
}

/**
 * Register an angle axis Eigen vector.
 * @tparam T The type of the elements.
 */
template<typename T> void regAngleAxis()
{
  REG_CLASS(Eigen::AngleAxis<T>);
  REG(T, angle);
  REG(Eigen::Matrix<T COMMA 3 COMMA 1>, axis);
}

/**
 * Writing an Eigen angle axis vector to a stream.
 * @param T The type of the elements.
 * @param stream The stream to write to.
 * @param angleaxis The angle axis vector to write.
 */
template<typename T>
In& operator>>(In& stream, Eigen::AngleAxis<T>& angleaxis)
{
  PUBLISH(regAngleAxis<T>);
  T& angle = angleaxis.angle();
  Eigen::Matrix<T, 3, 1>& axis = angleaxis.axis();

  STREAM_EXT(stream, angle);
  STREAM_EXT(stream, axis);

  return stream;
}

/**
 * Reading an Eigen angle axis vector from a stream.
 * @param T The type of the elements.
 * @param stream The stream to read from.
 * @param angleaxis The angle axis vector to read.
 */
template<typename T>
Out& operator<<(Out& stream, const Eigen::AngleAxis<T>& angleaxis)
{
  PUBLISH(regAngleAxis<T>);
  const T& angle = angleaxis.angle();
  const Eigen::Matrix<T, 3, 1>& axis = angleaxis.axis();

  STREAM_EXT(stream, angle);
  STREAM_EXT(stream, axis);

  return stream;
}
