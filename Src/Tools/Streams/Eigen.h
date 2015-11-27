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

#include "Streamable.h"

/**
 * Helper class to stream a fixed-sized row or column of an Eigen matrix.
 * @param T The element type of the row or column.
 * @param N The number of elements in the row or column.
 */
template<typename T, int N> class EigenMatrixRow
{
public:
  T elems[N]; /**< The elements of the row or column. */
};

/**
 * Writing a row or column of an Eigen matrix to a stream.
 * @param T The element type of the row or column.
 * @param N The number of elements in the row or column.
 * @param stream The stream to write to.
 * @param row The row or column to write.
 */
template<typename T, int N>
Out& operator<<(Out& stream, const EigenMatrixRow<T, N>& row)
{
  STREAM_REGISTER_BEGIN_EXT(row);
  STREAM_EXT(stream, row.elems);
  STREAM_REGISTER_FINISH;
  return stream;
}

/**
 * Reading a row or column of an Eigen matrix from a stream.
 * @param T The element type of the row or column.
 * @param N The number of elements in the row or column.
 * @param stream The stream to read from.
 * @param row The row or column to read.
 */
template<typename T, int N>
In& operator>>(In& stream, EigenMatrixRow<T, N>& row)
{
  STREAM_REGISTER_BEGIN_EXT(row);
  STREAM_EXT(stream, row.elems);
  STREAM_REGISTER_FINISH;
  return stream;
}

/**
 * Writing an Eigen matrix to a stream.
 * @param T The type of the elements.
 * @param ROWS The number of rows of the matrix.
 * @param COLS The number of columns of the matrix.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param MAX_ROWS The maximum number of rows of the matrix. Must equal ROWS.
 * @param MAX_COLS The maximum number of columns of the matrix. Must equal COLS.
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

  STREAM_REGISTER_BEGIN_EXT(matrix);
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
  STREAM_REGISTER_FINISH;

  return stream;
}

/**
 * Writing a horizontal Eigen vector to a stream.
 * @param T The type of the elements.
 * @param ELEMS The number of elements of the vector.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param MAX_ELEMS The maximum number of elements of the vector. Must equal ELEMS.
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

  T (*const elems)[ELEMS] = (T (*const)[ELEMS]) vector.data();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, elems);
  STREAM_REGISTER_FINISH;

  return stream;
}

/**
 * Writing a horizontal two dimensional Eigen vector to a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 1, 2, OPTIONS, 1, 2>& vector)
{
  const T& x = vector.x();
  const T& y = vector.y();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_REGISTER_FINISH;

  return stream;
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
  const T& x = vector.x();
  const T& y = vector.y();
  const T& z = vector.z();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_REGISTER_FINISH;

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

  T (*const elems)[ELEMS] = (T (*const)[ELEMS]) vector.data();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, elems);
  STREAM_REGISTER_FINISH;

  return stream;
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
  const T& x = array.x();
  const T& y = array.y();

  STREAM_REGISTER_BEGIN_EXT(array);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_REGISTER_FINISH;

  return stream;
}

/**
 * Writing a vertical two dimensional Eigen vector to a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to write to.
 * @param vector The vector to write.
 */
template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Matrix<T, 2, 1, OPTIONS, 2, 1>& vector)
{
  const T& x = vector.x();
  const T& y = vector.y();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_REGISTER_FINISH;

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
  const T& x = vector.x();
  const T& y = vector.y();
  const T& z = vector.z();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_REGISTER_FINISH;

  return stream;
}

/**
 * Reading an Eigen matrix from a stream.
 * @param T The type of the elements.
 * @param ROWS The number of rows of the matrix.
 * @param COLS The number of columns of the matrix.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param MAX_ROWS The maximum number of rows of the matrix. Must equal ROWS.
 * @param MAX_COLS The maximum number of columns of the matrix. Must equal COLS.
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

  STREAM_REGISTER_BEGIN_EXT(matrix);
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
  STREAM_REGISTER_FINISH;

  return stream;
}

/**
 * Reading a horizontal Eigen vector from a stream.
 * @param T The type of the elements.
 * @param ELEMS The number of elements of the vector.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param MAX_ELEMS The maximum number of elements of the vector. Must equal ELEMS.
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

  T (*elems)[ELEMS] = (T (*)[ELEMS]) vector.data();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, elems);
  STREAM_REGISTER_FINISH;

  return stream;
}

/**
 * Reading a horizontal two dimensional Eigen vector from a stream.
 * @param T The type of the elements.
 * @param OPTIONS Mainly describes whether the matrix is row major or column major.
 * @param stream The stream to read from.
 * @param vector The vector to read.
 */
template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Matrix<T, 1, 2, OPTIONS, 1, 2>& vector)
{
  T& x = vector.x();
  T& y = vector.y();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_REGISTER_FINISH;

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
  T& x = vector.x();
  T& y = vector.y();
  T& z = vector.z();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_REGISTER_FINISH;

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

  T (*elems)[ELEMS] = (T (*)[ELEMS]) vector.data();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, elems);
  STREAM_REGISTER_FINISH;

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
  T& x = array.x();
  T& y = array.y();

  STREAM_REGISTER_BEGIN_EXT(array);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_REGISTER_FINISH;

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
In& operator>>(In& stream, Eigen::Matrix<T, 2, 1, OPTIONS, 2, 1>& vector)
{
  T& x = vector.x();
  T& y = vector.y();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_REGISTER_FINISH;

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
  T& x = vector.x();
  T& y = vector.y();
  T& z = vector.z();

  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_REGISTER_FINISH;

  return stream;
}

template<typename T, int OPTIONS>
In& operator>>(In& stream, Eigen::Quaternion<T, OPTIONS>& quaternion)
{
  T& x = quaternion.x();
  T& y = quaternion.y();
  T& z = quaternion.z();
  T& w = quaternion.w();

  STREAM_REGISTER_BEGIN_EXT(quaternion);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_EXT(stream, w);
  STREAM_REGISTER_FINISH;

  return stream;
}

template<typename T, int OPTIONS>
Out& operator<<(Out& stream, const Eigen::Quaternion<T, OPTIONS>& quaternion)
{
  const T& x = quaternion.x();
  const T& y = quaternion.y();
  const T& z = quaternion.z();
  const T& w = quaternion.w();

  STREAM_REGISTER_BEGIN_EXT(quaternion);
  STREAM_EXT(stream, x);
  STREAM_EXT(stream, y);
  STREAM_EXT(stream, z);
  STREAM_EXT(stream, w);
  STREAM_REGISTER_FINISH;

  return stream;
}
