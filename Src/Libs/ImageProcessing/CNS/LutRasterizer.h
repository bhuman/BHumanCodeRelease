#pragma once

#include "TriangleMesh.h"
#include "CameraModelOpenCV.h"
#include "CodedContour.h"
#include <Eigen/StdVector>

//! Algorithms and precomputed data-structures to perform ShapeCNSDetector::rasterize efficiently
/*! The current implementation supports only distortion-free cameras.

    There is a 3-D table mapping the camera position in object coordinates to the
    list of edges visible. There is also an optimized implementation that perspectively
    projects the vertices of these edges computes there center and normal and stores
    that as a point in a \c CodedContour. This means the \c LutRasterizer only computes
    sparse contours.

    At the moment the object is limited to 255 vertices.
 */
class LutRasterizer
{
public:
  //! The 3-D object that is going to be rasterized
  /* The geometry is given in object coordinates. */
  TriangleMesh object;

  //! The points from \c object.vertex with a 1 as fourth component
  /*! These vectors can be used more easily in SIMD operations
      in particular a SIMD matrix vector product.
   */
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > vertexWith1;

  //! Center of the object padded with 1
  /*! A point where the max distance to any vertex is smallest. */
  Eigen::Vector4f centerWith1;

  //! Projection of 3-D object-coordinates to 2-D image coordinates as homogenous matrix
  using Perspective = Eigen::Matrix<float, 3, 4>;

  //! List of edges that form a contour from a certain point of view in a specially coded way
  /*! Actually the list is a sequence of indices to \c
      object.vertex. The edges are sorted in a way that is as
      sequential as possible, so usually the end-vertex of an edge is
      the starting vertex of the next edge. If this is not the case,
      a \c NEWSTART entry is inserted. The formally the edges in an \c VertexList el
      are (object.vertex[el[i]], object.vertex[el[i+1]]), for i=0..el.size()-2 and
          object.vertex[el[i]]!=NEWSTART and object.vertex[el[i+1]]!=NEWSTART
   */
  using VertexList = std::vector<unsigned char>;

  enum {NEWSTART = 0xff};

  //! Look-up-table storing the contour of \c object from different viewpoints as \c VertexList objects
  /*! edgelist[indexOfViewPoint(v)] is the contour of \c object viewed from \c v as a list of vertex
      indices.

      Technically, the array is a regular \c spacing grid of points with dimensions \c edgeListSize[0]*
      \c edgeListSize[1] * \c edgeListSize[2] in X, Y, Z.
   */
  std::vector<VertexList> vertexList;

  //! See \c indexOfViewPoint
  int vertexListSize[3];

  //! See \c indexOfViewPoint
  Eigen::Vector3d basePoint;

  //! See \c indexOfViewPoint
  double spacing;

  //! The range of viewpoints covered by the LUT \c vertexList
  /*! This is the parameter passed to \c create, so viewpoints inside this box
      are tabulated, the actually tabulated area may be larger due to effects of
      rotational normalization and rounding to \c spacing.
   */
  Eigen::AlignedBox3d viewpointRange;

  LutRasterizer() : spacing(0) {vertexListSize[0] = vertexListSize[1] = vertexListSize[2] = 0;}

  //! See \c create
  LutRasterizer(const TriangleMesh& object, const Eigen::AlignedBox3d& viewpointRange, double spacing)
  {create(object, viewpointRange, spacing);}

  //! Computes the main look-up-table needed to for \c rasterize
  /*! \c object is the 3-D object considered; \c viewpointRange is the cube of camera positions in
      object coordinates for which the look-up-table is computed. \c spacing gives the distance
      between discretized samples in that volume. If \c object.isRotationalSymmetricZ, the table
      only uses normalized positions (X>0, Y=0) and is much smaller.
   */
  void create(const TriangleMesh& object, const Eigen::AlignedBox3d& viewpointRange, double spacing);

  //! Same as \c create but tries to load and saves the look-up-table in \c filename
  /*! Note: At the moment the user has to take care to delete \c filename the parameters have been changed.
      If \c filename is \c nullptr, the table is neither loaded nor saved.
   */
  void loadOrCreate(const TriangleMesh& object, const Eigen::AlignedBox3d& viewpointRange, double spacing, const char* filename = nullptr);

  //! Copies \c object into \c this->object and updates \c vertexWith1 and \c centerWith1
  void setObject(const TriangleMesh& object);

  //! Renders the contour of \c mesh at pose \c object2World from the perspective of \c camera
  /*! At the moment only sparse rasterization is implemented (one point in the middle of each edge).

   */
  void rasterize(CodedContour& contour, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera) const;

  //! Returns the viewpoint from which \c vertexList[idx] is computed.
  Eigen::Vector3d viewpointOfIndex(int idx) const
  {
    return basePoint + spacing * Eigen::Vector3d(
             idx % vertexListSize[0],
             (idx / vertexListSize[0]) % vertexListSize[1],
             idx / vertexListSize[0] / vertexListSize[1]);
  }

  //! Rotates \c object2World around object-Z such that the viewpoint is in the tabulated region
  /*! It rotates \c object2Camera such that the object relative to the
      camera is the same due to symmetry but the viewpoint \c
      object2Camera.inverse().translation() is in the region tabulated
      (X>0, Y=0). If \c !isRotationalSymmetricZ nothing is done.
   */
  void normalizeObject2Camera(Eigen::Isometry3d& object2Camera) const
  {
    if(!object.isRotationalSymmetricZ)
      return;
    Eigen::Vector3d viewpointInObject = object2Camera.inverse().translation();

    double len = sqrt(viewpointInObject(0) * viewpointInObject(0) + viewpointInObject(1) * viewpointInObject(1));
    if(len == 0)
      return;
    double c = viewpointInObject(0) / len, s = viewpointInObject(1) / len;
    double angle = atan2(s, c);
    /*    // c,s are cos and sin of a rotation matrix that rotates viewpointInObject to X>0, Y=0
    Eigen::Matrix4d rot;
    rot <<
      1, 0, 0, 0,
      0, c,-s, 0,
      0, s, c, 0,
      0, 0, 0, 1;
    object2Camera = object2Camera*Eigen::Isometry3d(rot); //TODO: improve performance?
     */
    object2Camera = object2Camera * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    viewpointInObject = object2Camera.inverse().translation();
  }

  //! Computes the index in \c vertexList corresponding to the viewpoint closest to \c viewpoint
  /*! If \c viewpoint is outside the volume (here cube) of tabulated viewpoints,
      -1 is returned.
   */
  int indexOfViewpoint(const Eigen::Vector3d& viewpoint) const
  {
    if(object.isRotationalSymmetricZ)    // We rotate around Z such that we get X>=0 and Y=0
    {
      Eigen::Vector3d normalizedViewpoint;
      normalizedViewpoint(0) = sqrt(viewpoint(0) * viewpoint(0) + viewpoint(1) * viewpoint(1));
      normalizedViewpoint(1) = 0;
      normalizedViewpoint(2) = viewpoint(2);
      return indexOfNormalizedViewpoint(normalizedViewpoint);
    }
    else
      return indexOfNormalizedViewpoint(viewpoint);
  }

  //! Compute \c indexOfViewpoint for a viewpoint that is already normalized w.r.t. rotational symmetry
  /*! More of internal interest. */
  int indexOfNormalizedViewpoint(const Eigen::Vector3d& viewpoint) const
  {
    Eigen::Vector3d index = (viewpoint - basePoint) / spacing;
    int idx[3];
    idx[0] = static_cast<int>(round(index(0)));
    idx[1] = static_cast<int>(round(index(1)));
    idx[2] = static_cast<int>(round(index(2)));
    if(0 <= idx[0] && idx[0] < vertexListSize[0] &&
       0 <= idx[1] && idx[1] < vertexListSize[1] &&
       0 <= idx[2] && idx[2] < vertexListSize[2])
      return idx[0] + vertexListSize[0] * (idx[1] + idx[2] * vertexListSize[1]);
    else
      return -1;
  }

  //! Computes \c centerWith1
  /*! At the moment by taking the middle between min and max in x/y/z
   */
  void computeCenterPoint();

  //! Computes the vertex list (see \c VertexList) of \c object seen from \c viewpoint
  static void computeVertexList(LutRasterizer::VertexList& vertexList, const TriangleMesh& object, const Eigen::Vector3d& viewpoint);

  //! Counts for each vertex how many edges are incident to it
  static void countVertices(std::vector<int>& counter, const TriangleMesh::EdgeList& el);

  //! Takes a list of edges and computes a vertex list from that in a clever way
  /*! The vertex-list (according to the definition in \c VertexList) contains exactly the edges in \c vertexList.
      This is done by moving through every edge once, avoiding to jump but jumping if necessary (\c NEWSTART).
     */
  static void convertEdgeListToVertexList(LutRasterizer::VertexList& vertexList, const TriangleMesh::EdgeList& edgeList);

  //! Takes a list of vertices defining an edge list and converts it back to an edge list
  static void convertVertexListToEdgeList(TriangleMesh::EdgeList& edgeList, LutRasterizer::VertexList& vertexList);

  //! Allocates \c vertexList for a ranges of discretized vertex coordinates (see \c computeLut)
  /*! \c object must already been set.*/
  void allocateLut(const Eigen::AlignedBox3d& viewpointRange, double spacing);

  //! Computes several quantities needed in \c rasterize to call \c projectUsingSSE (see \c rasterize for last two parameters)
  /*! It computes the index of the precomputed vertexlist applying to the given \c object2World and camera.
      Also a reference point which is the projection of \c centerWith1 around which the 8bit signed coordinates
      in \c CodedContour are defined. And it computes a projection matrix in the form taken by \c projectUsingSSE
      which maps homogenous object vectors (e.g. \c vertexWith1) to homogenous image coordinates relative to
      the reference point.

      (clipRange[0]..clipRange[2])*(clipRange[1]..clipRange[3]) is the range of coordinates relative to the
      reference point which are both inside the image (\c camera) and within +/-127. This can be passed top
      \c code2DEdgeUsingSSE.

      \c P0,P1,P2 must be 16byte aligned (because \c projectUsingSSE is internally used).

      If a projection can not be computed (i.e. because the reference is behind or far outside the camera)
      also \c idx<0 is returned.
   */
  void computeProjection(int& idx, int& referenceX, int& referenceY,
                         float P0[4], float P1[4], float P2[4], float clipRange[4],
                         const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera) const;

  //! Tries to load \c vertexList from \c filename
  /*! \c vertexList must already be allocated and all parameter set. If loading fails, \c false is returned.
   */
  bool loadVertexList(const char* filename);

  //! Save \c vertexList such that it can be loaded by \c loadVertexList and \c loadOrCreate.
  void saveVertexList(const char* filename) const;

  //! Returns the memory consumption of \c this
  int memory() const
  {
    int mem = static_cast<int>(sizeof(*this) + vertexList.capacity() * sizeof(vertexList.front()) + object.memory());
    for(size_t i = 0; i < vertexList.size(); i++)
      mem += static_cast<int>(vertexList[i].capacity() * sizeof(vertexList[i].front()));
    return mem;
  }
};
