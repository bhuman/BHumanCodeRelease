#pragma once

#include "ImageProcessing/SIMD.h"
#include <Eigen/Geometry>
#include <vector>

//! A 3-D object in surface representation decomposed into a set of triangles
/*! The representations maintains a face, edge and vertex list with the
    faces and edges referring to vertices by index.
 */
class TriangleMesh
{
public:
  //! When an edge is adjacent to a face of \c IGNORECOLOR it is not assumed
  //! to be present in the image and treated as not visible.
  enum {IGNORECOLOR = 0xff};

  //! A face defined by 3 vertex indices
  /*! The vertices are ordered ccw, when viewed from the outside.
      The color is just an id. Edges between faces of different colors
      are visible in a contour even if viewed from the front. Edges
      with a face of color \c IGNORECOLOR are always treated as invisible.
   */
  class Face
  {
  public:
    int vertex[3];
    int color;
    Face(): color(0) {vertex[0] = vertex[1] = vertex[2] = -1;}
    Face(int vertex0, int vertex1, int vertex2, int color = 0): color(color) {vertex[0] = vertex0; vertex[1] = vertex1; vertex[2] = vertex2;}
    Face(int vertex[3], int color = 0): color(color) {this->vertex[0] = vertex[0]; this->vertex[1] = vertex[1]; this->vertex[2] = vertex[2];}
    bool operator==(const Face& f2) const
    {
      return vertex[0] == f2.vertex[0] && vertex[1] == f2.vertex[1] && vertex[2] == f2.vertex[2] &&
             color == f2.color;
    }
  };

  //! A face defined by 3 vertices (with coordinates)
  /*! The vertices are ordered ccw, when viewed from the outside.
      The color is just an id. Edges between faces of different colors
      are visible in a contour even if viewed from the front.
   */
  class ExplicitFace
  {
  public:
    Eigen::Vector3d vertex[3];
    int color;
    ExplicitFace(): color(0) {}
    ExplicitFace(const Eigen::Vector3d& vertex0, const Eigen::Vector3d& vertex1, const Eigen::Vector3d& vertex2, int color = 0): color(color) {vertex[0] = vertex0; vertex[1] = vertex1; vertex[2] = vertex2;}
    ExplicitFace(const Eigen::Vector3d vertex[3], int color = 0): color(color) {this->vertex[0] = vertex[0]; this->vertex[1] = vertex[1]; this->vertex[2] = vertex[2];}
  };

  //! An edge defined by indices into \c TriangleMesh.vertex with links to faces
  /*! The orientation of the edge is from \c vertex[0] to \c vertex[1] for face[0]
      and from \c vertex[1] to \c vertex[0] for face[1]. The face indices can be
      -1 for half constructed triangle meshes.
   */
  class Edge
  {
  public:
    int vertex[2];
    int face[2];
    Edge() {vertex[0] = vertex[1] = -1; face[0] = face[1] = -1;}
    Edge(int vertex0, int vertex1) {vertex[0] = vertex0; vertex[1] = vertex1; face[0] = face[1] = -1;}
    Edge(int vertex[2]) {this->vertex[0] = vertex[0]; this->vertex[1] = vertex[1]; face[0] = face[1] = -1;}
    bool empty() const {return vertex[0] == -1 && vertex[1] == -1;}
    bool valid() const {return vertex[0] >= 0 && vertex[1] >= 0;}
    void order() {if(vertex[0] > vertex[1]) std::swap(vertex[0], vertex[1]); }
    bool operator==(const Edge& e2) const
    {
      return vertex[0] == e2.vertex[0] && vertex[1] == e2.vertex[1] &&
             face[0] == e2.face[0] && face[1] == e2.face[1];
    }
  };

  //! An edge defined by to vertices with coordinates (no links to faces)
  class ExplicitEdge
  {
  public:
    Eigen::Vector3d vertex[2];
    ExplicitEdge() = default;
    ExplicitEdge(const Eigen::Vector3d& vertex0, const Eigen::Vector3d& vertex1) {vertex[0] = vertex0; vertex[1] = vertex1;}
    ExplicitEdge(const Eigen::Vector3d vertex[2]) {this->vertex[0] = vertex[0]; this->vertex[1] = vertex[1];}
  };

  using EdgeList = std::vector<TriangleMesh::Edge>;

  //! Empty mesh
  TriangleMesh() : isRotationalSymmetricZ(false) {}

  //! See \c computeFrom
  TriangleMesh(const std::vector<ExplicitFace>& face, bool isRotationalSymmetricZ = false, double eps = 0) {computeFrom(face, isRotationalSymmetricZ, eps);}

  //! Creates a triangle mesh from a set of vertices and a set of indices into the vertex set.
  /*! Three \c indices in a row define one triangle. Three entries in a row form one vertex in
   \c vertices. Entries of \c indices are multiplied by three to address the first coordinate
   of the corresponding vertex in \c vertices.
   */
  TriangleMesh(const std::vector<float>& vertices, const std::vector<int>& indices, bool isRotationalSymmetricZ = false, int extraColors = 0);

  TriangleMesh(const char* filename)
  {if(!load(filename)) abort();}

  //! Is it empty or is thereat least one triangle?
  bool empty() const {return face.empty();}

  //! Return to empty state
  void clear() {vertex.clear(); face.clear();}

  //! create a triangle mesh from numerically given triangles
  /*! vertices <= than eps apart are joined as one formal vertex
      The triangle mesh needs to be regular.
   */
  void computeFrom(const std::vector<ExplicitFace>& face, bool isRotationalSymmetricZ = false, double eps = 0);

  //! Computes the contour of this object viewed from \c pointOfView
  /*! The computed contour \c contourEdge is the list of edge, which are visible when
      viewing the object from \c pointOfView. An edge is visible, if it is not
      occluded, and either one adjacent face is viewed from the front and one
      from the back (real contour) or both are viewed from the front but have
      different \c .color values (texture contour).

      At the moment the implementation does not handle occlusion.

      The edge list is not sorted.
   */
  void computeContour(EdgeList& contourEdge, const Eigen::Vector3d& pointOfView) const;

  //! Overloaded function
  void computeContour(std::vector<ExplicitEdge>& edge, const Eigen::Vector3d& pointOfView) const
  {
    EdgeList implicitEdge;
    computeContour(implicitEdge, pointOfView);
    edge.resize(implicitEdge.size());
    for(int i = 0; i < static_cast<int>(implicitEdge.size()); i++)
      edge[i] = explicitEdge(implicitEdge[i]);
  }

  //! Converts \c Edge to \c ExplicitEdge by filling in coordinates
  ExplicitEdge explicitEdge(const Edge& edge) const {return ExplicitEdge(vertexOfIndex(edge.vertex[0]), vertexOfIndex(edge.vertex[1]));};

  //! Converts \c Face to \c ExplicitFace by filling in coordinates
  ExplicitFace explicitFace(const Face& face) const {return ExplicitFace(vertexOfIndex(face.vertex[0]), vertexOfIndex(face.vertex[1]), vertexOfIndex(face.vertex[2]));};

  //! Vertex with a given number and an empty vector if outside the range
  Eigen::Vector3d& vertexOfIndex(int index)
  {
    static Eigen::Vector3d emptyVertex;
    if(0 <= index && index < static_cast<int>(vertex.size()))
      return vertex[index];
    else
      return emptyVertex;
  }

  //! Vertex with a given number and an empty vector if outside the range
  const Eigen::Vector3d& vertexOfIndex(int index) const
  {
    static Eigen::Vector3d emptyVertex;
    if(0 <= index && index < static_cast<int>(vertex.size()))
      return vertex[index];
    else
      return emptyVertex;
  }

  //! Searches for \c v in the vertex list, adds it if necessary and returns its index
  /*! The comparison uses tolerance \c eps.
   */
  int indexOfVertexAndAdd(const Eigen::Vector3d& v, double eps = 0)
  {
    int index = indexOfVertex(v, eps);
    if(index >= 0)
      return index;
    else
    {
      vertex.push_back(v);
      return static_cast<int>(vertex.size()) - 1;
    }
  }

  //! Searches for \c v in the vertex list and returns its index
  /*! The comparison uses tolerance \c eps. If \c v is not found,
      \c -1 is returned
   */
  int indexOfVertex(const Eigen::Vector3d& v, double eps = 0) const;

  //! Adds an edge pointing from vertex0 to vertex1 and being part of \c face
  /*! The routine returns the index of the edge.
    The routine looks, whether the is an edge already involving these two vertices
    and stores \c face in that Edge. \c vertex0-vertex1 must be ccw in the face.
    The routine correctly handles that there are two faces for each edge with opposite
    direction of ccw travel. If there are more than two faces, an assertion is thrown.
   */
  int addEdge(int vertex0, int vertex1, int face);

  //! Add a face to the list of faces (with edges and vertices)
  /*! The routine identifies, whether the vertices of \c face
      are already part of the vertex list (tolerance \c eps).
      If this is the case it uses the existing indices.
   */
  void add(const ExplicitFace& newFace, double eps = 0);

  //! Overloaded function
  /*! p0-p1-p2 must be in ccw order when viewed from the outside. */
  void add(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int color, double eps = 0)
  {add(ExplicitFace(p0, p1, p2, color), eps);  }

  //! Adds a quadrangle by adding two triangles
  /*! p0-p1-p2-p3 must be in ccw order when viewed from the outside. */
  void addQuadrangle(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, int color, double eps = 0);

  //! Adds another triangle mesh to this one
  /*! No merging is performed. The method might not work if both meshes contain identical edges. */
  void addTriangleMesh(const TriangleMesh& other);

  //! Returns the outward pointing normal of \c face[i]
  Eigen::Vector3d normal(int i) const
  {
    Eigen::Vector3d p0 = vertex[face[i].vertex[0]];
    Eigen::Vector3d p1 = vertex[face[i].vertex[1]];
    Eigen::Vector3d p2 = vertex[face[i].vertex[2]];
    return (p1 - p0).cross(p2 - p0);
  }

  //! Returns, whether from \c pointOfView \c faceIdx is visible
  /*! At the moment, occlusion is ignored. The function just returns
      whether \c pointOfView is on the \c outside of \c faceIdx.
      If \c faceIdx is <0, \c false is returned.
   */
  bool isFaceVisible(int faceIdx, const Eigen::Vector3d& pointOfView) const
  {
    if(0 <= faceIdx && faceIdx < static_cast<int>(face.size()))
      return isFaceVisible(face[faceIdx], pointOfView);
    else
      return false;
  }

  //! Returns, whether from \c pointOfView the outside of \c face is visible
  /*! At the moment, occlusion is ignored.
   */
  bool isFaceVisible(const Face& face, const Eigen::Vector3d& pointOfView) const
  {
    Eigen::Vector3d p0 = vertex[face.vertex[0]];
    Eigen::Vector3d p1 = vertex[face.vertex[1]];
    Eigen::Vector3d p2 = vertex[face.vertex[2]];
    return (p1 - p0).cross(p2 - p0).dot(pointOfView - p0) > 0;
  }

  //! Checks that every edge has two adjacent faces
  bool isComplete() const;

  //! checks that all indices are within range
  bool indicesValid() const ;

  //! Should be called after adding all vertices/edges/faces
  /*! Checks that nothing is missing and computes derived quantities (e.g. \c middle and \c radius)
   */
  void close()
  {
    assert(isComplete());
    assert(indicesValid());
    computeMiddleAndRadius();
  }

  //! Computes \c middle and \c radius.
  void computeMiddleAndRadius()
  {
    Eigen::Vector3d sum(0, 0, 0);
    for(int i = 0; i < static_cast<int>(vertex.size()); i++)
      sum += vertex[i];
    middle = sum / static_cast<double>(vertex.size());

    radius  = 0;
    for(int i = 0; i < static_cast<int>(vertex.size()); i++)
      radius = std::max(radius, (middle - vertex[i]).norm());
  }

  std::vector<Eigen::Vector3d> vertex;
  std::vector<Edge> edge;
  std::vector<Face> face;

  //! Middle of the object
  /*! This is the average vector of all vertices. Since it refers to the vertices,
      not to the actual volume it is not the center of gravity but reasonably in
      the middle of the object.
   */
  Eigen::Vector3d middle;

  //! Largest distance of a vertex to the cog
  /*! Is computed by \c computeRadius */
  double radius;

  //! True if the object described by the triangle mesh is rotationally symmetric around Z-axis
  /*! This information can be exploited by algorithms operating on the object (in particular \c LutRasterizer).
      If \c isRotationalSymmetricZ==true, the original ideal object approximated by this meshes
      is rotationally symmetric around Z=0. The actual mesh is of course never exactly symmetric
      under rotation but this is viewed as an approximation.
   */
  bool isRotationalSymmetricZ;
  bool isRotationalSymmetric = false;

  //! Computes a triangle mesh approximation of a cylinder
  /*! The cylinders axis is Z, \c radius given, the top and bottom planes are Z=zBottom and Z=zTop.
      (\c zBottom<zTop).
      The approximation uses \c nCircular points along each circle and nAxial
      triangles in axial direction. \c nAxial>1 can be used for special purposes
      but does not actually increase the approximation quality as a cylinder is
      "flat" in axial direction.

      The cylinder is closed, i.e. has a top and bottom face. The outline has color 0, top 1, bottom 2.
   */
  static TriangleMesh cylinder(double radius, double zBottom, double zTop, int nCircular = 32, int nAxial = 1,
                               int colorCircumference = 0, int colorBottom = 1, int colorTop = 2);

  static TriangleMesh sphere(double radius, int nSubDivide = 3, int color = 0);

  //! Returns the memory consumption of \c this
  int memory() const
  {
    return static_cast<int>(sizeof(*this)
                            + vertex.capacity() * sizeof(vertex.front())
                            + edge.capacity() * sizeof(edge.front()) +
                            + face.capacity() * sizeof(face.front()));
  }

  //! Saves this mesh into \c filename in a proprietary text format
  /*! The return code specifies success. */
  bool save(const char* filename) const;

  //! Loads this mesh from \c filename
  /*! The return code specifies success. */
  bool load(const char* filename);
};

inline std::ostream& operator<<(std::ostream& o, const TriangleMesh::Edge& edge)
{
  o << "[" << edge.vertex[0] << " " << edge.vertex[1] << " " << edge.face[0] << " " << edge.face[1] << "]";
  return o;
}

inline std::ostream& operator<<(std::ostream& o, const TriangleMesh::Face& face)
{
  o << "[" << face.vertex[0] << " " << face.vertex[1] << " " << face.vertex[2] << " " << face.color << "]";
  return o;
}

class LessOnEdges
{
public:
  bool operator()(const TriangleMesh::Edge& a, const TriangleMesh::Edge& b) const
  {
    TriangleMesh::Edge aOrdered(a), bOrdered(b);
    aOrdered.order();
    bOrdered.order();
    return aOrdered.vertex[0] < bOrdered.vertex[0] ||
           (aOrdered.vertex[0] == bOrdered.vertex[0] && aOrdered.vertex[1] < bOrdered.vertex[1]);
  }
};

inline std::ostream& operator<<(std::ostream& os, const TriangleMesh::EdgeList& el)
{
  TriangleMesh::EdgeList el2(el);
  sort(el2.begin(), el2.end(), LessOnEdges());
  os << "{";
  for(int i = 0; i < static_cast<int>(el2.size()); i++)
  {
    if(i > 0)
      os << ", ";
    TriangleMesh::Edge el2IOrdered(el2[i]);
    el2IOrdered.order();
    os << el2IOrdered;
  }
  os << "}";
  return os;
}
