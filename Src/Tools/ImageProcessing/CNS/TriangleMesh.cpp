#include "TriangleMesh.h"

using namespace std;

TriangleMesh::TriangleMesh(const std::vector<float>& vertices, const std::vector<int>& indices, bool isRotationalSymmetricZ, int extraColors) :
  isRotationalSymmetricZ(isRotationalSymmetricZ)
{
  extraColors += 1 - static_cast<int>(vertices.size()) / 3;
  int currentColor = 0;
  for(unsigned i = 0; i < indices.size(); i += indices[i] < 0 ? 1 : 3)
  {
    if(indices[i] < 0)
    {
      currentColor = -indices[i];
      continue;
    }
    const int* triangle = &indices[i];
    Eigen::Vector3d points[3];
    int color = currentColor;
    for(int j = 0; j < 3; ++j)
    {
      int index = triangle[j];
      color = std::max(color, index + extraColors);
      const float* vertex = &vertices[3 * index];
      points[j] = Eigen::Vector3d(vertex[0], vertex[1], vertex[2]);
    }
    double maxNorm = max(max((points[0] - points[1]).norm(), (points[1] - points[2]).norm()), (points[2] - points[0]).norm());
    add(points[0], points[1], points[2], color, 1E-3 * maxNorm);
  }
  close();
}

void TriangleMesh::computeContour(vector<Edge>& contourEdge, const Eigen::Vector3d& pointOfView) const
{
  contourEdge.clear();
  for(int i = 0; i < static_cast<int>(edge.size()); i++)
  {
    const Edge& e = edge[i];
    const Face& face0 = face[e.face[0]];
    const Face& face1 = face[e.face[1]];
    if(face0.color == IGNORECOLOR || face1.color == IGNORECOLOR)
      continue;
    bool visible0 = isFaceVisible(e.face[0], pointOfView);
    bool visible1 = isFaceVisible(e.face[1], pointOfView);
    if(visible0 != visible1 ||
       (visible0 && visible1 && face0.color != face1.color))
    {
      contourEdge.push_back(e);
    }
  }
}

void TriangleMesh::computeFrom(const vector<ExplicitFace>& face, bool isRotationalSymmetricZ, double eps)
{
  clear();
  this->isRotationalSymmetricZ = isRotationalSymmetricZ;
  for(int i = 0; i < static_cast<int>(face.size()); i++)
    add(face[i], eps);
}

int TriangleMesh::indexOfVertex(const Eigen::Vector3d& v, double eps) const
{
  double eps2 = eps * eps;
  for(int i = 0; i < static_cast<int>(vertex.size()); i++)
    if((v - vertex[i]).squaredNorm() <= eps2)
      return i;
  return -1;
}

void TriangleMesh::add(const ExplicitFace& newFace, double eps)
{
  Face f;
  f.color = newFace.color;
  for(int i = 0; i < 3; i++)
    f.vertex[i] = indexOfVertexAndAdd(newFace. vertex[i], eps);
  addEdge(f.vertex[0], f.vertex[1], static_cast<int>(face.size()));
  addEdge(f.vertex[1], f.vertex[2], static_cast<int>(face.size()));
  addEdge(f.vertex[2], f.vertex[0], static_cast<int>(face.size()));
  face.push_back(f);
}

void TriangleMesh::addQuadrangle(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, int color, double eps)
{
  add(p0, p1, p3, color, eps);
  add(p1, p2, p3, color, eps);
}

void TriangleMesh::addTriangleMesh(const TriangleMesh& other)
{
  int maxColor = -1;
  for(std::vector<Face>::const_iterator f = face.begin(); f != face.end(); ++f)
    maxColor = std::max(maxColor, f->color);
  for(std::vector<Face>::const_iterator f = other.face.begin(); f != other.face.end(); ++f)
  {
    ExplicitFace ef = other.explicitFace(*f);
    ef.color = f->color + maxColor + 1;
    add(ef);
  }
  computeMiddleAndRadius();
}

int TriangleMesh::addEdge(int vertex0, int vertex1, int face)
{
  for(int i = 0; i < static_cast<int>(edge.size()); i++)
  {
    Edge& e = edge[i];
    if(e.vertex[0] == vertex0 && e.vertex[1] == vertex1) // found the edge already
    {
      assert(e.face[0] == -1);
      e.face[0] = face;
      return i;
    }
    else if(e.vertex[0] == vertex1 && e.vertex[1] == vertex0) // found the reverse edge
    {
      assert(e.face[1] == -1);
      e.face[1] = face;
      return i;
    }
  }
  // Found no edge, add new
  edge.push_back(Edge(vertex0, vertex1));
  edge.back().face[0] = face;
  return static_cast<int>(edge.size() - 1);
}

TriangleMesh TriangleMesh::cylinder(double radius, double zBottom, double zTop, int nCircular, int nAxial,
                                    int colorCircumference, int colorBottom, int colorTop)
{
  assert(zBottom < zTop);
  TriangleMesh tm;
  tm.isRotationalSymmetricZ = true;
  double eps = max(radius / nCircular, (zTop - zBottom) / nAxial) * 1E-3;
  for(int i = 0; i < nAxial; i++)
    for(int j = 0; j < nCircular; j++)
    {
      double angle0 = j * 2 * M_PI / nCircular, angle1 = (j + 1) * 2 * M_PI / nCircular;
      double x0 = cos(angle0) * radius, y0 = sin(angle0) * radius;
      double x1 = cos(angle1) * radius, y1 = sin(angle1) * radius;
      double z0 = zBottom + i * (zTop - zBottom) / nAxial, z1 = zBottom + (i + 1) * (zTop - zBottom) / nAxial;
      tm.addQuadrangle(Eigen::Vector3d(x0, y0, z0), Eigen::Vector3d(x1, y1, z0), Eigen::Vector3d(x1, y1, z1), Eigen::Vector3d(x0, y0, z1), colorCircumference, eps);
      if(i == 0)  // top and bottom face
      {
        tm.add(Eigen::Vector3d(x0, y0, zTop), Eigen::Vector3d(x1, y1, zTop), Eigen::Vector3d(0, 0, zTop), colorTop, eps);
        tm.add(Eigen::Vector3d(x1, y1, zBottom), Eigen::Vector3d(x0, y0, zBottom), Eigen::Vector3d(0, 0, zBottom), colorBottom, eps);
      }
    }
  tm.close();
  return tm;
}

TriangleMesh TriangleMesh::sphere(double radius, int nSubDivide, int color)
{
  struct Triangle
  {
    Eigen::Vector3d v1, v2, v3;
    Triangle() = default;
    Triangle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3)
      : v1(v1), v2(v2), v3(v3) {}
  };

  std::vector<Triangle> tris;
  const double r = radius / sqrt(3.);

  // create cube
  // create cube
  //top
  tris.push_back(Triangle(Eigen::Vector3d(r, r, r), Eigen::Vector3d(-r, r, -r), Eigen::Vector3d(r, r, -r)));
  tris.push_back(Triangle(Eigen::Vector3d(-r, r, -r), Eigen::Vector3d(r, r, r), Eigen::Vector3d(-r, r, r)));
  //bottom
  tris.push_back(Triangle(Eigen::Vector3d(r, -r, r), Eigen::Vector3d(-r, -r, -r), Eigen::Vector3d(r, -r, -r)));
  tris.push_back(Triangle(Eigen::Vector3d(-r, -r, -r), Eigen::Vector3d(r, -r, r), Eigen::Vector3d(-r, -r, r)));

  //right
  tris.push_back(Triangle(Eigen::Vector3d(r, r, r), Eigen::Vector3d(r, -r, -r), Eigen::Vector3d(r, r, -r)));
  tris.push_back(Triangle(Eigen::Vector3d(r, -r, -r), Eigen::Vector3d(r, r, r), Eigen::Vector3d(r, -r, r)));
  //left
  tris.push_back(Triangle(Eigen::Vector3d(-r, r, r), Eigen::Vector3d(-r, -r, -r), Eigen::Vector3d(-r, r, -r)));
  tris.push_back(Triangle(Eigen::Vector3d(-r, -r, -r), Eigen::Vector3d(-r, r, r), Eigen::Vector3d(-r, -r, r)));

  //front
  tris.push_back(Triangle(Eigen::Vector3d(r, r, r), Eigen::Vector3d(-r, -r, r), Eigen::Vector3d(r, -r, r)));
  tris.push_back(Triangle(Eigen::Vector3d(-r, -r, r), Eigen::Vector3d(r, r, r), Eigen::Vector3d(-r, r, r)));
  //rear
  tris.push_back(Triangle(Eigen::Vector3d(r, r, -r), Eigen::Vector3d(-r, -r, -r), Eigen::Vector3d(r, -r, -r)));
  tris.push_back(Triangle(Eigen::Vector3d(-r, -r, -r), Eigen::Vector3d(r, r, -r), Eigen::Vector3d(-r, r, -r)));

  for(int i = 0; i < nSubDivide; ++i)
  {
    std::vector<Triangle> newTris;
    for(const Triangle& t : tris)
    {
      Eigen::Vector3d mid = (t.v1 + t.v2).normalized() * radius;
      newTris.push_back(Triangle(t.v2, t.v3, mid));
      newTris.push_back(Triangle(t.v1, t.v3, mid));
    }
    tris.swap(newTris);
  }

  TriangleMesh tm;
  tm.isRotationalSymmetric = tm.isRotationalSymmetricZ = true;
  double eps = radius / ((nSubDivide * 2 / 3) << 4) * 1E-3;
  for(const Triangle& t : tris)
  {
    Eigen::Vector3d n = (t.v2 - t.v1).cross(t.v3 - t.v1);
    Eigen::Vector3d a = t.v1 + t.v2 + t.v3;
    if(n.normalized().dot(a.normalized()) > 0)
      tm.add(t.v1, t.v2, t.v3, color, eps);
    else
      tm.add(t.v1, t.v3, t.v2, color, eps);
  }
  tm.close();
  return tm;
}

bool TriangleMesh::save(const char* filename) const
{
  FILE* f = fopen(filename, "w");
  if(f == nullptr)
    return false;

  if(fprintf(f, "isRotationalSymmetricZ= %d\n", isRotationalSymmetricZ) == 0)
    return false;

  if(fprintf(f, "isRotationalSymmetric= %d\n", isRotationalSymmetric) == 0)
    return false;

  for(int i = 0; i < static_cast<int>(vertex.size()); i++)
    if(fprintf(f, "vertex+= %d %15.10e %15.10e %15.10e\n", i, vertex[i][0], vertex[i][1], vertex[i][2]) == 0)
      return false;

  if(fprintf(f, "\n") == 0)
    return false;

  for(int i = 0; i < static_cast<int>(edge.size()); i++)
    if(fprintf(f, "edge+= %d %d %d %d %d\n", i, edge[i].vertex[0], edge[i].vertex[1], edge[i].face[0], edge[i].face[1]) == 0)
      return false;

  if(fprintf(f, "\n") == 0)
    return false;

  for(int i = 0; i < static_cast<int>(face.size()); i++)
    if(fprintf(f, "face+= %d %d %d %d %d\n", i, face[i].vertex[0], face[i].vertex[1], face[i].vertex[2], face[i].color) == 0)
      return false;

  fclose(f);
  return true;
}

bool TriangleMesh::load(const char* filename)
{
#ifndef WINDOWS
  FILE* f = fopen(filename, "r");
  if(f == nullptr)
    return false;

  clear();
  while(!feof(f))
  {
    char* s = 0;
    size_t n = 0;
    long ret = getline(&s, &n, f);
    if(s == nullptr || ret == -1)
      break;
    int tmp, idx;
    Eigen::Vector3d p;
    Edge e;
    Face fc;
    //    printf(s);
    if(strcmp(s, "\n") == 0)
    {
    }
    else if(sscanf(s, "isRotationalSymmetricZ= %d", &tmp) == 1)
    {
      isRotationalSymmetricZ = static_cast<bool>(tmp);
    }
    else if(sscanf(s, "isRotationalSymmetric= %d", &tmp) == 1)
    {
      isRotationalSymmetric = static_cast<bool>(tmp);
    }
    else if(sscanf(s, "vertex+=%d %lf %lf %lf", &idx, &p[0], &p[1], &p[2]) == 4)
    {
      assert(idx >= 0);
      if(idx >= static_cast<int>(vertex.size()))
        vertex.resize(idx + 1);
      vertex[idx] = p;
    }
    else if(sscanf(s, "edge+=%d %d %d %d %d", &idx, &e.vertex[0], &e.vertex[1], &e.face[0], &e.face[1]) == 5)
    {
      assert(idx >= 0);
      if(idx >= static_cast<int>(edge.size()))
        edge.resize(idx + 1);
      edge[idx] = e;
    }
    else if(sscanf(s, "face+=%d %d %d %d %d", &idx, &fc.vertex[0], &fc.vertex[1], &fc.vertex[2], &fc.color) == 5)
    {
      assert(idx >= 0);
      if(idx >= static_cast<int>(face.size()))
        face.resize(idx + 1);
      face[idx] = fc;
    }
    else
    {
      fprintf(stderr, "Illegal line in 3d file\n%s\n", s);
      fclose(f);
      return false;
    }
    if(s != nullptr)
      free(s);
  }

  close();
  fclose(f);
  return true;
#else
  return false;
#endif
}

bool TriangleMesh::indicesValid() const
{
  for(int i = 0; i < static_cast<int>(edge.size()); i++)
  {
    if(edge[i].vertex[0] < 0 || edge[i].vertex[0] >= static_cast<int>(vertex.size()))
      return false;
    if(edge[i].vertex[1] < 0 || edge[i].vertex[1] >= static_cast<int>(vertex.size()))
      return false;
    if(edge[i].face  [0] < 0 || edge[i].face  [0] >= static_cast<int>(face.size()))
      return false;
    if(edge[i].face  [1] < 0 || edge[i].face  [1] >= static_cast<int>(face.size()))
      return false;
  }
  for(int i = 0; i < static_cast<int>(face.size()); i++)
  {
    if(face[i].vertex[0] < 0 || face[i].vertex[0] >= static_cast<int>(vertex.size()))
      return false;
    if(face[i].vertex[1] < 0 || face[i].vertex[1] >= static_cast<int>(vertex.size()))
      return false;
    if(face[i].vertex[2] < 0 || face[i].vertex[2] >= static_cast<int>(vertex.size()))
      return false;
  }
  return true;
}

bool TriangleMesh::isComplete() const
{
  for(int i = 0; i < static_cast<int>(edge.size()); i++)
  {
    if(edge[i].face[0] < 0 || edge[i].face[1] < 0)
    {
      return false;
    }
  }
  return true;
}
