#include "LutRasterizer.h"
#include "CNSSSE.h"

using namespace std;

void LutRasterizer::create(const TriangleMesh& object, const Eigen::AlignedBox3d& viewpointRange, double spacing)
{
  setObject(object);
  allocateLut(viewpointRange, spacing);
  for(int idx = 0; idx < static_cast<int>(vertexList.size()); idx++)
    computeVertexList(vertexList[idx], object, viewpointOfIndex(idx));
}

void LutRasterizer::loadOrCreate(const TriangleMesh& object, const Eigen::AlignedBox3d& viewpointRange, double spacing, const char* filename)
{
  setObject(object);
  allocateLut(viewpointRange, spacing);
  if(!loadVertexList(filename))
  {
    for(int idx = 0; idx < static_cast<int>(vertexList.size()); idx++)
      computeVertexList(vertexList[idx], object, viewpointOfIndex(idx));
    if(filename != nullptr)
      saveVertexList(filename);
  }
}

bool LutRasterizer::loadVertexList(const char* filename)
{
  FILE* f = fopen(filename, "rb");
  if(f == nullptr)
    return false;
  for(int idx = 0; idx < static_cast<int>(vertexList.size()); idx++)
  {
    int ctr;
    fread(&ctr, sizeof(ctr), 1, f);
    vertexList[idx].resize(ctr);
    if(ctr > 0)
      fread(&vertexList[idx][0], sizeof(vertexList[idx][0]), ctr, f);
    if(feof(f) || ferror(f))
    {
      vertexList.clear();
      fclose(f);
      return false;
    }
  }
  char x;
  if(fread(&x, sizeof(x), 1, f) != 0)
  {
    vertexList.clear();
    return false; // file should have an end
  }
  fclose(f);
  return true;
}

void LutRasterizer::saveVertexList(const char* filename) const
{
  FILE* f = fopen(filename, "wb");
  for(int idx = 0; idx < static_cast<int>(vertexList.size()); idx++)
  {
    int ctr = static_cast<int>(vertexList[idx].size());
    fwrite(&ctr, sizeof(ctr), 1, f);
    fwrite(&vertexList[idx][0], sizeof(vertexList[idx][0]), ctr, f);
  }
  fclose(f);
}

void LutRasterizer::allocateLut(const Eigen::AlignedBox3d& viewpointRange, double spacing)
{
  Eigen::Vector3d point0X, point1X;
  assert(spacing > 0);
  this->viewpointRange = viewpointRange;
  this->spacing = spacing;
  if(object.isRotationalSymmetricZ)
  {
    // Compute max X coordinate that can come from normalizing inside point0-point1
    double xMax = max(fabs(viewpointRange.min()(0)), fabs(viewpointRange.max()(0)));
    double yMax = max(fabs(viewpointRange.min()(1)), fabs(viewpointRange.max()(1)));
    double rMax = sqrt(xMax * xMax + yMax * yMax);

    point0X(0) = point0X(1) = point1X(1) = 0;
    point1X(0) = rMax;
    point0X(2) = viewpointRange.min()(2);
    point1X(2) = viewpointRange.max()(2);
  }
  else
  {
    point0X = viewpointRange.min();
    point1X = viewpointRange.max();
  }
  double eps = 1e-3;
  for(int i = 0; i < 3; i++)
  {
    basePoint[i] = min(point0X[i], point1X[i]);
    vertexListSize[i] = static_cast<int>(ceil(fabs(point1X[i] - point0X[i]) / spacing - eps + 1));
  }
  vertexList.resize(vertexListSize[0] * vertexListSize[1] * vertexListSize[2]);
}

void LutRasterizer::countVertices(vector<int>& counter, const TriangleMesh::EdgeList& el)
{
  int n = 0;
  for(int i = 0; i < static_cast<int>(el.size()); i++)
  {
    const TriangleMesh::Edge& e = el[i];
    assert(e.valid());
    if(e.vertex[0] > n)
      n = e.vertex[0];
    if(e.vertex[1] > n)
      n = e.vertex[1];
  }
  n++;
  counter = vector<int>(n, 0);
  for(int i = 0; i < static_cast<int>(el.size()); i++)
  {
    const TriangleMesh::Edge& e = el[i];
    assert(e.valid());
    counter[e.vertex[0]]++;
    counter[e.vertex[1]]++;
  }
}

void LutRasterizer::convertEdgeListToVertexList(LutRasterizer::VertexList& vertexList, const TriangleMesh::EdgeList& edgeList)
{
  vertexList.clear();
  TriangleMesh::EdgeList el(edgeList);  // need to make a copy since we remove processed edges from \c el
  vector<int> counter;
  countVertices(counter, el);
  while(true)
  {
    bool found = false;
    if(!vertexList.empty())     // Look for edge from el.back
    {
      for(int i = 0; i < static_cast<int>(el.size()); i++)
        if(el[i].valid())
        {
          if(el[i].vertex[0] == vertexList.back())  // Take edge i from 0 to 1
          {
            vertexList.push_back(static_cast<unsigned char>(el[i].vertex[1]));
            counter[el[i].vertex[0]]--;
            counter[el[i].vertex[1]]--;
            el[i] = TriangleMesh::Edge();
            found = true;
            break;
          }
          else if(el[i].vertex[1] == vertexList.back())  // Take edge i from 1 to 0
          {
            vertexList.push_back(static_cast<unsigned char>(el[i].vertex[0]));
            counter[el[i].vertex[0]]--;
            counter[el[i].vertex[1]]--;
            el[i] = TriangleMesh::Edge();
            found = true;
            break;
          }
        }
    }
    if(!found)    // No edge found to continue, look for solo starting edge
    {
      for(int i = 0; i < static_cast<int>(el.size()); i++)
        if(el[i].valid())
        {
          if(counter[el[i].vertex[0]] == 1)  // Start at 0
          {
            if(!vertexList.empty())
              vertexList.push_back(NEWSTART);
            vertexList.push_back(static_cast<unsigned char>(el[i].vertex[0]));
            found = true;
            break;
          }
        }
    }
    if(!found)    // No solo edge found, simply take first edge
    {
      for(int i = 0; i < static_cast<int>(el.size()); i++)
        if(el[i].valid())
        {
          if(!vertexList.empty())
            vertexList.push_back(NEWSTART);
          vertexList.push_back(static_cast<unsigned char>(el[i].vertex[0]));
          found = true;
          break;
        }
    }
    if(!found)
      break;  // All edges processed
  }
  for(int i = 0; i < static_cast<int>(counter.size()); i++)
    assert(counter[i] == 0);
}

void LutRasterizer::computeVertexList(LutRasterizer::VertexList& vertexList, const TriangleMesh& object, const Eigen::Vector3d& viewpoint)
{
  TriangleMesh::EdgeList el;
  object.computeContour(el, viewpoint);
  convertEdgeListToVertexList(vertexList, el);
}

void LutRasterizer::convertVertexListToEdgeList(TriangleMesh::EdgeList& edgeList, LutRasterizer::VertexList& vertexList)
{
  edgeList.clear();
  for(int i = 0; i < static_cast<int>(vertexList.size()) - 1; i++)
    if(vertexList[i] != NEWSTART && vertexList[i + 1] != NEWSTART)
      edgeList.push_back(TriangleMesh::Edge(vertexList[i], vertexList[i + 1]));
}

void LutRasterizer::setObject(const TriangleMesh& object)
{
  assert(object.vertex.size() <= NEWSTART); // We only have 8 bit for vertex indices
  this->object = object;
  vertexList.clear();
  vertexWith1.resize(object.vertex.size());
  for(int i = 0; i < static_cast<int>(vertexWith1.size()); i++)
  {
    vertexWith1[i](0) = static_cast<float>(object.vertex[i](0));
    vertexWith1[i](1) = static_cast<float>(object.vertex[i](1));
    vertexWith1[i](2) = static_cast<float>(object.vertex[i](2));
    vertexWith1[i](3) = 1.f;
  }
  computeCenterPoint();
}

void LutRasterizer::computeCenterPoint()
{
  if(vertexWith1.empty())
    centerWith1 = Eigen::Vector4f(0, 0, 0, 1);
  Eigen::Vector4f minV = vertexWith1[0];
  Eigen::Vector4f maxV = minV;
  for(int i = 1; i < static_cast<int>(vertexWith1.size()); i++)
    for(int j = 0; j < 3; j++)
    {
      minV(j) = min(minV(j), vertexWith1[i](j));
      maxV(j) = max(maxV(j), vertexWith1[i](j));
    }
  centerWith1 = (minV + maxV) / 2;
}

void LutRasterizer::computeProjection(int& idx, int& referenceX, int& referenceY,
                                      float P0[4], float P1[4], float P2[4], float clipRange[4],
                                      const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera) const
{
  assert(!camera.isDistorted);
  Eigen::Isometry3d object2Camera = camera.camera2World.inverse() * object2World;
  normalizeObject2Camera(object2Camera);
  idx = indexOfNormalizedViewpoint(object2Camera.inverse().translation());

  // The matrix is yet without offset
  P0[0] = static_cast<float>(object2Camera(0, 0) * camera.scale_x);
  P0[1] = static_cast<float>(object2Camera(0, 1) * camera.scale_x);
  P0[2] = static_cast<float>(object2Camera(0, 2) * camera.scale_x);
  P0[3] = static_cast<float>(object2Camera(0, 3) * camera.scale_x);

  P1[0] = static_cast<float>(object2Camera(1, 0) * camera.scale_y);
  P1[1] = static_cast<float>(object2Camera(1, 1) * camera.scale_y);
  P1[2] = static_cast<float>(object2Camera(1, 2) * camera.scale_y);
  P1[3] = static_cast<float>(object2Camera(1, 3) * camera.scale_y);

  P2[0] = static_cast<float>(object2Camera(2, 0));
  P2[1] = static_cast<float>(object2Camera(2, 1));
  P2[2] = static_cast<float>(object2Camera(2, 2));
  P2[3] = static_cast<float>(object2Camera(2, 3));

  float centerInImage [2];
  if(!projectUsingSSE(centerInImage, const_cast<float*>(centerWith1.data()), P0, P1, P2))
  {
    idx = -1;
    return;
  }

  referenceX = static_cast<int>(round(centerInImage[0] + camera.offset_x));
  referenceY = static_cast<int>(round(centerInImage[1] + camera.offset_y));

  // Modify P such that centerWith1 is mapped to (0,0)
  float origin[2];
  origin[0] = static_cast<float>(camera.offset_x - referenceX);
  P0[0] += P2[0] * origin[0];
  P0[1] += P2[1] * origin[0];
  P0[2] += P2[2] * origin[0];
  P0[3] += P2[3] * origin[0];

  origin[1] = static_cast<float>(camera.offset_y - referenceY);
  P1[0] += P2[0] * origin[1];
  P1[1] += P2[1] * origin[1];
  P1[2] += P2[2] * origin[1];
  P1[3] += P2[3] * origin[1];

  // Compute clipRange as an intersection
  clipRange[0] = static_cast<float>(max(0 - referenceX, -127));
  clipRange[1] = static_cast<float>(max(0 - referenceY, -127));
  clipRange[2] = static_cast<float>(min(camera.width - referenceX - 1, 127.0));
  clipRange[3] = static_cast<float>(min(camera.height - referenceY - 1, 127.0));
}

void LutRasterizer::rasterize(CodedContour& contour, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera) const
{
  contour.clear();

  // Compute the projection parameters
  int idx;
  alignas(16) float P0[4];
  alignas(16) float P1[4];
  alignas(16) float P2[4];
  alignas(16) float clipRange[4];
  computeProjection(idx, contour.referenceX, contour.referenceY, P0, P1, P2, clipRange, object2World, camera);
  if(idx < 0)
    return;

  // Project all points and store the edges midpoints as CodedContourPoint
  const VertexList& vl = vertexList[idx];
  alignas(16) float p[4]; // First and second point (x,y) of the current edge
  bool isNewEdge = true;
  int clippedCtr = 0;
  for(int i = 0; i < static_cast<int>(vl.size()); i++)
  {
    int vIdx = vl[i];
    if(vIdx != NEWSTART)
    {
      shift4Floats(p);
      assert(0 <= vIdx && vIdx < static_cast<int>(vertexWith1.size()));
      projectUsingSSE(p + 2, const_cast<float*>(vertexWith1[vIdx].data()), P0, P1, P2);
      if(!isNewEdge)
      {
        CodedContourPoint ccp = code2DEdgeUsingSSE(p, clipRange);
        if(ccp != EMPTYCONTOURPOINT)
          contour.push_back(ccp);
        else
          clippedCtr++;
      }
      isNewEdge = false;
    }
    else isNewEdge = true;
  }

  // Scale the coded normal vector according to the number of points
  // This is needed so the 16bit accumulator in \c responseX8YRUsingSSE3 does not overflow
  int factorI = static_cast<int>(round(factorOfN(static_cast<int>(static_cast<float>(contour.size()) + clippedCtr * contour.mapping.clippedDenom), true) * (256.0 / 127.0)));
  for(int i = 0; i < static_cast<int>(contour.size()); i++)
    scaleNormalVector(contour[i], factorI);

  // TODO: compute mapping
  // This is deferred to later because with sparse rasterization I am not sure how to normalize the response
}
