#include "ObjectCNSStereoDetector.h"
#include "CNSSSE.h"

using namespace std;

void ObjectCNSStereoDetector::create(const TriangleMesh& object, const CameraModelOpenCV& camera,
                                     const Eigen::AlignedBox3d& viewpointRange, double spacing,
                                     const SearchSpecification& spec, const ParametricLaplacian& distribution,
                                     const char* filenameForLutRasterizer)
{
  this->object = object;
  setCamera(camera);
  lr.loadOrCreate(object, viewpointRange, spacing, filenameForLutRasterizer);
  this->distribution = distribution;
  setSearchSpecification(spec);
  assert(distribution.isFlat());
}

void ObjectCNSStereoDetector::setCamera(const CameraModelOpenCV& camera)
{
  this->camera = camera;
}

void ObjectCNSStereoDetector::setSearchSpecification(const SearchSpecification& spec)
{
  this->spec = spec;
}

void ObjectCNSStereoDetector::renderSearchSpace(Contour& ct, const CameraModelOpenCV& renderCamera, int onlyEvery) const
{
  ct.clear();
  assert(static_cast<int>(camera.width) % spec.blockX == 0);
  assert(static_cast<int>(camera.height) % spec.blockY == 0);
  for(int y = 0; y < camera.height; y += onlyEvery * spec.blockY)
    for(int x = 0; x < camera.width; x += onlyEvery * spec.blockY)
      renderBlockAllPoses(ct, x, y, renderCamera, onlyEvery);
}

void ObjectCNSStereoDetector::search(IsometryWithResponses& object2WorldList,
                                     const CNSImage& cns,
                                     int xLo, int xHi, int yLo, int yHi) const
{
  //cnsLeft.assertBuffer(spec.blockY);
  //cnsRight.assertBuffer(spec.blockY);
  IsometryWithResponses oldObject2WorldList = object2WorldList;
  object2WorldList.clear();
  assert(static_cast<int>(camera.width) % spec.blockX == 0);
  assert(static_cast<int>(camera.height) % spec.blockY == 0);

  if(xLo < 0)
    xLo = 0;
  if(xHi > camera.width - 1)
    xHi = static_cast<int>(camera.width) - 1;
  if(yLo < 0)
    yLo = 0;
  if(yHi > camera.height - 1)
    yHi = static_cast<int>(camera.height) - 1;

  // Global search
  assert((xHi - xLo + 1) % spec.blockX == 0);
  assert((yHi - yLo + 1) % spec.blockY == 0);
  for(int y = yLo; y <= yHi; y += spec.blockY)
    for(int x = xLo; x <= xHi; x += spec.blockX)
      searchBlockAllPoses(object2WorldList, cns, x, y);
  if(spec.refineExisting)
    object2WorldList.insert(object2WorldList.end(), oldObject2WorldList.begin(), oldObject2WorldList.end());

  // Refinement
  // TODO: Eliminate duplicates
  if(spec.nRefineIterations > 0)
    for(int i = 0; i < static_cast<int>(object2WorldList.size()); i++)
      refine(cns, object2WorldList[i], spec.nRefineIterations);
  if(!spec.refineExisting)
    object2WorldList.insert(object2WorldList.end(), oldObject2WorldList.begin(), oldObject2WorldList.end());
  sort(object2WorldList.begin(), object2WorldList.end(), MoreOnResponse());
  while(static_cast<int>(object2WorldList.size()) > spec.nResponses)
    object2WorldList.pop_back();
}

void ObjectCNSStereoDetector::renderBlockAllPoses(Contour& ct,
    int x, int y,
    const CameraModelOpenCV& renderCamera, int onlyEvery) const
{
  Eigen::Vector3d p, v;
  camera.image2WorldRay(x + spec.blockX / 2, y + spec.blockY / 2, p, v);
  double minLambda, maxLambda;
  spec.positionSpace.intersectWithRay(minLambda, maxLambda, p, v);
  if(!(minLambda <= maxLambda))
    return;
  Eigen::Vector3d object2WorldTranslation = p + minLambda * v;
  int ctr = 0;
  while((object2WorldTranslation - p).dot(v) <= maxLambda * v.squaredNorm())
  {
    assert((object2WorldTranslation - p).dot(v) >= (minLambda - 1E-3)*v.squaredNorm());

    for(int i = 0; i < static_cast<int>(spec.object2WorldOrientation.size()); i++)
    {
      Eigen::Isometry3d object2World(spec.object2WorldOrientation[i]);
      object2World.translation() = object2WorldTranslation;
      if(ctr % onlyEvery == 0)
        renderBlockFixedPose(ct, object2World, renderCamera);
    }

    object2WorldTranslation = searchStepTranslationViewing(object2WorldTranslation, spec.stepInPixelGlobalDiscretization);
    ctr++;
    assert(ctr < 1000);
  }
}

void ObjectCNSStereoDetector::searchBlockAllPoses(IsometryWithResponses& object2WorldList,
    const CNSImage& cns,
    int x, int y) const
{
  Eigen::Vector3d p, v;
  camera.image2WorldRay(x + spec.blockX / 2, y + spec.blockY / 2, p, v);
  double minLambda, maxLambda;
  spec.positionSpace.intersectWithRay(minLambda, maxLambda, p, v);
  if(!(minLambda <= maxLambda))
    return;
  Eigen::Vector3d object2WorldTranslation = p + minLambda * v;
  int ctr = 0;
  while((object2WorldTranslation - p).dot(v) <= maxLambda * v.squaredNorm())
  {
    assert((object2WorldTranslation - p).dot(v) >= (minLambda - 1E-3)*v.squaredNorm());

    for(int i = 0; i < static_cast<int>(spec.object2WorldOrientation.size()); i++)
    {
      Eigen::Isometry3d object2World(spec.object2WorldOrientation[i]);
      object2World.translation() = object2WorldTranslation;
      IsometryWithResponse result;
      if(object2WorldList.size() == static_cast<size_t>(spec.nResponses))
        result.response = object2WorldList.back().response;
      if(searchBlockFixedPose(result, cns, object2World))
        addToList(object2WorldList, result, spec.nResponses);
    }

    object2WorldTranslation = searchStepTranslationViewing(object2WorldTranslation, spec.stepInPixelGlobalDiscretization);
    ctr++;
    assert(ctr < 1000);
  }
}

void ObjectCNSStereoDetector::renderBlockFixedPose(Contour& ct,
    const Eigen::Isometry3d& object2WorldTry,
    const CameraModelOpenCV& renderCamera) const
{
  CodedContour ctThis;
  lr.rasterize(ctThis, object2WorldTry, renderCamera);
  Contour ctThis2;
  decode(ctThis2, ctThis);
  for(int i = 0; i < static_cast<int>(ctThis2.size()); i++)
  {
    ct.push_back(ctThis2[i]);
    ct.back().x += ctThis2.referenceX - ct.referenceX;
    ct.back().y += ctThis2.referenceY - ct.referenceY;
  }
}

bool ObjectCNSStereoDetector::searchBlockFixedPose(IsometryWithResponse& object2World,
    const CNSImage& cns,
    const Eigen::Isometry3d& object2WorldTry) const
{
  int maxVal = 0, argMaxX = 0, argMaxY = 0;
  responseXYMax(maxVal, argMaxX, argMaxY, cns, object2WorldTry, spec.blockX, spec.blockY);
  double maxF = LinearResponseMapping().finalBin2FinalFloat(static_cast<short>(maxVal));

  if(maxF > object2World.response)
  {
    object2World = IsometryWithResponse(object2WorldTry, maxF);
    updateWithXY(object2World, argMaxX, argMaxY);
    return true;
  }
  else
    return false;
}

double ObjectCNSStereoDetector::response(const CNSImage& cns, const Eigen::Isometry3d& object2World) const
{
  //  int64 t0 = cv::getTickCount();

  CodedContour ct;
  lr.rasterize(ct, object2World, camera);
  return response(cns, ct);
}

double ObjectCNSStereoDetector::response(const CNSImage& cns, const CodedContour& ct) const
{
  //  int64 t0 = cv::getTickCount();

  alignas(16) signed short responseBin[8][8];
  ct.evaluateX8Y8(responseBin, cns);

  double result = ct.mapping.finalBin2FinalFloat(responseBin[0][0]);
  // int64 t1 = cv::getTickCount();
  //  double t = ((double)(t1-t0))/cv::getTickFrequency();
  //  cout << "RESPONSE (using LutRasterizer):" << t*1000 << "ms" << endl;
  return result;
}

void ObjectCNSStereoDetector::responseX16Y16(signed short response[16][16],
    const CNSImage& cns, const Eigen::Isometry3d& object2World) const
{
  CodedContour ct;
  lr.rasterize(ct, object2World, camera);
  ct.evaluateX16Y16(response, cns, -8, -8);
}

void ObjectCNSStereoDetector::responseXYMax(int& maxVal, int& argMaxX, int& argMaxY,
    const CNSImage& cns, const Eigen::Isometry3d& object2World,
    int blockX, int blockY) const
{
  assert((blockX & 0xf) == 0 && (blockY & 0xf) == 0);
  CodedContour ct;
  lr.rasterize(ct, object2World, camera);

  int xMin = -blockX / 2, xMax = blockX / 2;
  int yMin = -blockY / 2, yMax = blockY / 2;
  for(int y = yMin; y < yMax; y += 16)
    for(int x = xMin; x < xMax; x += 16)
    {
      alignas(16) signed short responseBin[16][16];

      ct.evaluateX16Y16(responseBin, cns, x, y);

      int maxI = -0xffff, argMaxI = -1;
      maximumUsingSSE2(maxI, argMaxI, 0, &responseBin[0][0], 16 * 16);
      if(maxI > maxVal)
      {
        maxVal  = maxI;
        argMaxX = x + (argMaxI & 0xf);
        argMaxY = y + (argMaxI >> 4);
      }
    }
}

Eigen::Vector3d ObjectCNSStereoDetector::searchStepTranslationViewing(const Eigen::Vector3d& object2WorldTrans, double stepInPixel) const
{
  double f = (camera.scale_x + camera.scale_y) / 2;
  Eigen::Vector3d object2CameraTrans = camera.camera2World.inverse() * object2WorldTrans;
  double z = object2CameraTrans(2);
  Eigen::Vector3d t = object2CameraTrans * stepInPixel / (object.radius * f / z);
  return object2WorldTrans + camera.camera2World.linear() * t;
}

Eigen::Isometry3d ObjectCNSStereoDetector::searchStepNewObject2Object(const Eigen::Isometry3d& object2World, double stepInPixel, int dimension) const
{
  Eigen::Isometry3d object2Camera = camera.camera2World.inverse() * object2World;
  double f = (camera.scale_x + camera.scale_y) / 2;
  double z = object2Camera.translation()(2);
  if(dimension == DIM_TRANS_VIEWING)  // translation towards averageCamera
  {
    Eigen::Vector3d t = object2Camera.translation() * stepInPixel / (object.radius * f / z);
    t = object2Camera.inverse().linear() * t;
    //    cout << "Translation by " << t.norm() << " " << object2World.translation() << " equals " << stepInPixel << " pixel." << endl;
    return (Eigen::Isometry3d) Eigen::Translation3d(t);
  }
  else if(dimension == DIM_TRANS_IMAGE_X || dimension == DIM_TRANS_IMAGE_Y) // translation in the image plane
  {
    double x0, y0;
    camera.world2Image(object2World.translation(), x0, y0);
    double x1 = x0, y1 = y0;
    if(dimension == DIM_TRANS_IMAGE_X)
      x1 += stepInPixel;
    else
      y1 += stepInPixel;
    //    cout << "Image based translation " << x0 << " " << y0 << " " << x1 << " " << y1 << endl;
    return object2Camera.inverse() * camera.rotationNewCamera2OldCamera(x0, y0, x1, y1) * object2Camera;
  }
  else // rotation around x/y/z in object coordinates
  {
    double alpha  = stepInPixel * z / (f * object.radius);
    //    cout << "Rotation by " << alpha/M_PI*180 << " degrees equals " << stepInPixel << " pixel." << endl;
    if(dimension == DIM_ROT_Z)
      return Eigen::Isometry3d(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()));
    else if(dimension == DIM_ROT_Y)
      return Eigen::Isometry3d(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()));
    else if(dimension == DIM_ROT_X)
      return Eigen::Isometry3d(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));
    else
    {
      assert(false);
      return Eigen::Isometry3d();
    }
  }
}

bool ObjectCNSStereoDetector::subpixelMaximum(float& max, float argMax[3], const signed short response[3][16][16]) const
{
  bool isSubpixel;
  // Find discrete max
  int maxI = -0xffff, argMaxI = -1, argMaxI2[3];
  maximumUsingSSE2(maxI, argMaxI, 0, &response[0][0][0], 3 * 16 * 16);
  assert(maxI >= -0x7fff);
  // Subpixelinterpolate max
  max         = static_cast<float>(maxI);
  argMaxI2[0] = (argMaxI & 0xf);
  argMaxI2[1] = ((argMaxI >> 4) & 0xf);
  argMaxI2[2] = (argMaxI >> 8);
  if(0 < argMaxI2[0] && argMaxI2[0] < 15 && 0 < argMaxI2[1] && argMaxI2[1] < 15 && argMaxI2[2] == 1
     && subpixelRefine(max, argMax, (&response[0][0][0]) + argMaxI))
  {
    // The maximum is in the interior of the tabulated responses, we can do subpixel-refinement
    // and subpixel-refinement succeeded
    argMax[0] += argMaxI2[0];
    argMax[1] += argMaxI2[1];
    argMax[2] += argMaxI2[2];
    isSubpixel = true;
  }
  else
  {
    max = static_cast<float>(maxI);
    argMax[0] = static_cast<float>(argMaxI2[0]);
    argMax[1] = static_cast<float>(argMaxI2[1]);
    argMax[2] = static_cast<float>(argMaxI2[2]);
    isSubpixel = false;
  }

  assert(0 <= argMax[0] && argMax[0] < 16);
  assert(0 <= argMax[1] && argMax[1] < 16);
  assert(0 <= argMax[2] && argMax[2] < 3);
  //error = !(0<=argMax[0] && argMax[0]<16 && 0<=argMax[1] && argMax[1]<16 && 0<=argMax[2] && argMax[2]<3);

  return isSubpixel;
}

void ObjectCNSStereoDetector::updateWithXY(IsometryWithResponse& object2World, double dX, double dY) const
{
  double x0, y0;
  camera.world2Image(object2World.translation(), x0, y0);
  Eigen::Isometry3d newObject2World =
    camera.rotationNewCamera2OldCamera(x0, y0, x0 + dX, y0 + dY).inverse() *
    object2World;
  object2World = IsometryWithResponse(normalize(newObject2World), object2World.response);
}

void ObjectCNSStereoDetector::updateWithXYand1Dimension(IsometryWithResponse& object2World, double dX, double dY, double stepInPixel, int dimension) const
{
  double x0, y0;
  camera.world2Image(object2World.translation(), x0, y0);
  Eigen::Isometry3d newObject2World =
    camera.rotationNewCamera2OldCamera(x0, y0, x0 + dX, y0 + dY).inverse() *
    object2World *
    searchStepNewObject2Object(object2World, stepInPixel, dimension);
  object2World = IsometryWithResponse(normalize(newObject2World), object2World.response);
}

bool ObjectCNSStereoDetector::refine1DimensionFromResponses(signed short response[3][16][16],
    IsometryWithResponse& object2World, double stepInPixel, int dimension) const
{
  float maxF, argMaxF[3];
  bool isSubpixel = subpixelMaximum(maxF, argMaxF, response);

  updateWithXYand1Dimension(object2World, argMaxF[0] - 8, argMaxF[1] - 8, (argMaxF[2] - 1)*stepInPixel, dimension);
  object2World.response = std::max(0.f, LinearResponseMapping().finalBin2FinalFloat(static_cast<short>(maxF)));
  // TODO: use a conversion derived from \c distribution
  // This is deferred to later because with sparse rasterization I am not sure how to normalize the response

  return isSubpixel;
}

bool ObjectCNSStereoDetector::refine1Dimension(const CNSImage& cns,
    IsometryWithResponse& object2World, double stepInPixel, int dimension) const
{
  Eigen::Isometry3d newObject2Object = searchStepNewObject2Object(object2World, stepInPixel, dimension);

  // [1+lambda][8+x][8+y] referes to \c object2World shift by x/y in the image and with c newObject2Object^lambda in the pose
  alignas(16) signed short response[3][16][16];
  responseX16Y16(response[0], cns, object2World * newObject2Object.inverse());
  responseX16Y16(response[1], cns, object2World);
  responseX16Y16(response[2], cns, object2World * newObject2Object);

  return refine1DimensionFromResponses(response, object2World, stepInPixel, dimension);
}

bool ObjectCNSStereoDetector::subpixelRefine(float& max, float argMax[3], const signed short* response) const
{
  enum {XOFS = 1, YOFS = 16, LOFS = 16 * 16};

  // argMax is x,y,r; copy data from \c response into data, with response[0]-->data[1][1][1]
  signed short data[4][3][3]; // bigger than needed, because of read access over the end
  data[0][0][0] = response[-XOFS - YOFS - LOFS];
  data[0][0][1] = response[-XOFS - YOFS];
  data[0][0][2] = response[-XOFS - YOFS + LOFS];
  data[0][1][0] = response[-XOFS     - LOFS];
  data[0][1][1] = response[-XOFS     ];
  data[0][1][2] = response[-XOFS     + LOFS];
  data[0][2][0] = response[-XOFS + YOFS - LOFS];
  data[0][2][1] = response[-XOFS + YOFS];
  data[0][2][2] = response[-XOFS + YOFS + LOFS];

  data[1][0][0] = response[     -YOFS - LOFS];
  data[1][0][1] = response[     -YOFS];
  data[1][0][2] = response[     -YOFS + LOFS];
  data[1][1][0] = response[          -LOFS];
  data[1][1][1] = response[         0];
  data[1][1][2] = response[          +LOFS];
  data[1][2][0] = response[      YOFS - LOFS];
  data[1][2][1] = response[     +YOFS];
  data[1][2][2] = response[     +YOFS + LOFS];

  data[2][0][0] = response[+XOFS - YOFS - LOFS];
  data[2][0][1] = response[+XOFS - YOFS];
  data[2][0][2] = response[+XOFS - YOFS + LOFS];
  data[2][1][0] = response[+XOFS     - LOFS];
  data[2][1][1] = response[+XOFS     ];
  data[2][1][2] = response[+XOFS     + LOFS];
  data[2][2][0] = response[+XOFS + YOFS - LOFS];
  data[2][2][1] = response[+XOFS + YOFS];
  data[2][2][2] = response[+XOFS + YOFS + LOFS];

  subpixelMaximizer.max(max, argMax, data);
  return isfinite(max);
}

void ObjectCNSStereoDetector::refine(const CNSImage& cns, IsometryWithResponse& object2World, int iterationCtr) const
{
  for(int i = 0; i < iterationCtr; i++)
  {
    refine1Dimension(cns, object2World, spec.stepInPixelDuringRefinement, DIM_TRANS_VIEWING);
    if(!object.isRotationalSymmetric)
    {
      refine1Dimension(cns, object2World, spec.stepInPixelDuringRefinement, DIM_ROT_X);
      refine1Dimension(cns, object2World, spec.stepInPixelDuringRefinement, DIM_ROT_Y);
      if(!object.isRotationalSymmetricZ)
        refine1Dimension(cns, object2World, spec.stepInPixelDuringRefinement, DIM_ROT_Z);
    }
  }
}

void ObjectCNSStereoDetector::rasteredConeOfOrientations(std::vector<Eigen::Isometry3d>& a2bList, const Eigen::Vector3d& az2b0, double delta, double eps)
{
  Eigen::Isometry3d a2b0 = fromTo(Eigen::Vector3d::UnitZ(), az2b0);
  a2bList.clear();

  int n = static_cast<int>(ceil(delta / eps));
  for(int i = -n; i <= n; i++)
    for(int j = -n; j <= n; j++)
      if(i * i + j * j < (n + 1) * (n + 1))
      {
        Eigen::Vector3d angleAxis(i * eps, j * eps, 0);
        double angle = angleAxis.norm();
        a2bList.push_back(a2b0 * Eigen::Isometry3d(Eigen::AngleAxisd(angle, angleAxis / angle)));
      }
}

void ObjectCNSStereoDetector::addToList(IsometryWithResponses& list, const IsometryWithResponse& newEntry, int maxLength)
{
  int idx = static_cast<int>(list.size() - 1);
  while(idx >= 0 && list[idx].response < newEntry.response) idx--;
  list.insert(list.begin() + (idx + 1), newEntry);
  if(static_cast<int>(list.size()) > maxLength)
    list.pop_back();
#ifndef NDEBUG
  for(int i = 0; i < static_cast<int>(list.size()) - 1; i++)
    assert(list[i].response >= list[i + 1].response);
  assert(static_cast<int>(list.size()) <= maxLength);
#endif
}
