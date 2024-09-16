#pragma once

#include "ImageProcessing/CNS/CameraModelOpenCV.h"
#include "ImageProcessing/CNS/CNSResponse.h"
#include "ImageProcessing/CNS/CodedContour.h"
#include "ImageProcessing/CNS/IsometryWithResponse.h"
#include "ImageProcessing/CNS/LutRasterizer.h"
#include "ImageProcessing/CNS/ResponseMapping.h"
#include "ImageProcessing/CNS/SearchSpecification.h"
#include "ImageProcessing/CNS/SubpixelMaximizer.h"
#include "ImageProcessing/CNS/TriangleMesh.h"
#include "ImageProcessing/Image.h"
#include <functional>

//! Algorithms and precomputed tables to find objects in a calibrated stereo image pair
/*! The detector detects objects given by a \c TriangleMesh in a
    calibrated stereo image pair. Therefor, it precomputes possible
    contours of the object in different poses and searches in the
    stereo image for a contours that has a high response.  It searches
    simultaneously in the left and right image, i.e. the response of a
    contour corresponding to a particular pose in the left image is
    added to the corresponding response of the object with the same
    pose in the right image.  This greatly increases robustness.

    The detector uses the contrast-normalized-Sobel (CNS, \c CNSResponse)
    method to evaluate in an linear illumination invariant way how
    well a given hypothetical contour is actually present in the image.
 */
class ObjectCNSStereoDetector
{
public:
  //! Type definition for a memory-aligned sequence of object poses and their responses
  using IsometryWithResponses = std::vector<IsometryWithResponse, Eigen::aligned_allocator<IsometryWithResponse>>;

  //! The object's geometry in object coordinates
  TriangleMesh object;

  //! Auxiliary data-structure to efficiently rasterize \c object
  LutRasterizer lr;

  //! Calibration of left/right camera
  /*! The camera2World frame in this calibration
      both defines the relative camera pose but also
      the reference (world) frame in which the object pose
      is given.
   */
  CameraModelOpenCV camera;

  //! The raw cns-responses are normalized for contour-length using this distribution
  /*! \c distribution contains the distribution of the raw-responses
      of random contours of a given length. It is used to convert a
      raw-response into the negative-log-likelihood (NLL) that this response is pure
      coincidence. As higher this NLL is, as less likely it is that the response issue
      pure coincidence and as more likely it is the real object.

      NOTE: It is unclear how this applies to sparse contours.
   */
  ParametricLaplacian distribution;

  //! whether the contour is rasterized with one point per edge (\c true) or a dense line (\c false)
  /*! For the efficient rasterizer this needs to be \c true. */
  bool useSparseContour;

  //! Defines which range to search for \c object in \c search regarding position and orientation
  SearchSpecification spec;

  //! Auxiliary function/data structure to interpolate a tabulated 3-D maximum
  SubpixelMaximizer subpixelMaximizer;

  //! Empty constructor
  ObjectCNSStereoDetector() : useSparseContour(true) {}

  //! Default constructor (see \c create)
  ObjectCNSStereoDetector(const TriangleMesh& object, const CameraModelOpenCV& camera, const Eigen::AlignedBox3d& viewpointRange, double spacing, const SearchSpecification& spec = SearchSpecification(), ParametricLaplacian distribution = ParametricLaplacian(), const char* filenameForLutRasterizer = nullptr)
  {create(object, camera, viewpointRange, spacing, spec, distribution, filenameForLutRasterizer);}

  //! Initializes a stereo-object detector
  /*! \c object is the object to be detected, \c camera the left \c
      [0] and \c [1] right camera. \c distribution allows to normalize
      for different contour lengths, this is at the moment not
      supported. The default case is to use a sparse contour,
      i.e. only selected points along the object contour.

      \c viewpointRange defines the range of viewpoints, i.e. camera positions
      in object frame for which a LUT is computed. Positions outside this
      range can later not be searched for. \c spacing is the grid-size of
      these precompute information. The precomputed information is
      WHICH edge is visible from a certain position, the edge is then perspectively
      mapped using the full - not discretized - camera position.

      \c spec defines in which range of positions and orientations to
      search for the object during \c search.

      Creation takes some time because the LUT \c lr is computed, in particular
      for not rotationally symmetric objects.

      \c If \c filenameForLutRasterizer
   */
  void create(const TriangleMesh& object, const CameraModelOpenCV& camera, const Eigen::AlignedBox3d& viewpointRange, double spacing,
              const SearchSpecification& spec = SearchSpecification(), const ParametricLaplacian& distribution = ParametricLaplacian(),
              const char* filenameForLutRasterizer = nullptr);

  //! Sets the camera calibration
  void setCamera(const CameraModelOpenCV& camera);

  //! Sets the search space specification \c spec
  void setSearchSpecification(const SearchSpecification& spec);

  //! Searches for the object \c object in the whole image
  /*! \c object2World is the list of \c n largest responses found.

      The search is restricted and discretized according to the
      definition in \c spec. In detail, it goes through the image of
      \c averageCamera in blocks of \c spec.blockX*spec.blockY. For
      each block it computes the ray corresponding to its center and
      intersects the ray with \c spec.positionSpace. The resulting
      line segment of object positions is discretized using \c
      searchStepNewObject2Object
      (...,spec.stepInPixelGlobalDiscretization,DIM_TRANS_VIEWING)
      such that the object size decreases by \c
      spec.stepInPixelGlobalDiscretization with each step. For each of
      the positions, all orientations in \c
      spec.object2WorldOrientation are tried.

      Each pose is rasterized using \c lr and evaluated in \c cnsLeft
      plus \c cnsRight. The same happens for the rasterized contour
      shifted by \c
      [-spec.blockX/2..+spec.blockX/2-1]*[-spec.blockY/2..+spec.blockY/2-1].

      The \c spec.nResponses largest responses are refined using \c refine
      and stored in \c object2World. If \c spec.refineExisting is \c true
      all these poses in \c object2World when calling \c search are additionally
      refined and returned (giving \c >spec.nResponses responses).

      If \c yLo and \c yHi is the range of y coordinates in which the routine
      searches. It is clipped to the image height. \c (yHi-yLo+1) must be
      a multiple of \c spec.blockY.
   */
  void search(IsometryWithResponses& object2WorldList,
              const Image<CNSResponse>& cns,
              int xLo = 0, int xHi = std::numeric_limits<int>::max(),
              int yLo = 0, int yHi = std::numeric_limits<int>::max()) const;

  //! Searches through all object poses where the origin maps to [x..spec.blockX-1]*[y..spec.blockY-1]
  /*! The center is converted to a ray, intersected with \c spec.positionSpace
      and discretized according to \c spec.stepInPixelGlobalDiscretization. This
      gives the set of positions. The set of orientations is taken from \c spec.object2WorldOrientation.
      For all these poses \c searchBlockFixedPose is called and the result
      added to \c object2World.
   */
  void searchBlockAllPoses(IsometryWithResponses& object2WorldList,
                           const Image<CNSResponse>& cns,
                           int x, int y) const;

  //! Search for a single object pose \c object2WorldTry with a block of image translation
  /*! \c object is rasterized with the pose \c object2WorldTry and the resulting
      contour evaluated shifted by \c
      [-spec.blockX/2..+spec.blockX/2-1]*[-spec.blockY/2..+spec.blockY/2-1].

      The best response is returned in \c object2World (incorporating the
      image translation into the pose) if the response is larger than the
      initial value of \c object2World.response. Otherwise \c object2World
      is untouched.

      If \c object2World is overwritten, \c true is returned.
   */
  bool searchBlockFixedPose(IsometryWithResponse& object2World,
                            const Image<CNSResponse>& cns,
                            const Eigen::Isometry3d& object2WorldTry) const;

  //! Renders the object in all poses searched for into \c ct for visualization of the search space
  /*! Actually the same contour is searched for in different translations according to
    \c spec.blockX and \c spec.blockY. Only the center of these is visualized, otherwise the
    result would be so cluttered nothing can be seen.

    The poses are rendered from \c renderCamera, the selection of poses however depends of
    \c averageCamera as in \c search.

    If \c onlyEvery>1, only every \c onlyEvery contour is rendered in each of the 3 translation
    DOF.
   */
  void renderSearchSpace(Contour& ct, const CameraModelOpenCV& renderCamera, int onlyEvery = 1) const;

  //! Rendering pendant to \c searchBlockAllPoses
  /*! Adds to \c ct. */
  void renderBlockAllPoses(Contour& ct, int x, int y,
                           const CameraModelOpenCV& renderCamera, int onlyEvery = 1) const;

  //! Rendering pendant to \c searchBlockFixedPose
  void renderBlockFixedPose(Contour& ct,
                            const Eigen::Isometry3d& object2WorldTry,
                            const CameraModelOpenCV& renderCamera) const;

  //! Adds \c newEntry to the sorted \c list at the right position.
  /*! \c list must be sorted by descending \c .response. If \c list becomes larger than \c maxLength, the last
    entry is dropped.

    Finally, addToList should not insert \c newEntry if it is too close to an existing entry, however
    this is not implemented yet.
   */
  static void addToList(IsometryWithResponses& list, const IsometryWithResponse& newEntry, int maxLength);

  //! Returns how much in the stereo-image \c imgLeft, imgRight there appears to be \c object at \c object2World
  /*! rasterizes \c object using \c object2World and \c camera[0/1], evaluates the contour response at these
      two contours and adds the result.
   */
  double response(const Image<CNSResponse>& cns, const Eigen::Isometry3d& object2World) const;
  double response(const Image<CNSResponse>& cns, const CodedContour& ct) const;

  //! Computes the response for \c object2World and stores it in \c object2World.response
  void computeResponse(const Image<CNSResponse>& cns, IsometryWithResponse& object2World) const
  {object2World.response = response(cns, object2World);}

  //! Computes a 16*16 block of responses for a given pose
  /*! \c object is rendered at \c object2World in the left and right camera and the resulting contour is
      evaluated in \c cnsLeft and \c cnsRight. This binary response is stored in response[8][8] and can be
      converted to float using \c mapping.finalBin2finalFloat(...).

      The same is done in parallel with contours shifted by -8..7 in x/y. All these responses are stored in
      \c response.
   */
  void responseX16Y16(signed short response[16][16],
                      const Image<CNSResponse>& cns, const Eigen::Isometry3d& object2World) const;

  //! Computes maximal response for \c object rendered at \c object2World and shifted by \c +/-blockX/2 * +/-blockY/2.
  /*! The contour is rasterized in the left and right camera and evaluated shifted
      by x in [-blockX/2..+blockX/2-1] and y in [-blockY/2...blockY/2-1]. The largest response is
      returned in \c maxVal and in \c argMaxX and \c argMaxY the corresponding shift vector.

      If the maximum is smaller than the input \c maxVal, \c maxVal, \c argMaxX and \c argMaxY do not change.

      \c blockX and \c blockY must be multiples of 16.
   */
  void responseXYMax(int& maxVal, int& argMaxX, int& argMaxY,
                     const Image<CNSResponse>& cns, const Eigen::Isometry3d& object2World,
                     int blockX, int blockY) const;

  //! Dimensions in the 6-DOF pose space used for searching
  enum {DIM_ROT_X = 0, DIM_ROT_Y = 1, DIM_ROT_Z = 2, DIM_TRANS_IMAGE_X = 3, DIM_TRANS_IMAGE_Y = 4, DIM_TRANS_VIEWING = 5};

  //! \c special function for the translation DOF of \c searchStepNewObject2Object
  /*! Receives and returns a position in world coordinates (!) instead of a pose.
      Positive \c stepInPixel moves from the camera away.
       */
  Eigen::Vector3d searchStepTranslationViewing(const Eigen::Vector3d& object2WorldTrans, double stepInPixel) const;

  //! Defines a search step in the 6DOF object pose
  /*! The search step is so large that it roughly results
    in a movement of \c stepInPixel on the object contour
    in the image.

    \c dimension defines the resulting type of Isometry,
    with object-x-y-z-rotation (\c DIM_ROT_X, \c DIM_ROT_Y ,\c DIM_ROT_Z) and translation
    in image x,y, (\c DIM_TRANS_IMAGE_X, \c DIM_TRANS_IMAGE_Y) and viewing direction (\C DIM_TRANS_VIEWING).

    Note that \c DIM_TRANS_IMAGE* are usually not used because they are
    hardcoded by the fact that \c responseX16Y16 computes an array of x/y
    translated responses.
   */
  Eigen::Isometry3d searchStepNewObject2Object(const Eigen::Isometry3d& object2World, double stepInPixel, int dimension) const;

  //! Changes \c object2World by an image translation by \c dX, dY
  void updateWithXY(IsometryWithResponse& object2World, double dX, double dY) const;

  //! Changes \c object2World by an image translation by \c dX, dY and a changing in \c dimension by \c stepInPixel
  /*! See \c searchStepNewObject2Object for a definition of \c stepInPixel and \c dimension
   */
  void updateWithXYand1Dimension(IsometryWithResponse& object2World, double dX, double dY, double stepInPixel, int dimension) const;

  //! Modifies \c object2World according to \c response being responses from this pose and surrounding poses
  /*! response[1+lambda][8+dX][8+dY] must be the response of the object contour
      at \c object2World*searchStepNewObject2Object (object2World, lambda*stepInPixel, dimension) shifted by dX, dY.

      This means \c response includes responses surrounding \c object2World in the dimensions of
      x/y image translation as well as the dimension specified by \c dimension. The function finds
      the interpolated maximum of \c response and approximately updates \c object2World accordingly.

      The return value specifies, whether this was an interpolated refinement (\c true) or whether
      the discrete maximum was at the border (\c false) and no interpolation could be done.

      Note that the \c object2World.response value returned is not an actually obtained response but
      a subpixel interpolated maximum.
   */
  bool refine1DimensionFromResponses(signed short response[3][16][16],
                                     IsometryWithResponse& object2World, double stepInPixel, int dimension) const;

  //! refines \c object2World in one 3-D direction (\c dimension) as well as 2D translation
  /*! Evaluates the response of this object in the pose \c object2World, \c object2World*newObject2Object
    and \c object2World*newObject2Object.inverse() each in a range of [-8..7]*[-8..7] regarding
    image translation in x/y. Of these responses it takes the subpixel-interpolated maximum
    and updates \c object2World accordingly.  Here \c newObject2Object is computed
    by \c searchStepNewObject2Object.

    The update in image-x/y translation is approximately converted into a camera rotation,
    the update regarding \c newObject2Object is scaled using \c searchStepNewObject2Object.

    The return value specifies, whether this was an interpolated refinement (\c true) or whether
    the discrete maximum was at the border (\c false) and no interpolation could be done.

      Note that the \c object2World.response value returned is not an actually obtained response but
      a subpixel interpolated maximum.
   */
  bool refine1Dimension(const Image<CNSResponse>& cns,
                        IsometryWithResponse& object2World, double stepInPixel, int dimension) const;

  //! Refines \c object2World
  /*! Calls \c refine1Dimension \c ctr times in each directions \c DIM_TRANS_VIEWING, DIM_ROT_X/Y/Z
      (unless rotational \c object.isRotationalSymmetricZ). This is repeated \c iterationCtr times.

      Note that the \c object2World.response value returned is not an actually obtained response but
      a subpixel interpolated maximum.
   */
  void refine(const Image<CNSResponse>& cns, IsometryWithResponse& object2World, int iterationCtr = 1) const;

  //! Finds the maximum of \c response with sub-pixel interpolation
  /*! The maximum is response[argMax[2]][argMax[1]][argMax[0]]. If the discrete maximum is
      in the interior of the array, it is subpixel-refined with fractional
      coordinates as the result. If it is at the border the integer values are returned.

      The (interpolated) maximal value is returned in \c max.
      The return value specifies, whether this was an interpolated refinement (\c true) or whether
      the discrete maximum was at the border (\c false) and no interpolation could be done.
   */
  bool subpixelMaximum(float& max, float argMax[3], const signed short response[3][16][16]) const;

  //! refines a pixel discrete maximum to sub-pixel
  /*! \c max must be a discrete maximum, and \c response a pointer
      into that entry of an unsigned char [][16][16] array that
      corresponds to this (x,y,lambda) combination.  Further, the 3*3*3
      entries around that entry must be valid in \c response.

      The routine then fits a quadratic function to the 3*3*3 window
      of responses, and determines the float maximum of that, storing
      it in \c max. If the fitted function has no maximum, \c false is
      returned and \c max set to 0. \c argMax are the floating point
      arguments of the interpolated maximum.
   */
  bool subpixelRefine(float& max, float argMax[3], const signed short* response) const;

  //! Computes a list of orientations \c a2bList where a's Z-axis points towards \c az2b0 with up to \c delta deviation
  /*! The set of possible Z-axes is rastered with vectors \c eps (angle) apart. Rotation around Z is not rasterized,
      so this is a 2-D raster.
   */
  static void rasteredConeOfOrientations(std::vector<Eigen::Isometry3d>& a2bList,
                                         const Eigen::Vector3d& az2b0, double delta, double eps);
};
