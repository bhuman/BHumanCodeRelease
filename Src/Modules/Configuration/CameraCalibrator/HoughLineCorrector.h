/**
 * @file HoughLineCorrector.h
 *
 * This file declares a module that fits lines using a Sobel image and Hough transformation.
 *
 * @author Andreas Baude
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#pragma once

#include "ImageProcessing/Image.h"
#include "ImageProcessing/Sobel.h"
#include "Framework/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/LineCorrector.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"

MODULE(HoughLineCorrector,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(LinesPercept),
  REQUIRES(OptionalECImage),
  PROVIDES(LineCorrector),
  DEFINES_PARAMETERS(
  {,
    (unsigned int)(1800) numOfAngles, /**< The number of angles that should be considered in the hough lines transformation. */
    (float)(0.25f) sobelThreshValue, /**< The minimum ratio of the maximum pixel value in the Sobel image for a pixel to be considered an edge pixel. */
    (float)(2000.f) maxLineLength, /**< If detected lines are too long, the fitting algorithm could fit them wrong and end up not parallel to the real line. */
    (int)(2) minDisImage, /**< Minimum distance that lines in hough space should have in the image. */
  }),
});

class HoughLineCorrector : public HoughLineCorrectorBase
{
public:
  HoughLineCorrector();

private:
  /** This struct represents a local maximum in the hough space. */
  struct Maximum
  {
    unsigned int maxAcc; /**< The accumulator value of the local maximum. */
    unsigned int angleIndex; /**< The angle index of the local maximum in the hough space. */
    unsigned int distanceIndex; /**< The distance index of the local maximum in the hough space. */
  };

  void update(LineCorrector& lineCorrector) override;

  /** Fills the sine/cosine lookup tables. */
  void createLookUpTables();

  /**
   * Corrects start and end point of a given kine using the hough lines transformation.
   * To avoid additional error potential, only the top/bottom edge of a line is searched for and an offset
   * is set accordingly, instead of using the mean of the top and bottom edges.
   * @param cline The line to be corrected.
   * @return Whether the line could be corrected.
   */
  bool fitLine(LineCorrector::Line& cline) const;

  /**
   * Extracts a gray-scaled image patch with given start pixel and size.
   * @param start The start pixel of the desired image patch.
   * @param size The desired size of the image patch.
   * @param grayImage The image to be filled.
   */
  void extractImagePatch(const Vector2i& start, const Vector2i& size, Sobel::Image1D& grayImage) const;

  /**
   * Determines a suitable threshold at which a pixel in the hough transformation is assumed to be an edge given the Sobel image.
   * @param sobelImage The Sobel image used in the hough transformation.
   * @return The selected threshold value.
   */
  unsigned int determineSobelThresh(const Sobel::SobelImage& sobelImage) const;

  /**
   * Performs the hough lines transformation to find lines in the given Sobel image.
   * @param sobelImage The Sobel image in which lines should be searched.
   * @param minIndex The minimum angle index to be considered.
   * @param maxIndex The maximum angle index to be considered.
   * @param dMax The maximum distance a line can possibly have in the given Sobel image.
   * @param houghSpace The hough space to be calculated.
   */
  void calcHoughSpace(const Sobel::SobelImage& sobelImage, unsigned int minIndex, unsigned int maxIndex,
                      unsigned int dMax, std::vector<std::vector<int> >& houghSpace) const;

  /**
   * Searches the hough space for local maxima.
   * @param houghSpace The hough space to be searched.
   * @param minIndex The minimum angle index from which the hough space is to be considered.
   * @param maxIndex The maximum angle index up to which the hough space is to be considered.
   * @param localMaxima The Container in which the local maxima should be stored.
   */
  void determineLocalMaxima(const std::vector<std::vector<int> >& houghSpace, unsigned int minIndex,
                            unsigned int maxIndex, std::vector<Maximum>& localMaxima) const;

  std::vector<float> cosAngles, sinAngles; /**< The sine/cosine lookup tables used in the hough lines transformation. */
};
