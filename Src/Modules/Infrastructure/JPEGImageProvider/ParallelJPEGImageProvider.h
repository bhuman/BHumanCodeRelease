/**
 * @file ParallelJPEGImageProvider.h
 *
 * This file declares a module that compresses images in parallel to the
 * current thread.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/JPEGImageRequest.h"

MODULE(ParallelJPEGImageProvider,
{,
  REQUIRES(BallPercept),
  REQUIRES(CameraImage),
  PROVIDES_WITHOUT_MODIFY(JPEGImage),
  USES(JPEGImage),
  PROVIDES(JPEGImageRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(70) jpegQuality, /**< The quality of the JPEG compressing (0 = bad ... 100 = very good). */
  }),
});

class ParallelJPEGImageProvider : public ParallelJPEGImageProviderBase, Thread
{
  Semaphore requestedSignal; /**< A signal used for announcing that a new image should be compressed. */
  Semaphore completedSignal; /**< A signal used for announcing that compression has been completed. */
  unsigned jobsRequested = 0; /**< The number of jobs requested so far. */
  unsigned jobResultsUsed = 0; /**< The number of jobs completed so far. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJPEGImage The representation updated.
   */
  void update(JPEGImage& theJPEGImage) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJPEGImage The representation updated.
   */
  void update(JPEGImageRequest& theJPEGImageRequest) override;

  /** The worker thread to compress images. */
  void compressImages();

public:
  ParallelJPEGImageProvider();
  ~ParallelJPEGImageProvider();
};
