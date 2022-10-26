/**
 * The file declares a contrast normalized Sobel (CNS) image, i.e. an image of
 * CNS responses.
 *
 * @author Udo Frese
 * @author Thomas Röfer
 */

#pragma once

#include "ImageProcessing/CNS/CNSResponse.h"
#include "ImageProcessing/Image.h"
#include "Debugging/DebugImages.h"

STREAMABLE_WITH_BASE(CNSImage, Image<CNSResponse>,
{
  CNSImage() : Image<CNSResponse>(640, 480, 640 * 64 * sizeof(CNSResponse))
  {
    std::memset(reinterpret_cast<char*>((*this)[-64]), CNSResponse::OFFSET, width * (128 + height) * sizeof(CNSResponse));
  }

  void draw() const
  {
    SEND_DEBUG_IMAGE("CNSImage", *this, PixelTypes::Edge2);
  },
});
