/**
 * @author Felix Thielke
 */

#include "NetworkThumbnailProvider.h"
#include "Tools/Debugging/DebugImages.h"
#include <cstring>

MAKE_MODULE(NetworkThumbnailProvider, communication)

void NetworkThumbnailProvider::update(NetworkThumbnail& networkThumbnail)
{
  if(thumbnail.empty() && theCameraInfo.camera == CameraInfo::Camera::upper)
  {
    transformImage();
    networkThumbnail.sequence = -1;
  }

  if(theTeammateData.sendThisFrame)
  {
    networkThumbnail.sequence++;

    if(offset + sendSize >= thumbnail.size())
    {
      networkThumbnail.data.resize(thumbnail.size() - offset);
      memcpy(networkThumbnail.data.data(), &thumbnail.data()[offset], thumbnail.size() - offset);
      thumbnail.clear();
      offset = 0;
    }
    else
    {
      networkThumbnail.data.resize(sendSize);
      memcpy(networkThumbnail.data.data(), &thumbnail.data()[offset], sendSize);
      offset += sendSize;
    }
  }

  COMPLEX_IMAGE("netThumbnail")
  {
    if(thumbnail.empty())
    {
      netThumbnail.setResolution(width, height);
      memset(netThumbnail[0], 0, width * height);
    }
    else
    {
      const unsigned char w = thumbnail[0];
      const unsigned char h = thumbnail[1];
      netThumbnail.setResolution(w, h);
      int pos = 0;
      for(size_t i = 2; i < thumbnail.size(); i++)
      {
        const unsigned char value = thumbnail[i] & 0xF0;
        unsigned char count = thumbnail[i] & 0x0F;
        if(count == 0)
        {
          count = 16;
        }

        for(int c = 0; c < count; c++)
        {
          netThumbnail[pos % w][pos / w] = value;
          pos++;
        }
      }
    }
    SEND_DEBUG_IMAGE("netThumbnail", netThumbnail);
  }
}

void NetworkThumbnailProvider::transformImage()
{
  width = thumbnailWidth;
  const float step = (float)theECImage.grayscaled.width / (float)width;
  height = (unsigned char)((float)theECImage.grayscaled.height / step);

  thumbnail.clear();
  thumbnail.emplace_back(width);
  thumbnail.emplace_back(height);

  unsigned char value = 0xFF;
  unsigned char count = 0;

  int h = 0;
  for(float y = step / 2; h < height; y += step, h++)
  {
    if(y >= theECImage.grayscaled.height)
    {
      y = (float)(theECImage.grayscaled.height - 1);
    }
    const unsigned char* srcLine = theECImage.grayscaled[(int)y];
    int w = 0;
    for(float x = step / 2; w < width; x += step, w++)
    {
      if(x >= theECImage.grayscaled.width)
      {
        x = (float)(theECImage.grayscaled.width - 1);
      }
      const unsigned char v = srcLine[(int)x] & 0xF0;

      if(v == value)
      {
        count++;
        if(count == 16)
        {
          count = 0;
          thumbnail.emplace_back(value | count);
        }
      }
      else
      {
        if(count > 0)
        {
          thumbnail.emplace_back(value | count);
        }
        value = v;
        count = 1;
      }
    }
  }
  if(count > 0)
  {
    thumbnail.emplace_back(value | count);
  }
}

