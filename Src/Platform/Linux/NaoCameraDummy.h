/**
* @file Platform/Linux/NaoCameraDummy.h
* This camera does nothing
* @author Andreas Stolpmann
*/

#pragma once

#include "NaoCamera.h"
#include "BHAssert.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <poll.h>
#include <linux/videodev2.h>
#ifdef USE_USERPTR
#include <malloc.h> // memalign
#endif

#include "SystemCall.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"

class NaoCameraDummy : public NaoCamera
{
public:
  NaoCameraDummy()
    : NaoCamera("/dev/video0", CameraInfo::upper, 320, 240, true) {}

  bool hasImage()
  {
    if(NaoCamera::hasImage())
      NaoCamera::releaseImage();

    return false;
  }
};