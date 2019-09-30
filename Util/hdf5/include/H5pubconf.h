#ifdef TARGET_V6
#include "H5pubconfNaoV6.h"
#elif defined TARGET_ROBOT
#include "H5pubconfNaoV5.h"
#elif defined LINUX
#include "H5pubconfLinux.h"
#elif defined WINDOWS
#include "H5pubconfWindows.h"
#elif defined MACOS
#include "H5pubconfMacOS.h"
#else
#error "Unknown platform"
#endif
