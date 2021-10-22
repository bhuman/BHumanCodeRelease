#ifdef TARGET_ROBOT
#include "H5pubconfNaoV6.h"
#elif defined LINUX
#include "H5pubconfLinux.h"
#elif defined WINDOWS
#include "H5pubconfWindows.h"
#elif defined MACOS && !defined __arm64__
#include "H5pubconfMacOS.h"
#elif defined MACOS && defined __arm64__
#include "H5pubconfMacOSarm64.h"
#else
#error "Unknown platform"
#endif
