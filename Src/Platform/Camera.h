/**
 * @file Platform/Camera.h
 *
 * Inclusion of a platform dependent camera interface.
 *
 * @author Colin Graf
 */

#pragma once

#ifdef TARGET_ROBOT
#include "Nao/NaoCamera.h"
#define CAMERA_INCLUDED
#endif
