/**
 * File:   LogFileFormat.h
 * Author: arne
 *
 * Created on September 25, 2013, 2:54 PM
 */

#pragma once

#include "Tools/Enum.h"

ENUM(LogFileFormat,
{,
  logFileUncompressed,
  logFileCompressed,
  logFileMessageIDs,
  logFileStreamSpecification,
});
