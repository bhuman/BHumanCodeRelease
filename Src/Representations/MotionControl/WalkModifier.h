/**
 * @file WalkModifier.h
 * Declaration of a struct for representing how the gyro balance factors for the ankles of the WalkingEngine shall be learned
 * @author Philip Reichenberg
 */

#pragma once
STREAMABLE(WalkModifier,
{,
  (int)(0) numOfGyroPeaks,
  (float)(0) balanceChangeFactor,
});
