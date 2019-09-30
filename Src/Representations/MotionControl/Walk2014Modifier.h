/**
 * @file Walk2014Modifier.h
 * Declaration of a struct for representing how the gyroBalanceFactors of the Walk2014Generator shall be learned
 * @author Philip Reichenberg
 */

#pragma once
STREAMABLE(Walk2014Modifier,
{,
  (int)(0) numOfGyroPeaks,
  (float)(0) balanceChangeFactor,
});
