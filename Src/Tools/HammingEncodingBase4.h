/**
* @file HammingEncodingBase4.h
*
* A collection of functions to convert between bytes and base-4-hamming-codes
*
* Hamming(7, 4) encodes 8 bits into 7 quaternary digits. 
*               when decoding, one error can be detected and corrected.
*
* Hamming(8, 4) encodes 8 bits into 8 quaternary digits.
*               when decoding, two errors can be detected but only one error can be corrected.
*
* Implementation based on:
*   Bystrykh, Leonid V. "Generalized DNA barcode design based on Hamming codes." PloS one 7.5 (2012): e36852.
*
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include <cmath>
#include <array>

namespace HammingEncodingBase4
{
  void toBase4Inverse(unsigned char number, std::array<unsigned char, 4>& quaternary);
  unsigned char fromBase4Inverse(const std::array<unsigned char, 4>& quaternary);

  void encodeHamming74(const std::array<unsigned char, 4>& quaternary, std::array<unsigned char, 7>& hamming);
  bool correctHamming74(std::array<unsigned char, 7>& hamming);
  void decodeHamming74(const std::array<unsigned char, 7>& hamming, std::array<unsigned char, 4>& quaternary);

  void encodeHamming84(const std::array<unsigned char, 4>& quaternary, std::array<unsigned char, 8>& hamming);
  bool correctHamming84(std::array<unsigned char, 8>& hamming);
  void decodeHamming84(const std::array<unsigned char, 8>& hamming, std::array<unsigned char, 4>& quaternary);
}