/**
* @file HammingEncodingBase4.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "HammingEncodingBase4.h"
#include <algorithm>

namespace HammingEncodingBase4
{
  void toBase4Inverse(unsigned char number, std::array<unsigned char, 4>& quaternary)
  {
    for(int i = 0; i < 4; ++i)
    {
      quaternary[i] = number % 4;
      number /= 4;
    }
  }

  unsigned char fromBase4Inverse(const std::array<unsigned char, 4>& quaternary)
  {
    unsigned char result = 0;
    for(size_t i = 0; i < quaternary.size(); ++i)
      result += quaternary[i] * static_cast<unsigned char>(std::pow(4, i));
    return result;
  }

  void encodeHamming74(const std::array<unsigned char, 4>& quaternary, std::array<unsigned char, 7>& hamming)
  {
    const unsigned char d = static_cast<unsigned char>(quaternary.size());

    const unsigned char p1 = (d - ((quaternary[0] + quaternary[1] + quaternary[3]) % d)) % d;
    const unsigned char p2 = (d - ((quaternary[0] + quaternary[2] + quaternary[3]) % d)) % d;
    const unsigned char p3 = (d - ((quaternary[1] + quaternary[2] + quaternary[3]) % d)) % d;

    hamming[0] = p1;
    hamming[1] = p2;
    hamming[2] = quaternary[0];
    hamming[3] = p3;
    hamming[4] = quaternary[1];
    hamming[5] = quaternary[2];
    hamming[6] = quaternary[3];
  }

  bool correctHamming74(std::array<unsigned char, 7>& hamming)
  {
    const unsigned char d = static_cast<unsigned char>(hamming.size()) - 3;

    const unsigned char p1 = (hamming[0] + hamming[2] + hamming[4] + hamming[6]) % d;
    const unsigned char p2 = (hamming[1] + hamming[2] + hamming[5] + hamming[6]) % d;
    const unsigned char p3 = (hamming[3] + hamming[4] + hamming[5] + hamming[6]) % d;

    const char errorType = std::max(std::max(p1, p2), p3);

    if(!errorType)
      return true;

    const unsigned char errorPos = (p3 ? 1 << 2 : 0) + (p2 ? 1 << 1 : 0) + (p1 ? 1 : 0) - 1;
    const unsigned char corrected = (hamming[errorPos] - errorType + 4) % 4;

    hamming[errorPos] = corrected;

    return (hamming[0] + hamming[2] + hamming[4] + hamming[6]) % d
        || (hamming[1] + hamming[2] + hamming[5] + hamming[6]) % d
        || (hamming[3] + hamming[4] + hamming[5] + hamming[6]) % d;
  }

  void decodeHamming74(const std::array<unsigned char, 7>& hamming, std::array<unsigned char, 4>& quaternary)
  {
    quaternary[0] = hamming[2];
    quaternary[1] = hamming[4];
    quaternary[2] = hamming[5];
    quaternary[3] = hamming[6];
  }


  void encodeHamming84(const std::array<unsigned char, 4>& quaternary, std::array<unsigned char, 8>& hamming)
  {
    const unsigned char d = 4;

    const unsigned char p1 = (d - ((quaternary[0] + quaternary[1] + quaternary[3]) % d)) % d;
    const unsigned char p2 = (d - ((quaternary[0] + quaternary[2] + quaternary[3]) % d)) % d;
    const unsigned char p3 = (d - ((quaternary[1] + quaternary[2] + quaternary[3]) % d)) % d;

    hamming[0] = p1;
    hamming[1] = p2;
    hamming[2] = quaternary[0];
    hamming[3] = p3;
    hamming[4] = quaternary[1];
    hamming[5] = quaternary[2];
    hamming[6] = quaternary[3];
    hamming[7] = (d - ((hamming[0] + hamming[1] + hamming[2] + hamming[3] + hamming[4] + hamming[5] + hamming[6]) % d)) % d;
  }

  bool correctHamming84(std::array<unsigned char, 8>& hamming)
  {
    const unsigned char d = 4;

    const unsigned char p1 = (hamming[0] + hamming[2] + hamming[4] + hamming[6]) % d;
    const unsigned char p2 = (hamming[1] + hamming[2] + hamming[5] + hamming[6]) % d;
    const unsigned char p3 = (hamming[3] + hamming[4] + hamming[5] + hamming[6]) % d;

    const char errorType = std::max(std::max(p1, p2), p3);

    if(!errorType)
      return true;

    const unsigned char errorPos = (p3 ? 1 << 2 : 0) + (p2 ? 1 << 1 : 0) + (p1 ? 1 : 0) - 1;
    const unsigned char corrected = (hamming[errorPos] - errorType + 4) % 4;

    hamming[errorPos] = corrected;

    return !((hamming[0] + hamming[2] + hamming[4] + hamming[6]) % d
          || (hamming[1] + hamming[2] + hamming[5] + hamming[6]) % d
          || (hamming[3] + hamming[4] + hamming[5] + hamming[6]) % d
          || (hamming[0] + hamming[1] + hamming[2] + hamming[3] + hamming[4] + hamming[5] + hamming[6] + hamming[7]) % d);
  }

  void decodeHamming84(const std::array<unsigned char, 8>& hamming, std::array<unsigned char, 4>& quaternary)
  {
    quaternary[0] = hamming[2];
    quaternary[1] = hamming[4];
    quaternary[2] = hamming[5];
    quaternary[3] = hamming[6];
  }
}