/**
 * @file ColorModelConversions.h
 *
 * Declaration and implementation of class ColorModelConversions
 */

#pragma once

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include "Tools/Math/BHMath.h"

/**
 * A class that defines static conversions between color models for single pixels.
 */
class ColorModelConversions
{
public:
  /** Converts an YCbCr pixel into an RGB pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param R The R channel of the target pixel.
   *  @param G The G channel of the target pixel.
   *  @param B The B channel of the target pixel.
   */
  static void fromYCbCrToRGB(unsigned char Y,
                             unsigned char Cb,
                             unsigned char Cr,
                             unsigned char& R,
                             unsigned char& G,
                             unsigned char& B)
  {
    int r = Y + ((1436 * (Cr - 128)) >> 10),
        g = Y - ((354 * (Cb - 128) + 732 * (Cr - 128)) >> 10),
        b = Y + ((1814 * (Cb - 128)) >> 10);
    if(r < 0) r = 0;
    else if(r > 255) r = 255;
    if(g < 0) g = 0;
    else if(g > 255) g = 255;
    if(b < 0) b = 0;
    else if(b > 255) b = 255;
    R = static_cast<unsigned char>(r);
    G = static_cast<unsigned char>(g);
    B = static_cast<unsigned char>(b);
  }

  /** Converts an RGB pixel into an YCbCr pixel.
   *  @param R The R channel of the source pixel.
   *  @param G The G channel of the source pixel.
   *  @param B The B channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cb channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromRGBToYCbCr(unsigned char R,
                             unsigned char G,
                             unsigned char B,
                             unsigned char& Y,
                             unsigned char& Cb,
                             unsigned char& Cr)
  {
    int y = (int)(0.2990 * R + 0.5870 * G + 0.1140 * B),
        cb = 127 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B),
        cr = 127 + (int)(0.5000 * R - 0.4187 * G - 0.0813 * B);
    if(y < 0) y = 0;
    else if(y > 255) y = 255;
    if(cb < 0) cb = 0;
    else if(cb > 255) cb = 255;
    if(cr < 0) cr = 0;
    else if(cr > 255) cr = 255;
    Y = static_cast<unsigned char>(y);
    Cb = static_cast<unsigned char>(cb);
    Cr = static_cast<unsigned char>(cr);
  }

  /** Converts an YCbCr pixel into an HSI pixel.
   *  @param Y The Y channel of the source pixel.
   *  @param Cb The Cb channel of the source pixel.
   *  @param Cr The Cr channel of the source pixel.
   *  @param H The H channel of the target pixel.
   *  @param S The S channel of the target pixel.
   *  @param I The I channel of the target pixel.
   */
  static void fromYCbCrToHSI(unsigned char Y,
                             unsigned char Cb,
                             unsigned char Cr,
                             unsigned char& H,
                             unsigned char& S,
                             unsigned char& I)
  {
    int r = Y + ((1436 * (Cr - 128)) >> 10);
    int g = Y - ((354 * (Cb - 128) + 732 * (Cr - 128)) >> 10);
    int b = Y + ((1814 * (Cb - 128)) >> 10);
    int k = 0;

    if(g < b)
    {
      std::swap(g, b);
      k = -256;
    }

    if(r < g)
    {
      std::swap(r, g);
      k = -85 - k;
    }

    int chroma = r - std::min(g, b);
    int h = std::abs(k + ((g - b) << 8) / (chroma ? 6 * chroma : 1));
    int s = (chroma << 8) / (r ? r : 1);

    // normalize values to their boundaries
    H = static_cast<unsigned char>(std::min(255, h));
    S = static_cast<unsigned char>(std::max(0, std::min(255, s)));
    I = static_cast<unsigned char>(std::max(0, std::min(255, r)));
  }

  /** Converts an HSI pixel into an YCbCr pixel.
   *  @param H The H channel of the source pixel.
   *  @param S The S channel of the source pixel.
   *  @param I The I channel of the source pixel.
   *  @param Y The Y channel of the target pixel.
   *  @param Cb The Cb channel of the target pixel.
   *  @param Cr The Cr channel of the target pixel.
   */
  static void fromHSIToYCbCr(unsigned char H,
                             unsigned char S,
                             unsigned char I,
                             unsigned char& Y,
                             unsigned char& Cb,
                             unsigned char& Cr)
  {
    float h = 1.0f - H / 255.0f,
          s = S / 255.0f,
          r,
          g,
          b;

    if(h < 1.0f / 3.0f)
    {
      g = (1 - s) / 3;
      b = (1 + s * std::cos(pi2 * h) / std::cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      r = 1 - (g + b);
    }
    else if(h < 2.0f / 3.0f)
    {
      h -= 1.0f / 3.0f;
      b = (1 - s) / 3;
      r = (1 + s * std::cos(pi2 * h) / std::cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      g = 1 - (b + r);
    }
    else
    {
      h -= 2.0f / 3.0f;
      r = (1 - s) / 3;
      g = (1 + s * std::cos(pi2 * h) / std::cos(pi2 * (1.0f / 6.0f - h))) / 3.0f;
      b = 1 - (r + g);
    }

    r *= I * 3;
    g *= I * 3;
    b *= I * 3;
    if(r > 255)
      r = 255;
    if(g > 255)
      g = 255;
    if(b > 255)
      b = 255;

    fromRGBToYCbCr(static_cast<unsigned char>(r),
                   static_cast<unsigned char>(g),
                   static_cast<unsigned char>(b),
                   Y, Cb, Cr);
  }
};
