#pragma once

//! Coefficients for a linear mapping that normalizes raw responses with respect to, e.g., contour length
/*! The raw response (rawFloat, eqn. ??) computed for a given contour on a cns-image is mathematically in the
    range [0..1]. However, for small contours it is more likely that
    they have a large response just by coincidence, then for large contours. To normalize for this
    effect raw responses can be replaced by the likelihood that such a response is coincidence taken
    from a distribution of responses. This is then the final response (finalFloat) which can be
    compared between contours of different length.
    If Laplacian is used for the distribution this approach leads
    to a linear mapping of the raw responses [0..1] to negative false-alarm log-likelihoods as final
    responses. However, the linear mapping can also be used for other purposes.

    Technically, the situation is more complicated as internally the raw responses are represented
    as uint16 and the (rawBin) and the final as int16 (finalBin) and the scaling factors for this
    representation must allow to handle all occurring values of all contours without overall.

    The class also contains method to convert between (rawFloat, rawBin, finalFloat, finalBin).

    The \c LinearResponseMapping class handles this issue.

 */
class LinearResponseMapping
{
public:
  //! Identity mapping seen as rawFloat-->finalFloat
  LinearResponseMapping()
    : rawBin2FinalBinScale(0x7fff), rawBin2FinalBinOffset(0),
      finalBin2FinalFloatScale(1.0f / 0x7fff), finalBin2FinalFloatOffset(0),
      clippedDenom(0.25f) {}

  //! range in final bin that is actually used
  /*!
    We cannot use the full range [-0x7fff...0x7fff], because otherwise rawFloat2FinalFloatScale can overflow in \c LinearResponseMapping
   */
  enum { FINAL_BIN_LOW = -0x3fff, FINAL_BIN_HIGH = 0x3fff };

  //! Linear rawFloat-->finalFloat mapping defined by scale and offset
  /*! The binary representation scale is chosen such that finalFloat values between min and max can
      be represented.
   */
  LinearResponseMapping(double rawFloat2FinalFloatScale, double rawFloat2FinalFloatOffset, double finalFloatMin, double finalFloatMax, double clippedDenom = 0.25)
    : clippedDenom(static_cast<float>(clippedDenom))
  {
    assert(rawFloat2FinalFloatScale >= 0); // better scores must give better scores
    // finalBin=-0x3fff --> finalFloat=finalFloatMin, finalBin=+0x3fff -> finalFloat=finalFloatMax
    finalBin2FinalFloatScale  = static_cast<float>((finalFloatMax - finalFloatMin) / (FINAL_BIN_HIGH - FINAL_BIN_LOW));
    finalBin2FinalFloatOffset = static_cast<float>(finalFloatMax - 0x3fff * finalBin2FinalFloatScale);

    // rawBin=0 --> finalFloat=rawFloat2FinalFloatOffset, rawBin=0xffff -> finalFloat=rawFloat2FinalFloatOffset+rawFloat2FinalFloatScale
    int hv  = int(rawFloat2FinalFloatScale / 0xffff / finalBin2FinalFloatScale * 0x10000);
    assert(-0x7fff <= hv && hv <= 0x7fff);
    rawBin2FinalBinScale = static_cast<short>(hv);
    hv = int((rawFloat2FinalFloatOffset - finalBin2FinalFloatOffset) / finalBin2FinalFloatScale);
    assert(-0x7fff <= hv && hv <= 0x7fff);
    rawBin2FinalBinOffset = static_cast<short>(hv);
  }

  //finalBin = rawBin*rawBin2FinalBinScale/0x10000
  //dfinalFloat=drawBin*rawBin2FinalBinScale/0x10000*finalBin2FinalFloatScale
  //rawFloat2FinalFloatScale = 0xffff*rawBin2FinalBinScale/0x10000*finalBin2FinalFloatScale
  //rawFloat2FinalFloatScale/0xffff*0x10000/finalBin2FinalFloatScale = rawBin2FinalBinScale

  //! Factors used in binary conversion
  signed short rawBin2FinalBinScale, rawBin2FinalBinOffset;

  //! Factors used in float conversion
  float finalBin2FinalFloatScale, finalBin2FinalFloatOffset;

  //! A clipped points counts as much as negative evidence as \c clippedNom (<<1) points with 0 response
  float clippedDenom;

  unsigned short rawFloat2RawBin(float rawFloat) const {return static_cast<unsigned short>(rawFloat * 0xffff);}

  float rawBin2RawFloat(unsigned short rawBin) const {return rawBin / static_cast<float>(0xffff);}

  signed short rawBin2FinalBin(unsigned short rawBin) const {return ((static_cast<int>(rawBin) * static_cast<int>(rawBin2FinalBinScale)) >> 16) + rawBin2FinalBinOffset;}

  float finalBin2FinalFloat(signed short finalBin) const {return finalBin2FinalFloatScale * finalBin + finalBin2FinalFloatOffset;}

  signed short finalFloat2FinalBin(float finalFloat) const {return short((finalFloat - finalBin2FinalFloatOffset) / finalBin2FinalFloatScale);}

  float rawBin2FinalFloat(unsigned short rawBin) const {return  finalBin2FinalFloat(rawBin2FinalBin(rawBin));}

  float rawFloat2FinalFloat(float rawFloat) const {return finalBin2FinalFloat(rawBin2FinalBin(rawFloat2RawBin(rawFloat)));}
};

//! A Laplacian distribution where the scale \c b is proportional to a power of a parameter \c l
/*! The distribution is given by

      \f[{equation}
            p(resp|\mu,b_0,\gamma,l) = \frac{1}{2b_0l^\gamma} exp(-\frac{|resp--mu|}{b_0l^\gamma}
      \f]

    \f$ \mu \f$, \f$ b_0 \f$ and \f$ \gamma  \f$ are parameters stored in an \c ParametricLaplacian
    object. \f$ l \f$ usually is the arc-length of a curve. \f$ resp \f$ is the raw response [0..1].

    A \c ParametricLaplacian object describes how likely curves of a certain arc-length have a particular
    response. Either at a random place in an image, defining how likely a response happens by coincidence
    or at a place where the sought for object is actually present, defining how likely a response is in
    the positive case.

    Both can be used to derive a \c LinearResponseMapping for a certain contour that converts the raw response
    to log-likelihoods and can be used in probabilistic reasoning.

    http://en.wikipedia.org/wiki/Laplace_distribution
 */
class ParametricLaplacian
{
public:
  //! Parameters of the distribution (see \c ParametricLaplacian)
  double mu, b0, gamma;

  //! A clipped points counts as much as negative evidence as \c clippedNom (<<1) points with 0 response
  double clippedDenom;

  //! Dummy parameters for flat mapping
  ParametricLaplacian() : mu(0), b0(0.5), gamma(0), clippedDenom(0.25) {}

  //! std. constructors
  ParametricLaplacian(double mu, double b0, double gamma, double clippedDenom = 0.25)
    : mu(mu), b0(b0), gamma(gamma), clippedDenom(clippedDenom)
  {
    assert(0 <= mu && mu <= 1 && b0 > 0 && gamma <= 0);
  }

  //! Computes the scale \c b for a given arc-length \c l
  double b(double l) const {return b0 * pow(l, gamma);}

  //! Computes the probability \c response at arc-length \c l
  double prob(double response, double l) const {double bL = b(l); return 1 / (2 * bL) * exp(-fabs(response - mu) / bL);}

  //! Computes -log of the probability \c response at arc-length \c l
  double nll(double response, double l) const {double bL = b(l); return log(2 * bL) + fabs(response - mu) / bL;}

  //! Computes nll replaced by a linear function of the left/right half
  double nllLinear(double response, double l, bool rightPart) const
  {
    double bL = b(l);
    double delta;
    if(rightPart)
      delta = response - mu;
    else
      delta = mu - response;
    return log(2 * bL) + delta / bL;
  }

  //! Computes the range of \c nll(response, l) for \c response in [0..1]
  void nllRange(double& low, double& high, double l) const
  {
    double bL = b(l);
    low = log(2 * bL);
    double delta = std::max(1 - mu, mu);
    high = low + delta / bL;
  }

  //! Computes the range of \c nll(response, l) for \c response in [0..1], \c l in [lMin..lMax]
  void nllRange(double& low, double& high, double lMin, double lMax) const
  {
    assert(lMin <= lMax);
    double bLow = b(lMax), bHigh = b(lMin);
    low = log(2 * bLow);
    double deltaMax = std::max(1 - mu, mu);
    high = maxOfFunction(deltaMax, bLow, bHigh);
  }

  //! Computes \c nllRange if the nll function was replaced by the linear function left or right part of \c mu
  /*! Actually the range may be somewhat larger due to conservative computation but is ensured
      to contain the real range.
   */
  void extendedNllRange(double& low, double& high, double lMin, double lMax, bool rightPart) const
  {
    assert(lMin <= lMax);
    double bLow = b(lMax), bHigh = b(lMin);
    double deltaLow, deltaHigh;
    if(rightPart)
    {
      deltaLow   = -mu;
      deltaHigh  = 1 - mu;
    }
    else
    {
      deltaLow = mu - 1;
      deltaHigh = mu;
    }
    low  = log(2 * bLow) + deltaLow / bLow;
    high = log(2 * bHigh) + deltaHigh / bLow;
  }

  //! Expresses \c nll(response, l) as \c scale*response+offset
  /*! If \c rightPart then the function for \c response>=mu is returned otherwise for \c response<=mu
   */
  void nllLinearPart(double& scale, double& offset, double l, bool rightPart = true) const
  {
    double bL = b(l);
    if(rightPart)
    {
      offset = log(2 * bL) - mu / bL;
      scale  = 1 / bL;
    }
    else
    {
      offset = log(2 * bL) + mu / bL;
      scale = -1 / bL;
    }
  }

  //! Returns the maximum of log(2*b)+delta/b for b in [bLow..bHigh]
  static double maxOfFunction(double delta, double bLow, double bHigh)
  {
    // The function has no local maxima, so the maximum is either bLow or bHigh
    return std::max(log(2 * bLow) + delta / bLow, log(2 * bHigh) + delta / bHigh);
  }

  //! Returns whether this is \c ParametricLaplacian
  /*! A flat mapping create a scale of 1 and offset of 0 in \c nllLinearPart */
  bool isFlat() const {return mu == 0 && b0 == 0.5 && gamma == 0;}
};

//! Computes the linear mapping that maps a response to its negative-log-likelihood in \c dist (given l).
/*! This function is actually only piece-wise linear, \c rightPart=true gives the piece resp>mu,
    \c rightPart=false the piece resp<mu.

    The scaling is chosen
 */
inline LinearResponseMapping nllMapping(const ParametricLaplacian& dist, double l, double lMin, double lMax, bool rightPart = true)
{
  double scale, offset, finalFloatMin, finalFloatMax;
  dist.extendedNllRange(finalFloatMin, finalFloatMax, lMin, lMax, rightPart);
  dist.nllLinearPart(scale, offset, l, rightPart);
  return LinearResponseMapping(scale, offset, finalFloatMin, finalFloatMax, dist.clippedDenom);
}
