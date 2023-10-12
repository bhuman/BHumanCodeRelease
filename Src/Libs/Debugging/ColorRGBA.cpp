#include "ColorRGBA.h"
#include "Network/RoboCupGameControlData.h"
#include "Platform/BHAssert.h"
#include "Streaming/FunctionList.h"
#include "Streaming/Streamable.h"

const ColorRGBA ColorRGBA::white(255, 255, 255);
const ColorRGBA ColorRGBA::gray(128, 128, 128);
const ColorRGBA ColorRGBA::black(0, 0, 0);
const ColorRGBA ColorRGBA::red(255, 0, 0);
const ColorRGBA ColorRGBA::green(0, 255, 0);
const ColorRGBA ColorRGBA::blue(0, 0, 255);
const ColorRGBA ColorRGBA::yellow(255, 255, 0);
const ColorRGBA ColorRGBA::cyan(0, 255, 255);
const ColorRGBA ColorRGBA::magenta(255, 0, 255);
const ColorRGBA ColorRGBA::orange(255, 128, 0);
const ColorRGBA ColorRGBA::violet(183, 10, 210);
const ColorRGBA ColorRGBA::brown(139, 69, 19);

ColorRGBA ColorRGBA::fromTeamColor(int teamColor)
{
  static_assert(TEAM_BLUE == 0 && TEAM_RED == 1 && TEAM_YELLOW == 2 && TEAM_BLACK == 3
                && TEAM_WHITE == 4 && TEAM_GREEN == 5 && TEAM_ORANGE == 6 && TEAM_PURPLE == 7
                && TEAM_BROWN == 8 && TEAM_GRAY == 9, "The following depends on the order of these macros.");

  ASSERT(teamColor >= 0 && teamColor <= 9);

  static const ColorRGBA colors[] =
  {
    ColorRGBA::blue,
    ColorRGBA::red,
    ColorRGBA::yellow,
    ColorRGBA::black,
    ColorRGBA::white,
    ColorRGBA::green,
    ColorRGBA::orange,
    ColorRGBA::violet,
    ColorRGBA::brown,
    ColorRGBA::gray
  };

  return colors[teamColor];
}

ColorRGBA ColorRGBA::operator*(float scale) const
{
  const unsigned char r2 = static_cast<unsigned char>(scale * r);
  const unsigned char g2 = static_cast<unsigned char>(scale * g);
  const unsigned char b2 = static_cast<unsigned char>(scale * b);
  const unsigned char a2 = static_cast<unsigned char>(scale * a);
  return ColorRGBA(r2, g2, b2, a2);
}

bool ColorRGBA::operator==(const ColorRGBA& other) const
{
  return r == other.r && g == other.g && b == other.b && a == other.a;
}

ColorRGBA ColorRGBA::blend(const ColorRGBA& other) const
{
  const unsigned char invA = (255 - a);
  const unsigned int outA = a * 255 + other.a * invA;
  return ColorRGBA(
           static_cast<unsigned char>(255 * (r * a + other.r * invA) / outA),
           static_cast<unsigned char>(255 * (g * a + other.g * invA) / outA),
           static_cast<unsigned char>(255 * (b * a + other.b * invA) / outA),
           static_cast<unsigned char>(outA / 255)
         );
}

ColorRGBA ColorRGBA::interpolate(const float factor, const ColorRGBA& other) const
{
  return ColorRGBA(
           static_cast<unsigned char>((1.f - factor) * r + factor * other.r),
           static_cast<unsigned char>((1.f - factor) * g + factor * other.g),
           static_cast<unsigned char>((1.f - factor) * b + factor * other.b),
           static_cast<unsigned char>((1.f - factor) * a + factor * other.a)
         );
}

void ColorRGBA::reg()
{
  PUBLISH(reg);
  REG_CLASS(ColorRGBA);
  REG(r);
  REG(g);
  REG(b);
  REG(a);
}

In& operator>>(In& stream, ColorRGBA& color)
{
  STREAM_EXT(stream, color.r);
  STREAM_EXT(stream, color.g);
  STREAM_EXT(stream, color.b);
  STREAM_EXT(stream, color.a);
  return stream;
}

Out& operator<<(Out& stream, const ColorRGBA& color)
{
  STREAM_EXT(stream, color.r);
  STREAM_EXT(stream, color.g);
  STREAM_EXT(stream, color.b);
  STREAM_EXT(stream, color.a);
  return stream;
}
