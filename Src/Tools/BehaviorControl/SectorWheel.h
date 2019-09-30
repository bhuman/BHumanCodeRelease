/**
 * @file SectorWheel.h
 *
 * This file declares a class that represents a wheel of sectors around a center point.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Range.h"
#include "Tools/Streams/Enum.h"

#include <list>

class SectorWheel
{
public:
  STREAMABLE(Sector,
  {
    ENUM(Type,
    {,
      free, /**< The sector is free. */
      obstacle, /**< There is an obstacle. */
      goal, /**< This is the goal. */
      erased, /**< This sector is currently not minded. */
    });
    Sector() = default;
    Sector(const Rangea& angleRange, float distance, Type type),
    (Rangea) angleRange, /**< The angular range that this sector spans. */
    (float) distance, /**< The distance from the wheel center at which this sector ends. */
    (Type) type, /**< The type of this sector. */
  });

  /** Virtual destructor for polymorphism. */
  virtual ~SectorWheel() = default;

  /**
   * Calculates the wheel for a given center position (e.g. hypothetical ball position).
   * @param positionOnField The center position.
   */
  void begin(const Vector2f& positionOnField);

  /**
   * Post-processes and returns the calculated wheel.
   * @return The resulting wheel.
   */
  const std::list<Sector>& finish();

  /**
   * Adds a sector to the wheel.
   *
   * This is only a wrapper around the other addSector method.
   * @param center The position of the object (in the same coordinate system as the wheel center).
   * @param width The width of the object.
   * @param type The type of the new sector.
   */
  void addSector(const Vector2f& center, float width, Sector::Type type);

  /**
   * Adds a sector to the wheel.
   *
   * This is only a wrapper around the other addSector method.
   * @param center The position of the object (in the same coordinate system as the wheel center).
   * @param width The width of the object.
   * @param distance The distance of the new sector.
   * @param type The type of the new sector.
   */
  void addSector(const Vector2f& center, float width, float distance, Sector::Type type);

  /**
   * Adds a sector to the wheel.
   *
   * This is only a wrapper around addSectorNormalized to handle the negative x axis and reverse ranges.
   * @param angleRange The angular range of the new sector.
   * @param distance The distance of the new sector.
   * @param type The type of the new sector.
   */
  void addSector(const Rangea& angleRange, float distance, Sector::Type type);

private:
  /**
   * Adds a sector to the wheel.
   *
   * This methods does the real work.
   * @param angleRange The angular range of the new sector.
   * @param distance The distance of the new sector.
   * @param type The type of the new sector.
   */
  void addSectorNormalized(const Rangea& angleRange, float distance, Sector::Type type);

  Vector2f positionOnField; /**< The current center of the wheel. */
  std::list<Sector> wheel; /**< The actual wheel of sectors (must form a sorted partition of [-pi,pi[). */
};

inline SectorWheel::Sector::Sector(const Rangea& angleRange, float distance, Type type) :
  angleRange(angleRange),
  distance(distance),
  type(type)
{}

/**
 * Draws a sector wheel as a field drawing.
 * @param id A drawing id.
 * @param wheel The wheel to draw.
 * @param position The position to draw the wheel at.
 */
#define DRAW_SECTOR_WHEEL(id, wheel, position) \
  DEBUG_DRAWING(id, "drawingOnField") \
  { \
    for(const auto& sector : wheel) \
    { \
      ColorRGBA color; \
      switch (sector.type) \
      { \
        case SectorWheel::Sector::free: \
          color = ColorRGBA(0, 255, 0, 125); \
          break; \
        case SectorWheel::Sector::obstacle: \
          color = ColorRGBA(255, 0, 0, 125); \
          break; \
        case SectorWheel::Sector::goal: \
          color = ColorRGBA(0, 0, 255, 125); \
          break; \
        case SectorWheel::Sector::erased: \
          color = ColorRGBA(128, 128, 128, 125); \
          break; \
      } \
      const Angle angleSpan = sector.angleRange.max - sector.angleRange.min + (sector.angleRange.max < sector.angleRange.min ? pi2 : 0); \
      ARC(id, position.x(), position.y(), std::min(sector.distance, 12000.f), \
          sector.angleRange.min, angleSpan, 0, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, color); \
    } \
  }
