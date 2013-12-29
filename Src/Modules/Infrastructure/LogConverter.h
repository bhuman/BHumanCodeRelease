/**
 * @file LogConverter.h
 *
 * @author reich
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/MessageQueue/InMessage.h"

class LogConverter
{
private:
  std::size_t sizeofSensorDataRev5703;
  std::size_t sizeofSensorData_2d284;
  std::size_t sizeofFrameInfo_83e22;
  std::size_t sizeofJointData_d198df791237;

  /**
   * @returns Newly allocated and converted SensorData representation.
   */
  Streamable* newConvertedSensorData(InMessage& message);

  /**
   * @returns Newly allocated and converted FrameInfo representation.
   */
  Streamable* newConvertedFrameInfo(InMessage& message);

  /**
   * @returns Newly allocated and converted JointData representation.
   */
  Streamable* newConvertedJointData(InMessage& message);

  /**
   * @returns Newly allocated and created CameraInfo representation.
   */
  Streamable* newConvertedCameraInfo(InMessage& message);

  std::size_t sizeofRepresentation(const Streamable& streamable);

public:
  LogConverter();

  /**
   * @param message The old representation destined for conversion.
   * @param representationId The message id of the representation.
   * @returns A newly allocated and converted representation.
   */
  Streamable* newConvertedRepresentation(InMessage& message, int representationId);

  bool isConversionRequired(InMessage& message, int representationId);
};
