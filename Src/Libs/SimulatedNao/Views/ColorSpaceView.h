/**
 * @file SimulatedNao/Views/ColorSpaceView.h
 *
 * Declaration of class ColorSpaceView
 *
 * @author Thomas Röfer
 */

#pragma once

#include <QString>
#include <QIcon>
#include <SimRobot.h>

#include "Math/Eigen.h"
#include "Streaming/Enum.h"

class RobotConsole;

/**
 * @class ColorSpaceView
 *
 * A class to represent a view with information about the timing of modules.
 *
 * @author Thomas Röfer
 */
class ColorSpaceView : public SimRobot::Object
{
public:
  /**
   * The image formats available.
   */
  ENUM(ColorModel,
  {,
    YUV,
    RGB,
    HSI,
  });

  /**
   * @param fullName The path to this view in the scene graph
   * @param c The console object.
   * @param n The name of the image to display.
   * @param cm The color model in which the image should be displayed by this view.
   * @param ch The channel to display (1..3) or 0 to display all channels.
   * @param b The background color.
   */
  ColorSpaceView(const QString& fullName, RobotConsole& c, const std::string& n, ColorModel cm, int ch, const Vector3f& b, const std::string& threadName);

  /**
   * The function returns the name of a channel in a certain color model as string.
   * @param cm The color model.
   * @param channel The channel in the color model.
   * @return The string representation of the name of the channel in the color model.
   */
  static const char* getChannelNameForColorModel(ColorModel cm, int channel)
  {
    static const char* names[][4] =
    {
      {"all", "Y", "U", "V"},
      {"all", "R", "G", "B"},
      {"all", "H", "S", "I"}
    };
    return names[cm][channel];
  }

private:
  /**
   * Need the display lists to be updated?
   * @return Yes or no?
   */
  bool needsUpdate() const;

  const QString fullName; /**< The path to this view in the scene graph */
  QIcon icon; /**< The icon used for listing this view in the scene graph */

  RobotConsole& console; /**< A reference to the console object. */
  const std::string name; /**< The name of the image. */
  const ColorModel colorModel; /**< The color model in which the image should be displayed by this view. */
  const int channel; /**< The channel to display (1..3) or 0 to display all channels. */
  const Vector3f& background; /**< The background color. */
  Vector3f lastBackground; /**< The last background color used. */
  unsigned lastTimestamp = 0; /**< The frame number of last image that was drawn. */
  const std::string threadName; /**< The thread that created the images shown in this view. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class ColorSpaceWidget;
};
