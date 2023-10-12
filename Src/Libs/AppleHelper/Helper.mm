/**
 * This file implements a helper function to determine the correct alternate background
 * color of table views.
 */

#define pi _pi
#import <AppKit/AppKit.h>
#undef pi
#include "Helper.h"

#if !QT_MACOS_PLATFORM_SDK_EQUAL_OR_ABOVE(__MAC_10_14)
@interface NSColor (MojaveForwardDeclarations)
@property (class, strong, readonly) NSArray<NSColor*>* alternatingContentBackgroundColors NS_AVAILABLE_MAC(10_14);
@end
#endif

QBrush getAlternateBase()
{
  CGFloat red, green, blue, alpha;
  @autoreleasepool
  {
    NSArray<NSColor*>* baseColors = [NSColor alternatingContentBackgroundColors];
    NSColor* nsColor = [baseColors[1] colorUsingColorSpace:NSColorSpace.deviceRGBColorSpace];
    [nsColor getRed:&red green:&green blue:&blue alpha:&alpha];
  }
  QColor qtColor;
  qtColor.setRgbF(static_cast<float>(red), static_cast<float>(green), static_cast<float>(blue), static_cast<float>(alpha));
  return QBrush(qtColor);
}

void runInMainThread(std::function<void()> function)
{
  dispatch_async(dispatch_get_main_queue(),
  ^{
    function();
  });
}
