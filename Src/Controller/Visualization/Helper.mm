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
    NSArray<NSColor*>* baseColors = nil;

    if(__builtin_available(macOS 10.14, *))
      baseColors = [NSColor alternatingContentBackgroundColors];
    else
      baseColors = [NSColor controlAlternatingRowBackgroundColors];

    NSColor* nsColor = [baseColors[1] colorUsingColorSpaceName:NSDeviceRGBColorSpace];
    [nsColor getRed:&red green:&green blue:&blue alpha:&alpha];
  }
  QColor qtColor;
  qtColor.setRgbF(red, green, blue, alpha);
  return QBrush(qtColor);
}
