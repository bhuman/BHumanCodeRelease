#include "MacFullscreen.h"
#include "MainWindow.h"

bool MacFullscreen::available = false;

void MacFullscreen::enable(MainWindow* window)
{
  NSString* string = [NSString string];
  available = [string respondsToSelector:@selector(linguisticTagsInRange:scheme:options:orthography:tokenRanges:)] != NO;
  if(available)
  {
    NSView* nsview = (NSView*) window->winId();
    NSWindow* nswindow = [nsview window];
    [nswindow setCollectionBehavior:NSWindowCollectionBehaviorFullScreenPrimary];
  }
}

bool MacFullscreen::isActive(MainWindow* window)
{
  NSView* nsview = (NSView*) window->winId();
  NSWindow* nswindow = [nsview window];
  return available && ([nswindow styleMask] & NSFullScreenWindowMask) != 0;
}

void MacFullscreen::setActive(MainWindow* window, bool active)
{
  if(available && isActive(window) != active)
  {
    NSView* nsview = (NSView*) window->winId();
    NSWindow* nswindow = [nsview window];
    [nswindow toggleFullScreen:nil];
  }
}
