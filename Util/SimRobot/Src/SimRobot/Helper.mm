/**
 * This file implements a helper function to effectively let the unified toolbar
 * appear on macOS High Sierra.
 *
 * @author Brigitte Dunsbach in QTBUG-63444
 */

#import <AppKit/AppKit.h>
#include <QApplication>
#include "Helper.h"

void setWindowTitleTransparent(QWidget* w)
{
  QCoreApplication::setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
  NSView* nsview = (__bridge NSView*)reinterpret_cast<void*>(w->window()->winId());
  NSWindow* nswindow = [nsview window];
  nswindow.titlebarAppearsTransparent = YES;
}
