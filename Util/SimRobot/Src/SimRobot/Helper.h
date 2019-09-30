/**
 * This file declares a helper function to effectively let the unified toolbar
 * appear on macOS High Sierra.
 *
 * @author Brigitte Dunsbach in QTBUG-63444
 */

#include <QWidget>
#include <QBrush>

/**
 * Make the window title transparent again. This was the default until macOS Sierra.
 * @param w The widget the title of which is made transparent.
 */
void setWindowTitleTransparent(QWidget* w);
