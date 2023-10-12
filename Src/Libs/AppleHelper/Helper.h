/**
 * This file declares a helper function to determine the correct alternate background
 * color for table views.
 */

#pragma once

#include <QBrush>
#include <functional>

/**
 * Returns the alternate background color for table views.
 * @return A brush with the correct color.
 */
QBrush getAlternateBase();

/**
 * Run a function in the main thread.
 * @param function The function to run. It runs asynchronously
 *                 to the caller.
 */
void runInMainThread(std::function<void()> function);
