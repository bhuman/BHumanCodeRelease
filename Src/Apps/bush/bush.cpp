#include "Initializer.h"
#include <QString>

int main(int argc, char** argv)
{
  Initializer initializer(argc, argv);
  return initializer.start();
}

#ifdef MACOS
/**
 * A helper for diplaying QStrings in Xcode that converts UTF16 strings to UTF8
 * character arrays that Xcode can display.
 * @param string The string to convert.
 * @return The address of the converted string. Note that it points to a static
 *         buffer that will be reused in the next call.
 */
const char* _fromQString(const QString& string)
{
  static char buffer[1000];
  strncpy(buffer, string.toUtf8().constData(), sizeof(buffer));
  buffer[sizeof(buffer) - 1] = 0;
  return buffer;
}
#endif // MACOS
