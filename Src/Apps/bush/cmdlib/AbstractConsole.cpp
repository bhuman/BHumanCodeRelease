#include "AbstractConsole.h"

AbstractConsole::AbstractConsole(QObject* parent) :
  QObject(parent)
{
  qRegisterMetaType<ConsolePrintTarget>();
}

void AbstractConsole::print(const QString& msg)
{
  emit sPrint(CPT_PRINT, msg);
}

void AbstractConsole::printLine(const QString& msg)
{
  emit sPrint(CPT_PRINT_LINE, msg);
}

void AbstractConsole::error(const QString& msg)
{
  emit sPrint(CPT_ERROR, msg);
}

void AbstractConsole::errorLine(const QString& msg)
{
  emit sPrint(CPT_ERROR_LINE, msg);
}
