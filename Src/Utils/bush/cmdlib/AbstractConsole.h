#pragma once

#include <QObject>
#include <QMetaType>
#include "Utils/bush/cmdlib/IConsole.h"

class QString;

enum ConsolePrintTarget
{
  CPT_PRINT,
  CPT_PRINT_LINE,
  CPT_ERROR,
  CPT_ERROR_LINE
};

Q_DECLARE_METATYPE(ConsolePrintTarget);

class AbstractConsole : public QObject,
                        public IConsole
{
  Q_OBJECT

public:

  explicit AbstractConsole(QObject* parent = 0);
  virtual void print(const std::string& msg);
  virtual void printLine(const std::string& msg);
  virtual void error(const std::string& msg);
  virtual void errorLine(const std::string& msg);

signals:
  void sPrint(ConsolePrintTarget target, const QString& msg);
};
