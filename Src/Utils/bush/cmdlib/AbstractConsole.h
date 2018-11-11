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
  explicit AbstractConsole(QObject* parent = nullptr);
  void print(const std::string& msg) override;
  void printLine(const std::string& msg) override;
  void error(const std::string& msg) override;
  void errorLine(const std::string& msg) override;

signals:
  void sPrint(ConsolePrintTarget target, const QString& msg);
};
