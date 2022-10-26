#pragma once

#include <QObject>
#include <QMetaType>

class QString;

enum ConsolePrintTarget
{
  CPT_PRINT,
  CPT_PRINT_LINE,
  CPT_ERROR,
  CPT_ERROR_LINE
};

Q_DECLARE_METATYPE(ConsolePrintTarget);

class AbstractConsole : public QObject
{
  Q_OBJECT

public:
  explicit AbstractConsole(QObject* parent = nullptr);
  virtual void print(const QString& msg);
  virtual void printLine(const QString& msg);
  virtual void error(const QString& msg);
  virtual void errorLine(const QString& msg);

signals:
  void sPrint(ConsolePrintTarget target, const QString& msg);
};
