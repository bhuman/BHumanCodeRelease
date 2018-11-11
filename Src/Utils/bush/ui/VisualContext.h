#include <QFrame>
#include <QFormLayout>
#include <QPushButton>
#include "Utils/bush/cmdlib/AbstractConsole.h"
#pragma once

class Console;
class Context;
class TeamSelector;
class QLabel;

class VisualContext : public QFrame
{
  Q_OBJECT

public:
  struct Entry
  {
    enum Type
    {
      TEXT_OUTPUT,
      TEXT_ERROR,
      CONTEXT,
      CONCURRENT
    } type;

    union
    {
      QString* text;
      VisualContext* context;
    };

    Entry(Type type, const QString& text)
      : type(type),
        text(new QString(text))
    {}

    Entry(VisualContext* context)
      : type(CONTEXT),
        context(context)
    {}

    ~Entry();
  };

private:
  /** The entries of the visual context. Actually they can be text or sub
   * contexts. */
  QList<Entry*> entries;

  /** The widges which visualize the entries. */
  QList<QWidget*> widgets;

  QFormLayout* formLayout;

  /** Indicates if the last printed line had a newline at their end. */
  bool nl;

  QFormLayout* getLayout() { return formLayout; }
  void updateWidget(size_t index, Entry* entry);
  void addWidget(Entry* entry, const QString& commandLine = "");

public:
  VisualContext(QWidget* parent);

  void executeInContext(Console* console, TeamSelector* teamSelector, const QString& cmdLine);

public slots:
  void doPrint(ConsolePrintTarget target, const QString& msg);
  void commandExecuted(Context* context, const QString& cmdLine);
  void commandFinished(bool status);
  void commandCanceled();
  void cancel();

signals:
  void statusChanged(bool status);
  void sCanceled();
  void sCancel();
};

/** Draws a cool Frame around a VisualContext. */
class VisualContextDecoration : public QFrame
{
  Q_OBJECT

  QPushButton* button; //TODO: button still inactive

  /** Shows which commandLine is executed. */
  QLabel* header;

  /** The visual context representation which should be decorated. */
  VisualContext* visualContext;

  /** The parent in the tree. */
  VisualContext* parentContext;

public:
  VisualContextDecoration(const QString& commandLine, VisualContext* parent, VisualContext* context);

public slots:
  void updateStatus(bool status);
  void canceled();
};
