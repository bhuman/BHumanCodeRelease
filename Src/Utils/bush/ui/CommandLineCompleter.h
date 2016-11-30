/**
 * @file CommandLineCompleter.h Contains the declarations of
 * CommandLineCompleter and TabFilter.
 *
 * @author <a href="ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#pragma once

#include <QCompleter>

class QKeyEvent;
class QStringListModel;

/**
 * Filters tab key events which would steal the focus from the completer
 * otherwise.
 *
 * Known error: If the tab key is pressed when the popup of th completer is
 * shown, the lineEdit and the popup loose their focus. This is not as
 * intended but this event is triggered by another unknown widget.
 */
class TabFilter : public QObject
{
public:
  explicit TabFilter(QObject* parent);
  bool eventFilter(QObject* o, QEvent* e);
};

/** A completer which is in charge to connect the model of the normal QCompleter
 * to @link Commands::complete(const std::string&).
 */
class CommandLineCompleter : public QCompleter
{
  Q_OBJECT

  /** The number of words as computed by @link split of the completion prefix at
   * the last call of @link setCompletionPrefix(const QString&).  Initialized
   * with @link QLINEEDIT_MAX_LENGTH since this is the default maximum length of
   * a QLineEdit and it is assumed that setCompletionPrefix is called by an
   * instance of it so that this never can have a word count of
   * QLINEEDIT_MAX_LENGTH.
   */
  size_t wordCount;

  /** The completion model. */
  QStringListModel* model;

public:
  explicit CommandLineCompleter(QObject* parent);

public slots:
  /**
   * Updates the model if needed and calls @link
   * QCompleter::setCompletionPrefix(const QString&).
   *
   * @param text The new completion prefix.
   */
  void setCompletionPrefix(const QString& text);
};
