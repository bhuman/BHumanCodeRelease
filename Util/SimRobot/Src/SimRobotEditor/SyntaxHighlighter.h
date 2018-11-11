/*
 * SyntaxHighlighter.h
 *
 *  Created on: 25.09.2010
 *      Author: Emil Huseynli (Originally: Martin Holmes, Meagan Timney, University of Victoria, HCMC)
 */

#pragma once

#include <QRegularExpression>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>

class QString;
class QTextDocument;

class SyntaxHighlighter: public QSyntaxHighlighter
{
  Q_OBJECT

public:
  SyntaxHighlighter(QTextDocument* parent = nullptr);

protected:
  void highlightBlock(const QString& text) override;

private:
  void highlightSubBlock(const QString& text, const int startIndex, const int currState);

  QRegularExpression xmlCommentStartExpression;
  QRegularExpression xmlCommentEndExpression;

  QRegularExpression xmlOpenTagStartExpression;
  QRegularExpression xmlOpenTagEndExpression;
  QRegularExpression xmlCloseTagStartExpression;
  QRegularExpression xmlCloseTagEndExpression;
  QRegularExpression xmlAttributeStartExpression;
  QRegularExpression xmlAttributeEndExpression;

  QTextCharFormat xmlDefaultFormat;
  QTextCharFormat xmlCommentFormat;
  QTextCharFormat xmlTagFormat;
  QTextCharFormat xmlAttributeFormat;
  QTextCharFormat xmlAttValFormat;

  //Enumeration for types of element, used for tracking what
  //we're inside while highlighting over multiline blocks.
  enum xmlState
  {
    inNothing, //Don't know if we'll need this or not.
    inOpenTag, //starting with < + xmlName and ending with /?>
    inAttVal, //after =" and before ". May also use single quotes. Implies inOpenTag.
    inCloseTag, //starting with </ and ending with >.
    inComment //starting with <!-- and ending with -->. Overrides all others.
  };
};
