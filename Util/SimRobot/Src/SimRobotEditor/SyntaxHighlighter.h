/*
 * SyntaxHighlighter.h
 *
 *  Created on: 25.09.2010
 *      Author: Emil Huseynli (Originally: Martin Holmes, Meagan Timney, University of Victoria, HCMC)
 */

#pragma once

#include <QSyntaxHighlighter>
#include <QHash>
#include <QTextCharFormat>
#include <QVector>
#include <QTextDocument>

class SyntaxHighlighter: public QSyntaxHighlighter
{
  Q_OBJECT

public:
  SyntaxHighlighter(QTextDocument* parent = 0);

  //Enumeration for the character formats. This is used in functions which get and set them.
  enum xmlCharFormat
  {
    xmlDefault,
    xmlProcInst,
    xmlDoctype,
    xmlComment,
    xmlTag,
    xmlEntity,
    xmlAttribute,
    xmlAttVal
  };

protected:
  void highlightBlock(const QString &text);
  void highlightSubBlock(const QString &text, const int startIndex, const int currState);

  void setCharFormat(const int whichFormat, const QTextCharFormat &newFormat);
  bool getCharFormat(const int whichFormat, QTextCharFormat &targFormat);

private:
  struct HighlightingRule
  {
      QRegExp pattern;
      QTextCharFormat format;
  };
  QVector<HighlightingRule> hlRules;

  QRegExp xmlProcInstStartExpression;
  QRegExp xmlProcInstEndExpression;
  QRegExp xmlCommentStartExpression;
  QRegExp xmlCommentEndExpression;
  QRegExp xmlDoctypeStartExpression;
  QRegExp xmlDoctypeEndExpression;

  QRegExp xmlOpenTagStartExpression;
  QRegExp xmlOpenTagEndExpression;
  QRegExp xmlCloseTagStartExpression;
  QRegExp xmlCloseTagEndExpression;
  QRegExp xmlAttributeStartExpression;
  QRegExp xmlAttributeEndExpression;
  QRegExp xmlAttValStartExpression;
  QRegExp xmlAttValEndExpression;

  QRegExp xmlAttValExpression;

  QTextCharFormat xmlDefaultFormat;
  QTextCharFormat xmlProcInstFormat;
  QTextCharFormat xmlDoctypeFormat;
  QTextCharFormat xmlCommentFormat;
  QTextCharFormat xmlTagFormat;
  QTextCharFormat xmlEntityFormat;
  QTextCharFormat xmlAttributeFormat;
  QTextCharFormat xmlAttValFormat;

  //Enumeration for types of element, used for tracking what
  //we're inside while highlighting over multiline blocks.
  enum xmlState
  {
    inNothing, //Don't know if we'll need this or not.
    inProcInst, //starting with <? and ending with ?>
    inDoctypeDecl, //starting with <!DOCTYPE and ending with >
    inOpenTag, //starting with < + xmlName and ending with /?>
    inOpenTagName, //after < and before whitespace. Implies inOpenTag.
    inAttribute, //if inOpenTag, starting with /s*xmlName/s*=/s*" and ending with ".
    inAttName, //after < + xmlName + whitespace, and before =". Implies inOpenTag.
    inAttVal, //after =" and before ". May also use single quotes. Implies inOpenTag.
    inCloseTag, //starting with </ and ending with >.
    inCloseTagName, //after </ and before >. Implies inCloseTag.
    inComment //starting with <!-- and ending with -->. Overrides all others.
  };
};
