/*
 * SyntaxHighlighter.cpp
 *
 *  Created on: 25.09.2010
 *      Author: Emil Huseynli (Originally: Martin Holmes, Meagan Timney, University of Victoria, HCMC)
 *
 */

#include "SyntaxHighlighter.h"

SyntaxHighlighter::SyntaxHighlighter(QTextDocument* parent) :
  QSyntaxHighlighter(parent)
{
  const QString nameStartCharList = ":A-Z_a-z";
  const QString nameCharList = nameStartCharList + "\\-\\.0-9";
  const QString nameStart = "[" + nameStartCharList + "]";
  const QString nameChar = "[" + nameCharList + "]";
  const QString xmlName = nameStart + "(" + nameChar + ")*";

  //Multiline comments.
  xmlCommentStartExpression = QRegularExpression("<!\\-\\-");
  xmlCommentStartExpression.optimize();
  xmlCommentEndExpression = QRegularExpression("\\-\\->");
  xmlCommentEndExpression.optimize();
  xmlCommentFormat.setForeground(Qt::darkGreen);
  xmlCommentFormat.setFontItalic(false);

  //Opening tags
  xmlOpenTagStartExpression = QRegularExpression("<" + xmlName);
  xmlOpenTagStartExpression.optimize();
  xmlOpenTagEndExpression = QRegularExpression(">");
  xmlOpenTagEndExpression.optimize();

  //Closing tags: first shot, handling them as start-end pairs.
  xmlCloseTagStartExpression = QRegularExpression("</" + xmlName);
  xmlCloseTagStartExpression.optimize();
  xmlCloseTagEndExpression = QRegularExpression(">");
  xmlCloseTagEndExpression.optimize();

  xmlTagFormat.setForeground(Qt::darkBlue);
  xmlTagFormat.setFontItalic(false);

  //Attributes
  xmlAttributeStartExpression = QRegularExpression("\\s*" + xmlName + "\\s*=\\s*\\\"");
  xmlAttributeStartExpression.optimize();
  xmlAttributeEndExpression = QRegularExpression("(?<!\\\\)(?:(\\\\\\\\)*)(\")");
  xmlAttributeEndExpression.optimize();

  xmlAttributeFormat.setForeground(Qt::darkMagenta);
  xmlAttributeFormat.setFontItalic(false);

  xmlAttValFormat.setForeground(QColor(139, 69, 19));
  xmlAttValFormat.setFontItalic(false);
}

void SyntaxHighlighter::highlightBlock(const QString& text)
{
  //Do the main block highlighting.
  highlightSubBlock(text, 0, previousBlockState());
}

void SyntaxHighlighter::highlightSubBlock(const QString& text, const int startIndex, const int currState)
{
  if(startIndex >= text.length())
    return;
  int lowest = -1;
  int newState = -1;

  int effectiveState = currState;
  if(currState < 0)
    effectiveState = inNothing;
  switch(effectiveState)
  {
    case inNothing:
    {
      //If we're not in anything, then what could be coming is either a comment or a tag (open or close).
      QRegularExpressionMatch commentMatch = xmlCommentStartExpression.match(text, startIndex);
      QRegularExpressionMatch openTagMatch = xmlOpenTagStartExpression.match(text, startIndex);
      QRegularExpressionMatch closeTagMatch = xmlCloseTagStartExpression.match(text, startIndex);
      if(commentMatch.hasMatch())
      {
        lowest = commentMatch.capturedStart();
        newState = inComment;
      }
      if(openTagMatch.hasMatch() && ((lowest == -1) || (openTagMatch.capturedStart() < lowest)))
      {
        lowest = openTagMatch.capturedStart();
        newState = inOpenTag;
      }
      if(closeTagMatch.hasMatch() && ((lowest == -1) || (closeTagMatch.capturedStart() < lowest)))
      {
        newState = inCloseTag;
      }
      switch(newState)
      {
        case -1:
        {
          //Nothing starts in this block.
          setCurrentBlockState(inNothing);
          break;
        }
        case inComment:
        {
          //We're into a comment.
          setFormat(commentMatch.capturedStart(), commentMatch.capturedLength(), xmlCommentFormat);
          setCurrentBlockState(inComment);
          highlightSubBlock(text, commentMatch.capturedEnd(), inComment);
          break;
        }
        case inOpenTag:
        {
          //We're into an opening tag.
          //Format the matched text
          setFormat(openTagMatch.capturedStart(), openTagMatch.capturedLength(), xmlTagFormat);
          //Call this function again with a new offset and state.
          setCurrentBlockState(inOpenTag);
          highlightSubBlock(text, openTagMatch.capturedEnd(), inOpenTag);
          break;
        }
        case inCloseTag:
        {
          //We're into a closing tag.
          //Format the matched text
          setFormat(closeTagMatch.capturedStart(), closeTagMatch.capturedLength(), xmlTagFormat);
          setCurrentBlockState(inCloseTag);
          //Call this function again with a new offset and state.
          highlightSubBlock(text, closeTagMatch.capturedEnd(), inCloseTag);
          break;
        }
      }
      break;
    }
    case inOpenTag:
    {
      //If we're in an open tag, we're looking either for the end of the open tag, or for
      //the beginning of an attribute name.
      QRegularExpressionMatch openTagEndMatch = xmlOpenTagEndExpression.match(text, startIndex);
      QRegularExpressionMatch attStartMatch = xmlAttributeStartExpression.match(text, startIndex);
      if(attStartMatch.hasMatch())
      {
        lowest = attStartMatch.capturedStart();
        newState = inAttVal;
      }
      if(openTagEndMatch.hasMatch() && ((lowest == -1) || (openTagEndMatch.capturedStart() < lowest)))
      {
        newState = inNothing;
      }
      switch(newState)
      {
        case -1:
        {
          //we're still in a tag. No need to highlight anything.
          setCurrentBlockState(inOpenTag);
          break;
        }
        case inNothing:
        {
          //We've come to the end of the open tag.
          setFormat(openTagEndMatch.capturedStart(), openTagEndMatch.capturedLength(), xmlTagFormat);
          setCurrentBlockState(inNothing);
          highlightSubBlock(text, openTagEndMatch.capturedEnd(), inNothing);
          break;
        }
        case inAttVal:
        {
          //We've started an attribute. First format the attribute name and quote.
          setFormat(attStartMatch.capturedStart(), attStartMatch.capturedEnd(), xmlAttributeFormat);
          setCurrentBlockState(inAttVal);
          highlightSubBlock(text, attStartMatch.capturedEnd(), inAttVal);
          break;
        }
      }
      break;
    }
    case inAttVal:
    {
      //When we're in an attribute value, we're only looking for the closing quote.
      QRegularExpressionMatch match = xmlAttributeEndExpression.match(text, startIndex);
      if(match.hasMatch())
      {
        //Do some highlighting. First the attribute value.
        setFormat(startIndex, match.capturedStart(2) - startIndex, xmlAttValFormat);
        //Now the closing quote.
        setFormat(match.capturedStart(2), match.capturedLength(2), xmlAttributeFormat);
        setCurrentBlockState(inOpenTag);
        highlightSubBlock(text, match.capturedEnd(2), inOpenTag);
      }
      else
      {
        //The attribute value runs over the end of the line.
        setFormat(startIndex, text.length() - startIndex, xmlAttValFormat);
        setCurrentBlockState(inAttVal);
      }
      break;
    }
    case inCloseTag:
    {
      QRegularExpressionMatch match = xmlCloseTagEndExpression.match(text, startIndex);
      if(match.hasMatch())
      {
        //We've found the end of the close tag.
        setFormat(match.capturedStart(), match.capturedLength(), xmlTagFormat);
        setCurrentBlockState(inNothing);
        highlightSubBlock(text, match.capturedEnd(), inNothing);
      }
      else
      {
        //There must be a linebreak inside the close tag.
        setCurrentBlockState(inCloseTag);
      }
      break;
    }
    case inComment:
    {
      //Once we're in a comment, we just have to search for the end of it. Nothing else takes precedence.
      //Look for the end of the comment.
      QRegularExpressionMatch match = xmlCommentEndExpression.match(text, startIndex);
      if(match.hasMatch())
      {
        setFormat(startIndex, match.capturedEnd() - startIndex, xmlCommentFormat);
        setCurrentBlockState(inNothing);
        highlightSubBlock(text, match.capturedEnd(), inNothing);
      }
      else
      {
        //We leave this block still inside the comment,
        //after formatting the rest of the line.
        setFormat(startIndex, text.length() - startIndex, xmlCommentFormat);
        setCurrentBlockState(inComment);
      }
      break;
    }
  }
}
