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
  HighlightingRule rule;

  //Taken from W3C: <http://www.w3.org/TR/REC-xml/#sec-suggested-names>
  //These are removed: \\x10000-\\xEFFFF (we can't fit them into the \xhhhh format).
  const QString nameStartCharList =
      ":A-Z_a-z\\x00C0-\\x00D6\\x00D8-\\x00F6\\x00F8-\\x02FF\\x0370-\\x037D\\x037F-\\x1FFF\\x200C-\\x200D\\x2070-\\x218F\\x2C00-\\x2FEF\\x3001-\\xD7FF\\xF900-\\xFDCF\\xFDF0-\\xFFFD";

  const QString nameCharList = nameStartCharList + "\\-\\.0-9\\x00B7\\x0300-\\x036F\\x203F-\\x2040";
  const QString nameStart = "[" + nameStartCharList + "]";
  const QString nameChar = "[" + nameCharList + "]";
  const QString xmlName = nameStart + "(" + nameChar + ")*";

  //Processing instructions.
  xmlProcInstStartExpression = QRegExp("<\\?");
  xmlProcInstEndExpression = QRegExp("\\?>");
  xmlProcInstFormat.setForeground(Qt::red);
  xmlProcInstFormat.setFontItalic(false);

  //Multiline comments.
  xmlCommentStartExpression = QRegExp("<!\\-\\-");
  xmlCommentStartExpression.setMinimal(true);
  xmlCommentEndExpression = QRegExp("\\-\\->");
  xmlCommentEndExpression.setMinimal(true);
  xmlCommentFormat.setForeground(Qt::darkGreen);
  xmlCommentFormat.setFontItalic(false);

  //Opening tags
  xmlOpenTagStartExpression = QRegExp("<" + xmlName);
  xmlOpenTagStartExpression.setMinimal(false);
  xmlOpenTagEndExpression = QRegExp(">");
  xmlOpenTagEndExpression.setMinimal(true);

  //Closing tags: first shot, handling them as start-end pairs.
  xmlCloseTagStartExpression = QRegExp("</" + xmlName);
  xmlCloseTagStartExpression.setMinimal(false);
  xmlCloseTagEndExpression = QRegExp(">");
  xmlCloseTagEndExpression.setMinimal(true);

  xmlTagFormat.setForeground(Qt::darkBlue);
  xmlTagFormat.setFontItalic(false);

  //Attributes
  xmlAttributeStartExpression = QRegExp("\\s*" + xmlName + "\\s*=\\s*((\\x0022)|(\\x0027))");
  xmlAttributeStartExpression.setMinimal(true);
  xmlAttributeEndExpression = QRegExp("(([^\\x0022]*\\x0022)|([^\\x0027]*\\x0027))");
  xmlAttributeStartExpression.setMinimal(true);
  xmlAttributeFormat.setForeground(Qt::darkMagenta);
  xmlAttributeFormat.setFontItalic(false);

  //Attribute values.
  xmlAttValStartExpression = QRegExp("((\\x0022[^\\x0022]*)|(\\x0027[^\\x0027]*))");
  xmlAttValStartExpression.setMinimal(true);

  //The end expression varies depending on what's matched with the start expression (single or double quote).
  //This regexp is actually reset repeatedly during the highlighting process.
  xmlAttValFormat.setForeground(QColor(139, 69, 19));
  xmlAttValFormat.setFontItalic(false);

  //The DOCTYPE declaration.
  xmlDoctypeStartExpression = QRegExp("<!DOCTYPE");
  xmlDoctypeEndExpression = QRegExp(">");
  xmlDoctypeFormat.setForeground(Qt::darkCyan);
  xmlDoctypeFormat.setFontItalic(false);

  //Next, XML entities. This is the first item we can assume not to be multiline,
  //So this can be added to the rules. We add this to the second set of rules so
  //that it can highlight inside attribute values.
  xmlEntityFormat.setForeground(Qt::darkGray);
  xmlEntityFormat.setFontItalic(true);
  rule.pattern = QRegExp("&" + xmlName + ";");
  rule.format = xmlEntityFormat;
  hlRules.append(rule);
}

void SyntaxHighlighter::highlightBlock(const QString &text)
{
  //Do the main block highlighting.
  highlightSubBlock(text, 0, previousBlockState());

  //Run the set of inline rules.
  foreach (const HighlightingRule &rule, hlRules)
    {
      QRegExp expression(rule.pattern);
      int index = expression.indexIn(text);
      while(index >= 0)
      {
        int length = expression.matchedLength();
        setFormat(index, length, rule.format);
        index = expression.indexIn(text, index + length);
      }
    }
}

void SyntaxHighlighter::highlightSubBlock(const QString &text, const int startIndex, const int currState)
{
  if(startIndex >= text.length())
    return;
  int offset, commentOffset, procInstOffset, doctypeOffset, openTagOffset, closeTagOffset;
  int matchedLength = 0;
  int lowest = -1;
  int newState = -1;

  int effectiveState = currState;
  if(currState < 0)
    effectiveState = inNothing;
  switch(effectiveState)
  {
    case (inNothing):
    {
      //If we're not in anything, then what could be coming is either a comment, processing instruction, a doctype decl, or a tag (open or close).
      commentOffset = xmlCommentStartExpression.indexIn(text, startIndex);
      procInstOffset = xmlProcInstStartExpression.indexIn(text, startIndex);
      doctypeOffset = xmlDoctypeStartExpression.indexIn(text, startIndex);
      openTagOffset = xmlOpenTagStartExpression.indexIn(text, startIndex);
      closeTagOffset = xmlCloseTagStartExpression.indexIn(text, startIndex);
      if(commentOffset > lowest)
      {
        lowest = commentOffset;
        newState = inComment;
        matchedLength = xmlCommentStartExpression.matchedLength();
      }
      if((procInstOffset > -1) && ((lowest == -1) || (procInstOffset < lowest)))
      {
        lowest = procInstOffset;
        newState = inProcInst;
        matchedLength = xmlProcInstStartExpression.matchedLength();
      }
      if((doctypeOffset > -1) && ((lowest == -1) || (doctypeOffset < lowest)))
      {
        lowest = doctypeOffset;
        newState = inDoctypeDecl;
        matchedLength = xmlDoctypeStartExpression.matchedLength();
      }
      if((openTagOffset > -1) && ((lowest == -1) || (openTagOffset < lowest)))
      {
        lowest = openTagOffset;
        newState = inOpenTag;
        matchedLength = xmlOpenTagStartExpression.matchedLength();
      }
      if((closeTagOffset > -1) && ((lowest == -1) || (closeTagOffset < lowest)))
      {
        newState = inCloseTag;
        matchedLength = xmlCloseTagStartExpression.matchedLength();
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
          setFormat(commentOffset, matchedLength, xmlCommentFormat);
          setCurrentBlockState(inComment);
          highlightSubBlock(text, commentOffset + matchedLength, inComment);
          break;
        }
        case inProcInst:
        {
          //We're into a processing instruction.
          //Format the matched text
          setFormat(procInstOffset, matchedLength, xmlProcInstFormat);
          //Call this function again with a new offset and state.
          setCurrentBlockState(inProcInst);
          highlightSubBlock(text, procInstOffset + matchedLength, inProcInst);
          break;
        }
        case inDoctypeDecl:
        {
          //We're into a document type declaration.
          //Format the matched text
          setFormat(doctypeOffset, matchedLength, xmlDoctypeFormat);
          //Call this function again with a new offset and state.
          setCurrentBlockState(inDoctypeDecl);
          highlightSubBlock(text, doctypeOffset + matchedLength, inDoctypeDecl);
          break;
        }
        case inOpenTag:
        {
          //We're into an opening tag.
          //Format the matched text
          setFormat(openTagOffset, matchedLength, xmlTagFormat);
          //Call this function again with a new offset and state.
          setCurrentBlockState(inOpenTag);
          highlightSubBlock(text, openTagOffset + matchedLength, inOpenTag);
          break;
        }
        case inCloseTag:
        {
          //We're into a closing tag.
          //Format the matched text
          setFormat(closeTagOffset, matchedLength, xmlTagFormat);
          setCurrentBlockState(inCloseTag);
          //Call this function again with a new offset and state.
          highlightSubBlock(text, closeTagOffset + matchedLength, inCloseTag);
          break;
        }
      }
      break;
    }
    case inProcInst:
    {
      //Look for the end of the processing instruction.
      offset = xmlProcInstEndExpression.indexIn(text, startIndex);
      matchedLength = xmlProcInstEndExpression.matchedLength();
      if(offset > -1)
      {
        setFormat(startIndex, (offset + matchedLength) - startIndex, xmlProcInstFormat);
        setCurrentBlockState(inNothing);
        highlightSubBlock(text, offset + matchedLength, inNothing);
      }
      else
      {
        //We leave this block still inside the processing instruction,
        //after formatting the rest of the line.
        setFormat(startIndex, text.length() - startIndex, xmlProcInstFormat);
        setCurrentBlockState(inProcInst);
      }
      break;
    }
    case inDoctypeDecl:
    {
      //Look for the end of the doctype declaration.
      offset = xmlDoctypeEndExpression.indexIn(text, startIndex);
      matchedLength = xmlDoctypeEndExpression.matchedLength();
      if(offset > -1)
      {
        setFormat(startIndex, (offset + matchedLength) - startIndex, xmlDoctypeFormat);
        setCurrentBlockState(inNothing);
        highlightSubBlock(text, offset + matchedLength, inNothing);
      }
      else
      {
        //We leave this block still inside the doctype declaration,
        //after formatting the rest of the line.
        setFormat(startIndex, text.length() - startIndex, xmlDoctypeFormat);
        setCurrentBlockState(inDoctypeDecl);
      }
      break;
    }
    case inOpenTag:
    {
      //If we're in an open tag, we're looking either for the end of the open tag, or for
      //the beginning of an attribute name.
      int openTagEndOffset = xmlOpenTagEndExpression.indexIn(text, startIndex);
      int attStartOffset = xmlAttributeStartExpression.indexIn(text, startIndex);
      if(attStartOffset > -1)
      {
        lowest = attStartOffset;
        newState = inAttVal;
        matchedLength = xmlAttributeStartExpression.matchedLength();
      }
      if((openTagEndOffset > -1) && ((lowest == -1) || (openTagEndOffset < lowest)))
      {
        newState = inNothing;
        matchedLength = xmlOpenTagEndExpression.matchedLength();
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
          setFormat(openTagEndOffset, matchedLength, xmlTagFormat);
          setCurrentBlockState(inNothing);
          highlightSubBlock(text, openTagEndOffset + matchedLength, inNothing);
          break;
        }
        case inAttVal:
        {
          //We've started an attribute. First format the attribute name and quote.
          setFormat(attStartOffset, matchedLength, xmlAttributeFormat);
          //We need to change the value of the attribute close matcher, so that
          //it pairs correctly with the opening quote.
          QChar quote = text.at(attStartOffset + matchedLength - 1);
          if(quote.unicode() == 0x27)
          {
            xmlAttributeEndExpression = QRegExp("\\x0027");
          }
          else
          {
            xmlAttributeEndExpression = QRegExp("\\x0022");
          }
          setCurrentBlockState(inAttVal);
          highlightSubBlock(text, attStartOffset + matchedLength, inAttVal);
          break;
        }
      }
      break;
    }
    case inAttVal:
    {
      //When we're in an attribute value, we're only looking for the closing quote.
      offset = xmlAttributeEndExpression.indexIn(text, startIndex);
      if(offset > -1)
      {
        //Do some highlighting. First the attribute value.
        setFormat(startIndex, offset - startIndex, xmlAttValFormat);
        //Now the closing quote.
        setFormat(offset, 1, xmlAttributeFormat);
        setCurrentBlockState(inOpenTag);
        highlightSubBlock(text, offset + 1, inOpenTag);
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
      int closeTagEndOffset = xmlCloseTagEndExpression.indexIn(text, startIndex);
      matchedLength = xmlCloseTagEndExpression.matchedLength();
      if(closeTagEndOffset > -1)
      {
        //We've found the end of the close tag.
        setFormat(closeTagEndOffset, matchedLength, xmlTagFormat);
        setCurrentBlockState(inNothing);
        highlightSubBlock(text, closeTagEndOffset + matchedLength, inNothing);
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
      offset = xmlCommentEndExpression.indexIn(text, startIndex);
      matchedLength = xmlCommentEndExpression.matchedLength();
      if(offset > -1)
      {
        setFormat(startIndex, (offset + matchedLength) - startIndex, xmlCommentFormat);
        setCurrentBlockState(inNothing);
        highlightSubBlock(text, offset + matchedLength, inNothing);
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

void SyntaxHighlighter::setCharFormat(const int whichFormat, const QTextCharFormat &newFormat)
{
  QTextCharFormat *targFormat;
  switch(whichFormat)
  {
    case xmlDefault:
    {
      targFormat = &xmlDefaultFormat;
      break;
    }
    case xmlProcInst:
    {
      targFormat = &xmlProcInstFormat;
      break;
    }
    case xmlDoctype:
    {
      targFormat = &xmlDoctypeFormat;
      break;
    }
    case xmlComment:
    {
      targFormat = &xmlCommentFormat;
      break;
    }
    case xmlTag:
    {
      targFormat = &xmlTagFormat;
      break;
    }
    case xmlEntity:
    {
      targFormat = &xmlEntityFormat;
      break;
    }
    case xmlAttribute:
    {
      targFormat = &xmlAttributeFormat;
      break;
    }
    case xmlAttVal:
    {
      targFormat = &xmlAttValFormat;
      break;
    }
    default:
    {
      return;
    }
  }

  targFormat->setForeground(newFormat.foreground());
  rehighlight();
}

bool SyntaxHighlighter::getCharFormat(const int whichFormat, QTextCharFormat &targFormat)
{
  QTextCharFormat *srcFormat;
  switch(whichFormat)
  {
    case xmlDefault:
    {
      srcFormat = &xmlDefaultFormat;
      break;
    }
    case xmlProcInst:
    {
      srcFormat = &xmlProcInstFormat;
      break;
    }
    case xmlDoctype:
    {
      srcFormat = &xmlDoctypeFormat;
      break;
    }
    case xmlComment:
    {
      srcFormat = &xmlCommentFormat;
      break;
    }
    case xmlTag:
    {
      srcFormat = &xmlTagFormat;
      break;
    }
    case xmlEntity:
    {
      srcFormat = &xmlEntityFormat;
      break;
    }
    case xmlAttribute:
    {
      srcFormat = &xmlAttributeFormat;
      break;
    }
    case xmlAttVal:
    {
      srcFormat = &xmlAttValFormat;
      break;
    }
    default:
    {
      return false;
    }
  }

  targFormat.setForeground(srcFormat->foreground());

  return true;
}
