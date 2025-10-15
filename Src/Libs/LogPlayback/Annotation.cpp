/**
 * @file Annotation.cpp
 *
 * This file implements a struct that represents an annotation created by
 * the macro ANNOTATION.
 *
 * @author Thomas Röfer
 */

#include "Annotation.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"

void Annotation::read(In& stream)
{
  InBinaryMemory& bin = static_cast<InBinaryMemory&>(stream);
  bin >> number;

  // Compatibility with old annotations.
  if(!(number & 0x80000000))
    bin >> frame;
  number &= ~0x80000000;

  const size_t size = bin.getSize() - bin.getPosition();
  std::string buffer;
  buffer.resize(size);
  bin.read(buffer.data(), size);
  InTextMemory text(buffer.data(), size);
  text >> name;
  description = text.readAll();
}

void Annotation::write(Out& stream) const
{
  stream << (number | 0x80000000);
  OutTextMemory text;
  text << name << description;
  stream.write(text.data(), text.size());
}

void Annotation::reg()
{
  REG_CLASS(Annotation);
  REG(std::string, number);
  // REG(unsigned char[], text);
}
