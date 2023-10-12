/**
 * @file AnnotationManager.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationManager.h"

AnnotationManager::AnnotationManager()
{
  outData.reserve(100000);
}

MessageQueue::OutBinary AnnotationManager::add()
{
  MessageQueue::OutBinary stream = outData.bin(idAnnotation);
  stream << (0x80000000 | annotationCounter++);
  return stream;
}

MessageQueue& AnnotationManager::getOut()
{
  return outData;
}
