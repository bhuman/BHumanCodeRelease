/**
 * @file AnnotationManager.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationManager.h"

AnnotationManager::AnnotationManager()
{
  outData.setSize(100000);
}

void AnnotationManager::addAnnotation()
{
  outData.out.bin << (0x80000000 | annotationCounter++);
}

MessageQueue& AnnotationManager::getOut()
{
  return outData;
}
