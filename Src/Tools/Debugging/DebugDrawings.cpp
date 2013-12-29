/**
* @file Tools/Debugging/DebugDrawings.cpp
*
* Functions for Debugging
*
* @author Michael Spranger
*/

#include "DebugDrawings.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDataTable.h" // HACK: this file needs to be included because something doesn't compile otherwise

In& operator>>(In& stream, ColorRGBA& color)
{
  stream >> color.r;
  stream >> color.g;
  stream >> color.b;
  stream >> color.a;
  return stream;
}

Out& operator<<(Out& stream, const ColorRGBA& color)
{
  stream << color.r;
  stream << color.g;
  stream << color.b;
  stream << color.a;
  return stream;
}

void DrawingManager::addDrawingId(const char* name, const char* typeName)
{
  if(drawings.find(name) == drawings.end())
  {
    char id = (char)drawings.size();
    Drawing& drawing = drawings[name];
    drawing.id = id;
    drawing.processIdentifier = processIdentifier;

    std::unordered_map< const char*, char>::const_iterator i = types.find(typeName);
    if(i == types.end())
    {
      drawing.type = (char)types.size();
      types[typeName] = drawing.type;
      unsigned int key = ((unsigned int)processIdentifier) << 24 | ((unsigned int)drawing.type);
      typesById[key] = typeName;
    }
    else
      drawing.type = i->second;

    unsigned int key = ((unsigned int)processIdentifier) << 24 | ((unsigned int)id);
    drawingsById[key] = name;
  }
}

void DrawingManager::clear()
{
  types.clear();
  drawings.clear();
  strings.clear();
  drawingsById.clear();
  typesById.clear();
}

const char* DrawingManager::getString(const std::string& string)
{
  std::unordered_map<std::string, const char*>::iterator i = strings.find(string);
  if(i == strings.end())
  {
    strings[string];
    i = strings.find(string);
    i->second = i->first.c_str();
  }
  return i->second;
}

In& operator>>(In& stream, DrawingManager& drawingManager)
{
  // note that this operator appends the data read to the drawingManager
  // clear() has to be called first to replace the existing data

  int size;
  stream >> size;;
  for(int i = 0; i < size; ++i)
  {
    std::string str;
    int id;
    stream >> id >> str;
    const char* name = drawingManager.getString(str);
    unsigned int key = ((unsigned int)drawingManager.processIdentifier) << 24 | ((unsigned int)id);
    drawingManager.types[name] = (char)id;
    drawingManager.typesById[key] = name;
  }

  stream >> size;
  for(int i = 0; i < size; ++i)
  {

    std::string str;
    int id,
        type;
    stream >> id >> type >> str;
    const char* name = drawingManager.getString(str);
    DrawingManager::Drawing& entry = drawingManager.drawings[name];
    entry.id = (char) id;
    entry.type = (char) type;
    entry.processIdentifier = drawingManager.processIdentifier;
    unsigned int key = ((unsigned int)entry.processIdentifier) << 24 | ((unsigned int)entry.id);
    drawingManager.drawingsById[key] = name;
  }

  return stream;
}

Out& operator<<(Out& stream, const DrawingManager& drawingManager)
{
  stream << (int)drawingManager.types.size();
  for(std::unordered_map< const char*, char>::const_iterator iter = drawingManager.types.begin(); iter != drawingManager.types.end(); ++iter)
  {
    stream << (int) iter->second;
    stream << iter->first;
  }

  stream << (int)drawingManager.drawings.size();
  for(std::unordered_map< const char*, DrawingManager::Drawing>::const_iterator iter = drawingManager.drawings.begin(); iter != drawingManager.drawings.end(); ++iter)
  {
    stream << (int) iter->second.id;
    stream << (int) iter->second.type;
    stream << iter->first;
  }

  return stream;
}
