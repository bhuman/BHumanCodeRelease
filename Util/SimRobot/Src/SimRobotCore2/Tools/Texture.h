/**
* @file Tools/Texture.h
* Declaration of class Texture
* @author Colin Graf
*/

#pragma once

#include <string>

/**
* @class Texture
* A class for managing texture data
*/
class Texture
{
public:
  unsigned int textureId; /**< The OpenGL id of the texture */
  int height; /**< The height of the texture */
  int width; /**< The width of the texture */
  unsigned char* imageData; /**< The pixel data of the texture image */
  unsigned int byteOrder; /**< The byte order of texture image (GL_BGR, GL_BGRA, ...) */
  bool hasAlpha; /**< Whether the texture has an alpha channel or not */

  /** Default constructor */
  Texture() : textureId(0), imageData(0) {}

  /** Destructor */
  ~Texture();

  /**
  * Loads a texture from a bmp or tga file (detected by file endling)
  * @param The path to the file to load
  * @return Whether the texture was successfully loaded or not
  */
  bool load(const std::string& file);

  /**
  * Adds the texture to the currently selected OpenGL context
  */
  void createGraphics();

private:
  /**
  * Loads a texture from a tga file
  * @param The path to the file to load
  * @return Whether the texture was successfully loaded or not
  */
  bool loadTGA(const std::string& file);
};
