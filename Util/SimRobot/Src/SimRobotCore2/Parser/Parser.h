/**
* @file Simulation/Parser.h
* Declaration of class Parser
* @author Colin Graf
*/

#pragma once

#include <list>
#include <unordered_set>
#include "Reader.h"

class Element;

/**
* @class Parser
* A parser for .ros2 files.
*/
class Parser : protected Reader
{
public:
  /** Default constructor. */
  Parser();

  /** Destructor. */
  ~Parser();

  /**
  * Parses a .ros2 file, builds the scene graph up and fills parameter sets out
  * @param fileName The name of the file
  * @param errors List of errors occured while parsing
  * @return Whether the file was parsed without errors or not
  */
  bool parse(const std::string& fileName, std::list<std::string>& errors);

private:
  using StartElementProc = Element* (Parser::*)();
  using TextProc = void (Parser::*)(std::string& text, Location location);

  /** Some flags to describe properties of an element class */
  enum ElementFlags
  {
    textFlag = (1 << 0), /**< The element has a text / data segment like Vertices, TexCoords, ... */
    constantFlag = (1 << 1), /**< The element is constant in a way that it can be used multiple times in scene graph to avoid multiple element instantiations */
  };

  struct ElementInfo
  {
    const char* name;
    unsigned int elementClass;
    StartElementProc startElementProc;
    TextProc textProc;
    unsigned int flags;
    unsigned int requiredChildren;
    unsigned int optionalChildren;
    unsigned int repeatableChildren;
  };

  class ElementData
  {
  public:
    ElementData* parent;
    const ElementInfo* info;
    unsigned int parsedChildren;
    unsigned int parsedAttributes;
    std::unordered_map<std::string, std::string> vars; /**< User defined variables for attribute strings */
    bool usedPlaceholdersInAttributes;
    Location location; /**< The location of the instanced element */

    ElementData(ElementData* parent, const Location& location, const ElementInfo* info) :
      parent(parent), info(info), parsedChildren(0), parsedAttributes(0), usedPlaceholdersInAttributes(false), location(location) {}
  };

  class MacroElement
  {
  public:
    MacroElement* parent;
    const ElementInfo* elementInfo;
    Attributes attributes;
    std::string text;
    Location textLocation;
    std::list<MacroElement*> children;
    Element* element; /**< An element that was created from the macro */
    Location location;

    MacroElement(MacroElement* parent, const ElementInfo* elementInfo,
      Attributes& attributes, const Location& location) :
      parent(parent), elementInfo(elementInfo), element(nullptr), location(location)
    {
      this->attributes.swap(attributes);
    }
    ~MacroElement()
    {
      for(std::list<MacroElement*>::const_iterator iter = children.begin(), end = children.end(); iter != end; ++iter)
        delete *iter;
    }

    bool hasTextOrChildren() const {return !children.empty() || !text.empty();}
  };

  class Macro : public MacroElement
  {
  public:
    std::string fileName; /**< The file in which the macro was declared */
    bool replaying; /**< A flag for detecting macro reference loops */

    Macro(const ElementInfo* elementInfo, const std::string& fileName, Attributes& attributes, const Location& location) :
      MacroElement(nullptr, elementInfo, attributes, location), fileName(fileName), replaying(false) {}
  };

  std::unordered_map<std::string, const ElementInfo*> elementInfos; /** Mapping element name strings to handler member function pointers */

  std::list<std::string>* errors; /**< List of errors occured while parsing */
  std::string parseRootDir; /**< The directory in which the main .ros2 file is stored. */
  std::string includeFile; /**< A file to be included */
  Location includeFileLocation; /**< The location of the path to the included file in the including file */

  std::unordered_map<std::string, Macro*> macros; /**< A storage for macros */
  Macro* sceneMacro; /**< A macro created for the <Scene> element */

  MacroElement* recordingMacroElement; /**< A macro element set to record subordinate nodes of a macro */
  MacroElement* replayingMacroElement; /** A macro element set to insert subordinate nodes of a macro */
  Element* element; /**< The last inserted xml element */
  ElementData* elementData; /** Element context data required for parsing a xml element */
  const Attributes* attributes; /**< The current set of attributes */

  bool passedSimulationTag; /**< Whether the <Simulation>-tag was passed or not */
  Location simulationTagLocation; /**< The location of the <Simulation>-tag */

  void parseSimulation();
  void parseMacroElements();
  void parseMacroElement(ElementData& elementData);

  /**
   * A handler called for each parsed element
   * Call \c readElements within the handler to parse subordinate elements.
   * @param name The name of the element
   * @param attributes The attributes of the element
   * @param location The location of the element in the current file (first character after <)
   * @return The result of \c readElements that has been called inside
   */
  bool handleElement(const std::string& name, Attributes& attributes, const Location& location) override;

  /**
   * A handler called when text was parsed
   * @param text The text read
   * @param location The location of the text in the current file
   */
  void handleText(std::string& text, const Location& location) override;
  /**
   * A handler called for syntax errors
   * @param msg An error message
   * @param location The location of the error in the current file
   */
  void handleError(const std::string& msg, const Location& location) override;

  /** Checks if there are any unexpected attributes in the current set of attributes */
  void checkAttributes();

  /** Checks if some required subordinate elements have not been parsed */
  void checkElements();

  const std::string* resolvePlaceholder(const std::string& name);
  const std::string& replacePlaceholders(const std::string& str, const Location& location);
  std::string placeholderBuffer;

  // functions for reading attributes
  bool getStringRaw(const char* key, bool required, const std::string*& value);
  bool getFloatRaw(const char* key, bool required, float& value);
  bool getIntegerRaw(const char* key, bool required, int& value);
  const std::string& getString(const char* key, bool required);
  bool getBool(const char* key, bool required, bool defaultValue);
  float getFloat(const char* key, bool required, float defaultValue);
  float getFloatPositive(const char* key, bool required, float defaultValue);
  int getInteger(const char* key, bool required, int defaultValue);
  int getIntegerNonZeroPositive(const char* key, bool required, int defaultValue);
  float getFloatMinMax(const char* key, bool required, float defaultValue, float min, float max);
  bool getFloatAndUnit(const char* key, bool required, float defaultValue, float& value, char** unit, Location& unitLocation);
  float getUnit(const char* key, bool required, float defaultValue);
  float getLength(const char* key, bool required, float defaultValue);
  float getMassLengthLength(const char* key, bool required, float defaultValue);
  float getMass(const char* key, bool required, float defaultValue);
  float getAngle(const char* key, bool required, float defaultValue);
  float getAngleNonZeroPositive(const char* key, bool required, float defaultValue);
  float getAngularVelocity(const char* key, bool required, float defaultValue);
  float getForce(const char* key, bool required, float defaultValue);
  float getVelocity(const char* key, bool required, float defaultValue);
  float getAcceleration(const char* key, bool required, float defaultValue);
  float getTimeNonZeroPositive(const char* key, bool required, float defaultValue);
  bool getColor(const char* key, bool required, float* color);

  // element classes
  enum ElementClass
  {
    infrastructureClass = 0,
    sceneClass          = (1 << 0),
    setClass            = (1 << 1),
    compoundClass       = (1 << 2),
    bodyClass           = (1 << 3),
    translationClass    = (1 << 4),
    rotationClass       = (1 << 5),
    massClass           = (1 << 6),
    geometryClass       = (1 << 7),
    appearanceClass     = (1 << 8),
    jointClass          = (1 << 9),
    axisClass           = (1 << 10),
    motorClass          = (1 << 11),
    deflectionClass     = (1 << 12),
    solverClass         = (1 << 13),
    surfaceClass        = (1 << 14),
    primitiveGroupClass = (1 << 15),
    verticesClass       = (1 << 16),
    normalsClass        = (1 << 17),
    texCoordsClass      = (1 << 18),
    intSensorClass      = (1 << 19),
    extSensorClass      = (1 << 20),
    materialClass       = (1 << 21),
    frictionClass       = (1 << 22),
    lightClass          = (1 << 23),
    userInputClass      = (1 << 24),
  };

  // element handlers
  Element* includeElement();
  Element* simulationElement();
  Element* sceneElement();
  Element* setElement();
  Element* compoundElement();
  Element* bodyElement();
  Element* translationElement();
  Element* rotationElement();
  Element* massElement();
  Element* boxMassElement();
  Element* sphereMassElement();
  Element* inertiaMatrixMassElement();
  Element* geometryElement();
  Element* boxGeometryElement();
  Element* sphereGeometryElement();
  Element* cylinderGeometryElement();
  Element* capsuleGeometryElement();
  Element* materialElement();
  Element* frictionElement();
  Element* rollingFrictionElement();
  Element* appearanceElement();
  Element* boxAppearanceElement();
  Element* sphereAppearanceElement();
  Element* cylinderAppearanceElement();
  Element* capsuleAppearanceElement();
  Element* complexAppearanceElement();
  Element* trianglesElement();
  Element* quadsElement();
  void trianglesAndQuadsText(std::string& text, Location location);
  Element* verticesElement();
  void verticesText(std::string& text, Location location);
  Element* normalsElement();
  void normalsText(std::string& text, Location location);
  Element* texCoordsElement();
  void texCoordsText(std::string& text, Location location);
  Element* hingeElement();
  Element* sliderElement();
  Element* axisElement();
  Element* deflectionElement();
  Element* PT2MotorElement();
  Element* servoMotorElement();
  Element* velocityMotorElement();
  Element* quickSolverElement();
  Element* lightElement();
  Element* surfaceElement();
  Element* gyroscopeElement();
  Element* accelerometerElement();
  Element* cameraElement();
  Element* collisionSensorElement();
  Element* objectSegmentedImageSensorElement();
  Element* singleDistanceSensorElement();
  Element* approxDistanceSensorElement();
  Element* depthImageSensorElement();
  Element* userInputElement();
};
