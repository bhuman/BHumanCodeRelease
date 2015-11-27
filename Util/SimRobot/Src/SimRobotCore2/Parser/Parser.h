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
* A libxml2 parser for .ros2 files.
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
  * @return Wheter the file was parses without errors or not
  */
  bool parse(const std::string& fileName, std::list<std::string>& errors);

private:
  typedef Element* (Parser::*StartElementProc)();
  typedef void (Parser::*TextProc)(std::string& text);

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

    ElementData(ElementData* parent, const ElementInfo* info = 0) : parent(parent), info(info), parsedChildren(0), parsedAttributes(0), usedPlaceholdersInAttributes(false) {}
  };

  class MacroElement
  {
  public:
    MacroElement* parent;
    const ElementInfo* elementInfo;
    Attributes attributes;
    std::string text;
    std::list<MacroElement*> children;
    Element* element; /**< An element that was created from the macro */
    int line;
    int column;
    int endLine;
    int endColumn;

    MacroElement(MacroElement* parent, const ElementInfo* elementInfo,
      Attributes& attributes,
      int line, int column) :
      parent(parent), elementInfo(elementInfo), element(0), line(line), column(column), endLine(0), endColumn(0)
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

    Macro(const ElementInfo* elementInfo, const std::string& fileName, Attributes& attributes, int line, int column) :
      MacroElement(0, elementInfo, attributes, line, column), fileName(fileName), replaying(false) {}
  };

  std::unordered_map<std::string, const ElementInfo*> elementInfos; /** Mapping element name strings to handler member function pointers */

  std::list<std::string>* errors; /**< List of errors occured while parsing */
  std::string parseRootDir; /**< The directory in which the main .ros2 file is stored. */
  std::string includeFile; /**< A file to be included */

  std::unordered_map<std::string, Macro*> macros; /**< A storage for macros */
  Macro* sceneMacro; /**< A macro created for the <Scene> element */

  MacroElement* recordingMacroElement; /**< A macro element set to record subordinate nodes of a macro */
  MacroElement* replayingMacroElement; /** A macro element set to insert subordinate nodes of a macro */
  Element* element; /**< The last inserted xml element */
  ElementData* elementData; /** Element context data required for parsing a xml element */
  const Attributes* attributes;  /**< The current set of attributes */

  bool passedSimulationTag; /**< Whether the <Simulation>-tag was passed or not */

  void parseSimulation();
  void parseMacroElements();
  void parseMacroElement();

  /**
  * A handler called for each node from the input file. Call readSubNodes() within in the handler to parse subordinate nodes.
  * @param node The name of the node
  * @param attributes The attributes of the node
  */
  virtual void handleNode(const std::string& name, Attributes& attributes);

  /**
  * A handler for nodes inserted from a macro. Call readSubNodes() within in the handler to parse subordinate nodes.
  * @param elementInfo Information about the element class to parse
  * @param attributes The attributes of the element
  */
  void handleNodeFromMacro(const ElementInfo* elementInfo, Attributes& attributes);

  /**
  * A function called to handle "normal" elements within in the <Scene></Scene> section.
  * @param readOnlyAttributes Whether it is allowed to modify the set of attributes (\c attributes) or not.
  */
  void handleSceneElement(bool readOnlyAttributes);

  /**
  * A handler called when text was parsed
  * @param text The text read
  */
  virtual void handleText(std::string& text);

  /**
  * Adds an error message to the list of error messages.
  * @param message The error message
  */
  virtual void handleError(const std::string& message);

  /** Checks if there are any unexpected attributes in the current set of attributes. */
  void checkAttributes();

  /** Checks if some required subordinate elements have not been parsed */
  void checkElements();

  const std::string* resolvePlaceholder(const std::string& name);
  const std::string& replacePlaceholders(const std::string& str);
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
  bool getFloatAndUnit(const char* key, bool required, float defaultValue, float& value, char** unit);
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
  void trianglesAndQuadsText(std::string& text);
  Element* verticesElement();
  void verticesText(std::string& text);
  Element* normalsElement();
  void normalsText(std::string& text);
  Element* texCoordsElement();
  void texCoordsText(std::string& text);
  Element* hingeElement();
  Element* sliderElement();
  Element* axisElement();
  Element* deflectionElement();
  Element* servoMotorElement();
  Element* velocityMotorElement();
  Element* quickSolverElement();
  Element* lightElement();
  Element* surfaceElement();
  Element* gyroscopeElement();
  Element* accelerometerElement();
  Element* cameraElement();
  Element* collisionSensor2Element();
  Element* singleDistanceSensorElement();
  Element* approxDistanceSensorElement();
  Element* depthImageSensorElement();
  Element* userInputElement();
};
