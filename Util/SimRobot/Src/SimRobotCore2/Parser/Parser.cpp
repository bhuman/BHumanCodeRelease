/**
* @file Simulation/Parser.cpp
* Implementation of class Parser
* @author Colin Graf
*/

#include <clocale>
#include "Platform/OpenGL.h"

#include "Parser/Parser.h"
#include "Platform/Assert.h"
#include "Simulation/Simulation.h"
#include "Simulation/Body.h"
#include "Simulation/Scene.h"
#include "Simulation/Compound.h"
#include "Simulation/UserInput.h"
#include "Simulation/Actuators/Hinge.h"
#include "Simulation/Actuators/Slider.h"
#include "Simulation/Axis.h"
#include "Simulation/Masses/BoxMass.h"
#include "Simulation/Masses/SphereMass.h"
#include "Simulation/Masses/InertiaMatrixMass.h"
#include "Simulation/Geometries/BoxGeometry.h"
#include "Simulation/Geometries/SphereGeometry.h"
#include "Simulation/Geometries/CylinderGeometry.h"
#include "Simulation/Geometries/CapsuleGeometry.h"
#include "Simulation/Motors/ServoMotor.h"
#include "Simulation/Motors/VelocityMotor.h"
#include "Simulation/Appearances/BoxAppearance.h"
#include "Simulation/Appearances/SphereAppearance.h"
#include "Simulation/Appearances/CylinderAppearance.h"
#include "Simulation/Appearances/CapsuleAppearance.h"
#include "Simulation/Appearances/ComplexAppearance.h"
#include "Simulation/Sensors/Gyroscope.h"
#include "Simulation/Sensors/Accelerometer.h"
#include "Simulation/Sensors/Camera.h"
#include "Simulation/Sensors/CollisionSensor.h"
#include "Simulation/Sensors/SingleDistanceSensor.h"
#include "Simulation/Sensors/ApproxDistanceSensor.h"
#include "Simulation/Sensors/DepthImageSensor.h"

Parser::Parser() : errors(0), sceneMacro(0), recordingMacroElement(0), replayingMacroElement(0), element(0), elementData(0), passedSimulationTag(false)
{
  static const ElementInfo elements[] = {
    // { element, class, handler, text handler, flaga
    //   required children, optional children, repeatable children }
    {"Include", infrastructureClass, &Parser::includeElement, 0, 0,
      0, 0, 0},
    {"Simulation", infrastructureClass, &Parser::simulationElement, 0, 0,
      sceneClass, 0, 0},

    {"Set", setClass, &Parser::setElement, 0, 0,
      0, 0, 0},

    {"Scene", sceneClass, &Parser::sceneElement,0, 0,
      0, solverClass, setClass | bodyClass | compoundClass | lightClass | userInputClass},
    {"QuickSolver", solverClass, &Parser::quickSolverElement, 0, 0,
      0, 0, 0},
    {"Light", lightClass, &Parser::lightElement, 0, 0,
      0, 0, 0},

    {"Compound", compoundClass, &Parser::compoundElement, 0, 0,
      0, translationClass | rotationClass, setClass | jointClass | compoundClass | bodyClass | appearanceClass | geometryClass | extSensorClass | userInputClass},
    {"Body", bodyClass, &Parser::bodyElement, 0, 0,
      massClass, translationClass | rotationClass, setClass | jointClass | appearanceClass | geometryClass | massClass | intSensorClass | extSensorClass | userInputClass},

    {"Translation", translationClass, &Parser::translationElement, 0, 0,
      0, 0, 0},
    {"Rotation", rotationClass, &Parser::rotationElement, 0, 0,
      0, 0, 0},

    {"Mass", massClass, &Parser::massElement, 0, constantFlag,
      0, translationClass | rotationClass, setClass | massClass},
    {"BoxMass", massClass, &Parser::boxMassElement, 0, constantFlag,
      0, translationClass | rotationClass, setClass | massClass},
    {"SphereMass", massClass, &Parser::sphereMassElement, 0, constantFlag,
      0, translationClass | rotationClass, setClass | massClass},
    {"InertiaMatrixMass", massClass, &Parser::inertiaMatrixMassElement, 0, constantFlag,
      0, translationClass | rotationClass, setClass | massClass},

    {"Geometry", geometryClass, &Parser::geometryElement, 0, 0,
      0, translationClass | rotationClass | materialClass, setClass | geometryClass},
    {"BoxGeometry", geometryClass, &Parser::boxGeometryElement, 0, 0,
      0, translationClass | rotationClass | materialClass, setClass | geometryClass},
    {"SphereGeometry", geometryClass, &Parser::sphereGeometryElement, 0, 0,
      0, translationClass | rotationClass | materialClass, setClass | geometryClass},
    {"CylinderGeometry", geometryClass, &Parser::cylinderGeometryElement, 0, 0,
      0, translationClass | rotationClass | materialClass, setClass | geometryClass},
    {"CapsuleGeometry", geometryClass, &Parser::capsuleGeometryElement, 0, 0,
      0, translationClass | rotationClass | materialClass, setClass | geometryClass},

    {"Material", materialClass, &Parser::materialElement, 0, constantFlag,
      0, 0, setClass | frictionClass},
    {"Friction", frictionClass, &Parser::frictionElement, 0, constantFlag,
      0, 0, 0},
    {"RollingFriction", frictionClass, &Parser::rollingFrictionElement, 0, constantFlag,
      0, 0, 0},

    {"Appearance", appearanceClass, &Parser::appearanceElement, 0, constantFlag,
      0, translationClass | rotationClass, setClass | appearanceClass},
    {"BoxAppearance", appearanceClass, &Parser::boxAppearanceElement, 0, constantFlag,
      surfaceClass, translationClass | rotationClass, setClass | appearanceClass},
    {"SphereAppearance", appearanceClass, &Parser::sphereAppearanceElement, 0, constantFlag,
      surfaceClass, translationClass | rotationClass, setClass | appearanceClass},
    {"CylinderAppearance", appearanceClass, &Parser::cylinderAppearanceElement, 0, constantFlag,
      surfaceClass, translationClass | rotationClass, setClass | appearanceClass},
    {"CapsuleAppearance", appearanceClass, &Parser::capsuleAppearanceElement, 0, constantFlag,
      surfaceClass, translationClass | rotationClass, setClass | appearanceClass},
    {"ComplexAppearance", appearanceClass, &Parser::complexAppearanceElement, 0, constantFlag,
      surfaceClass | verticesClass | primitiveGroupClass, translationClass | rotationClass | normalsClass | texCoordsClass, setClass | primitiveGroupClass | appearanceClass},

    {"Vertices", verticesClass, &Parser::verticesElement, &Parser::verticesText, textFlag | constantFlag,
      0, 0, 0},
    {"Normals", normalsClass, &Parser::normalsElement, &Parser::normalsText, textFlag | constantFlag,
      0, 0, 0},
    {"TexCoords", texCoordsClass, &Parser::texCoordsElement, &Parser::texCoordsText, textFlag | constantFlag,
      0, 0, 0},
    {"Triangles", primitiveGroupClass, &Parser::trianglesElement, &Parser::trianglesAndQuadsText, textFlag | constantFlag,
      0, 0, 0},
    {"Quads", primitiveGroupClass, &Parser::quadsElement, &Parser::trianglesAndQuadsText, textFlag | constantFlag,
      0, 0, 0},

    {"Surface", surfaceClass, &Parser::surfaceElement, 0, constantFlag,
      0, 0, 0},

    {"Hinge", jointClass, &Parser::hingeElement, 0, 0,
      bodyClass | axisClass, translationClass | rotationClass, setClass},
    {"Slider", jointClass, &Parser::sliderElement, 0, 0,
      bodyClass | axisClass, translationClass | rotationClass, setClass},
    {"Axis", axisClass, &Parser::axisElement, 0, 0,
      0, motorClass | deflectionClass, setClass},
    {"Deflection", deflectionClass, &Parser::deflectionElement, 0, 0,
      0, 0, 0},
    {"ServoMotor", motorClass, &Parser::servoMotorElement, 0, 0,
      0, 0, 0},
    {"VelocityMotor", motorClass, &Parser::velocityMotorElement, 0, 0,
      0, 0, 0},

    {"Gyroscope", intSensorClass, &Parser::gyroscopeElement, 0, 0,
      0, 0, 0},
    {"Accelerometer", intSensorClass, &Parser::accelerometerElement, 0, 0,
      0, 0, 0},
    {"Camera", extSensorClass, &Parser::cameraElement, 0, 0,
      0, translationClass | rotationClass, 0},
    {"CollisionSensor", intSensorClass, &Parser::collisionSensor2Element, 0, 0,
      0, translationClass | rotationClass, geometryClass},
    {"SingleDistanceSensor", extSensorClass, &Parser::singleDistanceSensorElement, 0, 0,
      0, translationClass | rotationClass, 0},
    {"ApproxDistanceSensor", extSensorClass, &Parser::approxDistanceSensorElement, 0, 0,
      0, translationClass | rotationClass, 0},
    {"DepthImageSensor", extSensorClass, &Parser::depthImageSensorElement, 0, 0,
      0, translationClass | rotationClass, 0},

    {"UserInput", userInputClass, &Parser::userInputElement, 0, 0,
      0, 0, 0},
  };

  // prepare proc info map
  for(unsigned int i = 0; i < sizeof(elements) / sizeof(*elements); ++i)
    elementInfos[elements[i].name] = &elements[i];
}

Element* Parser::simulationElement()
{
  passedSimulationTag = true;
  return 0;
}

Element* Parser::includeElement()
{
  includeFile = getString("href", true);
  return 0;
}

Element* Parser::setElement()
{
  ASSERT(element);
  const std::string& name = getString("name", true);
  const std::string& value = getString("value", true);
  std::unordered_map<std::string, std::string>& vars = elementData->parent->vars;
  if(vars.find(name) == vars.end())
    vars[name] = value;
  return 0;
}

Element* Parser::sceneElement()
{
  Scene* scene = new Scene();
  scene->name = getString("name", false);
  scene->controller = getString("controller", false);
  getColor("color", false, scene->color);
  scene->stepLength = getTimeNonZeroPositive("stepLength", false, 0.01f);
  scene->gravity = getAcceleration("gravity", false, -9.80665f);
  scene->cfm = getFloatMinMax("CFM", false, -1.f, 0.f, 1.f);
  scene->erp = getFloatMinMax("ERP", false, -1.f, 0.f, 1.f);
  scene->contactSoftERP = getFloatMinMax("contactSoftERP", false, -1.f, 0.f, 1.f);
  if(scene->contactSoftERP != -1.f)
    scene->contactMode |= dContactSoftERP;
  scene->contactSoftCFM = getFloatMinMax("contactSoftCFM", false, -1.f, 0.f, 1.f);
  if(scene->contactSoftCFM != -1.f)
    scene->contactMode |= dContactSoftCFM;
  scene->detectBodyCollisions = getBool("bodyCollisions", false, true);

  ASSERT(!Simulation::simulation->scene);
  Simulation::simulation->scene = scene;
  return scene;
}

Element* Parser::quickSolverElement()
{
  Scene* scene = dynamic_cast<Scene*>(element);
  ASSERT(scene);
  scene->useQuickSolver = true;
  scene->quickSolverIterations = getIntegerNonZeroPositive("iterations", false, -1);
  scene->quickSolverSkip = getIntegerNonZeroPositive("skip", false, 1);
  return 0;
}

Element* Parser::lightElement()
{
  Scene::Light* light = new Scene::Light();
  getColor("diffuseColor", false, light->diffuseColor);
  getColor("ambientColor", false, light->ambientColor);
  getColor("specularColor", false, light->specularColor);
  light->position[0] = getLength("x", false, light->position[0]);
  light->position[1] = getLength("y", false, light->position[1]);
  light->position[2] = getLength("z", false, light->position[2]);
  light->constantAttenuation = getFloatPositive("constantAttenuation", false, light->constantAttenuation);
  light->linearAttenuation = getFloatPositive("linearAttenuation", false, light->linearAttenuation);
  light->quadraticAttenuation = getFloatPositive("quadraticAttenuation", false, light->quadraticAttenuation);
  light->spotCutoff = getAngle("spotCutoff", false, light->spotCutoff);
  light->spotDirection.x = getLength("spotDirectionX", false, light->spotDirection.x);
  light->spotDirection.y = getLength("spotDirectionY", false, light->spotDirection.y);
  light->spotDirection.z = getLength("spotDirectionZ", false, light->spotDirection.z);
  light->spotExponent = getFloatMinMax("spotExponent", false, light->spotExponent, 0.f, 128.f);
  return light;
}

Element* Parser::bodyElement()
{
  Body* body = new Body();
  body->name = getString("name", false);
  return body;
}

Element* Parser::compoundElement()
{
  Compound* compound = new Compound();
  compound->name = getString("name", false);
  return compound;
}

Element* Parser::hingeElement()
{
  Hinge* hinge = new Hinge();
  hinge->name = getString("name", false);
  return hinge;
}

Element* Parser::sliderElement()
{
  Slider* slider = new Slider();
  slider->name = getString("name", false);
  return slider;
}

Element* Parser::massElement()
{
  Mass* mass = new Mass();
  mass->name = getString("name", false);
  return mass;
}

Element* Parser::boxMassElement()
{
  BoxMass* boxMass = new BoxMass();
  boxMass->name = getString("name", false);
  boxMass->value = getMass("value", true, 0.f);
  boxMass->width = getLength("width", true, 0.f);
  boxMass->height = getLength("height", true, 0.f);
  boxMass->depth = getLength("depth", true, 0.f);
  return boxMass;
}

Element* Parser::sphereMassElement()
{
  SphereMass* sphereMass = new SphereMass();
  sphereMass->name = getString("name", false);
  sphereMass->value = getMass("value", true, 0.f);
  sphereMass->radius = getLength("radius", true, 0.f);
  return sphereMass;
}

Element* Parser::inertiaMatrixMassElement()
{
  InertiaMatrixMass* inertiaMatrixMass = new InertiaMatrixMass();
  inertiaMatrixMass->name = getString("name", false);
  inertiaMatrixMass->value = getMass("value", true, 0.f);
  inertiaMatrixMass->x = getLength("x", false, 0.f);
  inertiaMatrixMass->y = getLength("y", false, 0.f);
  inertiaMatrixMass->z = getLength("z", false, 0.f);
  inertiaMatrixMass->ixx = getMassLengthLength("ixx", true, 0.f);
  inertiaMatrixMass->ixy = getMassLengthLength("ixy", false, 0.f);
  inertiaMatrixMass->ixz = getMassLengthLength("ixz", false, 0.f);
  inertiaMatrixMass->iyy = getMassLengthLength("iyy", true, 0.f);
  inertiaMatrixMass->iyz = getMassLengthLength("iyz", false, 0.f);
  inertiaMatrixMass->izz = getMassLengthLength("izz", true, 0.f);
  return inertiaMatrixMass;
}

Element* Parser::geometryElement()
{
  Geometry* geometry = new Geometry();
  geometry->name = getString("name", false);
  return geometry;
}

Element* Parser::boxGeometryElement()
{
  BoxGeometry* boxGeometry = new BoxGeometry();
  getColor("color", false, boxGeometry->color);
  boxGeometry->name = getString("name", false);
  boxGeometry->width = getLength("width", true, 0.f);
  boxGeometry->height = getLength("height", true, 0.f);
  boxGeometry->depth = getLength("depth", true, 0.f);
  return boxGeometry;
}

Element* Parser::sphereGeometryElement()
{
  SphereGeometry* sphereGeometry = new SphereGeometry();
  getColor("color", false, sphereGeometry->color);
  sphereGeometry->name = getString("name", false);
  sphereGeometry->radius = getLength("radius", true, 0.f);
  return sphereGeometry;
}

Element* Parser::cylinderGeometryElement()
{
  CylinderGeometry* cylinderGeometry = new CylinderGeometry();
  getColor("color", false, cylinderGeometry->color);
  cylinderGeometry->name = getString("name", false);
  cylinderGeometry->radius = getLength("radius", true, 0.f);
  cylinderGeometry->height = getLength("height", true, 0.f);
  return cylinderGeometry;
}

Element* Parser::capsuleGeometryElement()
{
  CapsuleGeometry* capsuleGeometry = new CapsuleGeometry();
  getColor("color", false, capsuleGeometry->color);
  capsuleGeometry->name = getString("name", false);
  capsuleGeometry->radius = getLength("radius", true, 0.f);
  capsuleGeometry->height = getLength("height", true, 0.f);
  return capsuleGeometry;
}

Element* Parser::materialElement()
{
  Geometry::Material* material = new Geometry::Material();
  material->name = getString("name", false);
  return material;
}

Element* Parser::frictionElement()
{
  Geometry::Material* material = dynamic_cast<Geometry::Material*>(element);
  ASSERT(material);
  const std::string& otherMaterial = getString("material", true);
  float friction = getFloatPositive("value", true, 1.f);
  material->frictions[otherMaterial] = friction;
  return 0;
}

Element* Parser::rollingFrictionElement()
{
  Geometry::Material* material = dynamic_cast<Geometry::Material*>(element);
  ASSERT(material);
  const std::string& otherMaterial = getString("material", true);
  float rollingFriction = getFloatPositive("value", true, 1.f);
  material->rollingFrictions[otherMaterial] = rollingFriction;
  return 0;
}

Element* Parser::appearanceElement()
{
  Appearance* appearance = new Appearance();
  appearance->name = getString("name", false);
  return appearance;
}

Element* Parser::boxAppearanceElement()
{
  BoxAppearance* boxAppearance = new BoxAppearance();
  boxAppearance->name = getString("name", false);
  boxAppearance->width = getLength("width", true, 0.f);
  boxAppearance->height = getLength("height", true, 0.f);
  boxAppearance->depth = getLength("depth", true, 0.f);
  return boxAppearance;
}

Element* Parser::sphereAppearanceElement()
{
  SphereAppearance* sphereAppearance = new SphereAppearance();
  sphereAppearance->name = getString("name", false);
  sphereAppearance->radius = getLength("radius", true, 0.f);
  return sphereAppearance;
}

Element* Parser::cylinderAppearanceElement()
{
  CylinderAppearance* cylinderAppearance = new CylinderAppearance();
  cylinderAppearance->name = getString("name", false);
  cylinderAppearance->height = getLength("height", true, 0.f);
  cylinderAppearance->radius = getLength("radius", true, 0.f);
  return cylinderAppearance;
}

Element* Parser::capsuleAppearanceElement()
{
  CapsuleAppearance* capsuleAppearance = new CapsuleAppearance();
  capsuleAppearance->name = getString("name", false);
  capsuleAppearance->height = getLength("height", true, 0.f);
  capsuleAppearance->radius = getLength("radius", true, 0.f);
  return capsuleAppearance;
}

Element* Parser::complexAppearanceElement()
{
  ComplexAppearance* complexAppearance = new ComplexAppearance();
  complexAppearance->name = getString("name", false);
  return complexAppearance;
}

Element* Parser::trianglesElement()
{
  return  new ComplexAppearance::PrimitiveGroup(GL_TRIANGLES);
}

void Parser::trianglesAndQuadsText(std::string& text)
{
  ComplexAppearance::PrimitiveGroup* primitiveGroup = dynamic_cast<ComplexAppearance::PrimitiveGroup*>(element);
  ASSERT(primitiveGroup);
  std::list<unsigned int>& vs = primitiveGroup->vertices;
  char* str = (char*)text.c_str();
  unsigned int l;
  while(isspace(*str))
    ++str;
  while(*str)
  {
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    l = (unsigned int) strtol(str, &str, 10);
    while(isspace(*str))
      ++str;
    vs.push_back(l);
  }
breakTwice: ;
}

Element* Parser::quadsElement()
{
  return new ComplexAppearance::PrimitiveGroup(GL_QUADS);
}

Element* Parser::verticesElement()
{
  ComplexAppearance::Vertices* vertices = new ComplexAppearance::Vertices();
  vertices->unit = getUnit("unit", false, 1);
  return vertices;
}

void Parser::verticesText(std::string& text)
{
  ComplexAppearance::Vertices* vertices = dynamic_cast<ComplexAppearance::Vertices*>(element);
  ASSERT(vertices);
  std::vector<ComplexAppearance::Vertex>& vs = vertices->vertices;
  char* str = (char*)text.c_str();
  double x, y, z;
  while(isspace(*str))
    ++str;
  while(*str)
  {
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    x = strtod(str, &str);
    while(isspace(*str))
      ++str;
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    y = strtod(str, &str);
    while(isspace(*str))
      ++str;
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    z = strtod(str, &str);
    while(isspace(*str))
      ++str;
    x *= vertices->unit;
    y *= vertices->unit;
    z *= vertices->unit;
    vs.push_back(ComplexAppearance::Vertex((float) x, (float) y, (float) z));
  }
breakTwice: ;
}

Element* Parser::normalsElement()
{
  return new ComplexAppearance::Normals();
}

void Parser::normalsText(std::string& text)
{
  ComplexAppearance::Normals* normals = dynamic_cast<ComplexAppearance::Normals*>(element);
  ASSERT(normals);
  std::vector<ComplexAppearance::Normal>& ns = normals->normals;
  char* str = (char*)text.c_str();
  double x, y, z;
  while(isspace(*str))
    ++str;
  while(*str)
  {
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    x = strtod(str, &str);
    while(isspace(*str))
      ++str;
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    y = strtod(str, &str);
    while(isspace(*str))
      ++str;
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    z = strtod(str, &str);
    while(isspace(*str))
      ++str;
    ns.push_back(ComplexAppearance::Normal((float) x, (float) y, (float) z, 1));
  }
breakTwice: ;
}

Element* Parser::texCoordsElement()
{
  return new ComplexAppearance::TexCoords();
}

void Parser::texCoordsText(std::string& text)
{
  ComplexAppearance::TexCoords* texCoords = dynamic_cast<ComplexAppearance::TexCoords*>(element);
  ASSERT(texCoords);
  std::vector<ComplexAppearance::TexCoord>& ts = texCoords->coords;
  char* str = (char*)text.c_str();
  double x, y;
  while(isspace(*str))
    ++str;
  while(*str)
  {
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    x = strtod(str, &str);
    while(isspace(*str))
      ++str;
    while(*str == '#') { while(*str && *str != '\n' && *str != '\r') ++str;  while(isspace(*str)) ++str; if(!*str) goto breakTwice; }
    y = strtod(str, &str);
    while(isspace(*str))
      ++str;
    ts.push_back(ComplexAppearance::TexCoord((float) x, (float) y));
  }
breakTwice: ;
}

Element* Parser::translationElement()
{
  Vector3<>* translation = new Vector3<>(getLength("x", false, 0.f), getLength("y", false, 0.f), getLength("z", false, 0.f));

  SimObject* simObject = dynamic_cast<SimObject*>(element);
  if(simObject)
  {
    ASSERT(!simObject->translation);
    simObject->translation = translation;
  }
  else
  {
    Mass* mass = dynamic_cast<Mass*>(element);
    ASSERT(mass);
    ASSERT(!mass->translation);
    mass->translation = translation;
  }
  return 0;
}

Element* Parser::rotationElement()
{
  Matrix3x3<>* rotation = new Matrix3x3<>(Vector3<>(0.f, 0.f, getAngle("z", false, 0.f)));
  *rotation *= Matrix3x3<>(Vector3<>(0.f, getAngle("y", false, 0.f), 0.f));
  *rotation *= Matrix3x3<>(Vector3<>(getAngle("x", false, 0.f), 0.f, 0.f));

  SimObject* simObject = dynamic_cast<SimObject*>(element);
  if(simObject)
  {
    ASSERT(!simObject->rotation);
    simObject->rotation = rotation;
  }
  else
  {
    Mass* mass = dynamic_cast<Mass*>(element);
    ASSERT(mass);
    ASSERT(!mass->rotation);
    mass->rotation = rotation;
  }
  return 0;
}

Element* Parser::axisElement()
{
  Axis* axis = new Axis();
  axis->x = getFloat("x", false, 0.f);
  axis->y = getFloat("y", false, 0.f);
  axis->z = getFloat("z", false, 0.f);
  axis->cfm = getFloatMinMax("cfm", false, -1.f, 0.f, 1.f);
  Joint* joint = dynamic_cast<Joint*>(element);
  ASSERT(joint);
  axis->joint = joint;
  return axis;
}

Element* Parser::deflectionElement()
{
  Axis::Deflection* deflection = new Axis::Deflection();
  Axis* axis = dynamic_cast<Axis*>(element);
  ASSERT(axis);
  ASSERT(axis->joint);

  if(dynamic_cast<Hinge*>(axis->joint))
  {
    deflection->min = getAngle("min", true, 0.f);
    deflection->max = getAngle("max", true, 0.f);
    deflection->offset = getAngle("init", false, 0.f);
  }
  else if(dynamic_cast<Slider*>(axis->joint))
  {
    deflection->min = getLength("min", true, 0.f);
    deflection->max = getLength("max", true, 0.f);
  }
  else
    ASSERT(false);

  deflection->stopCFM = getFloatMinMax("stopCFM", false, -1.f, 0.f, 1.f);
  deflection->stopERP = getFloatMinMax("stopERP", false, -1.f, 0.f, 1.f);

  ASSERT(!axis->deflection);
  axis->deflection = deflection;
  return 0;
}

Element* Parser::servoMotorElement()
{
  ServoMotor* servoMotor = new ServoMotor();
  Axis* axis = dynamic_cast<Axis*>(element);
  ASSERT(axis);
  ASSERT(!axis->motor);

  if(dynamic_cast<Hinge*>(axis->joint))
    servoMotor->maxVelocity = getAngularVelocity("maxVelocity", true, 0.f);
  else if(dynamic_cast<Slider*>(axis->joint))
    servoMotor->maxVelocity = getVelocity("maxVelocity", true, 0.f);
  else
    ASSERT(false);

  servoMotor->maxForce = getForce("maxForce", true, 0.f);
  servoMotor->controller.p = getFloat("p", true, 0.f);
  servoMotor->controller.i = getFloat("i", false, 0.f);
  servoMotor->controller.d = getFloat("d", false, 0.f);

  axis->motor = servoMotor;
  return 0;
}

Element* Parser::velocityMotorElement()
{
  VelocityMotor* velocityMotor = new VelocityMotor();
  Axis* axis = dynamic_cast<Axis*>(element);
  ASSERT(axis);
  ASSERT(!axis->motor);

  if(dynamic_cast<Hinge*>(axis->joint))
    velocityMotor->maxVelocity = getAngularVelocity("maxVelocity", true, 0.f);
  else if(dynamic_cast<Slider*>(axis->joint))
    velocityMotor->maxVelocity = getVelocity("maxVelocity", true, 0.f);
  else
    ASSERT(false);

  velocityMotor->maxForce = getForce("maxForce", true, 0.f);

  axis->motor = velocityMotor;
  return 0;
}

Element* Parser::surfaceElement()
{
  Appearance::Surface* surface = new Appearance::Surface();
  getColor("diffuseColor", true, surface->diffuseColor);
  surface->hasAmbientColor = getColor("ambientColor", false, surface->ambientColor);
  getColor("specularColor", false, surface->specularColor);
  getColor("emissionColor", false, surface->emissionColor);
  surface->shininess = getFloatMinMax("shininess", false, surface->shininess, 0.f, 128.f);
  surface->diffuseTexture = getString("diffuseTexture", false);
  return surface;
}

Element* Parser::gyroscopeElement()
{
  Gyroscope* gyroscope = new Gyroscope();
  gyroscope->name = getString("name", false);
  return gyroscope;
}

Element* Parser::accelerometerElement()
{
  Accelerometer* accelerometer = new Accelerometer();
  accelerometer->name = getString("name", false);
  return accelerometer;
}

Element* Parser::cameraElement()
{
  Camera* camera = new Camera();
  camera->name = getString("name", false);
  camera->imageWidth = getIntegerNonZeroPositive("imageWidth", true, 0);
  camera->imageHeight = getIntegerNonZeroPositive("imageHeight", true, 0);
  camera->angleX = getAngleNonZeroPositive("angleX", true, 0.f);
  camera->angleY = getAngleNonZeroPositive("angleY", true, 0.f);
  return camera;
}

Element* Parser::collisionSensor2Element()
{
  CollisionSensor* collisionSensor = new CollisionSensor();
  collisionSensor->name = getString("name", false);
  return collisionSensor;
}

Element* Parser::singleDistanceSensorElement()
{
  SingleDistanceSensor* singleDistanceSensor = new SingleDistanceSensor();
  singleDistanceSensor->name = getString("name", false);
  singleDistanceSensor->min = getLength("min", false, 0.f);
  singleDistanceSensor->max = getLength("max", false, 999999.f);
  return singleDistanceSensor;
}

Element* Parser::approxDistanceSensorElement()
{
  ApproxDistanceSensor* approxDistanceSensor = new ApproxDistanceSensor();
  approxDistanceSensor->name = getString("name", false);
  approxDistanceSensor->min = getLength("min", false, 0.f);
  approxDistanceSensor->max = getLength("max", false, 999999.f);
  approxDistanceSensor->angleX = getAngleNonZeroPositive("angleX", true, 0.f);
  approxDistanceSensor->angleY = getAngleNonZeroPositive("angleY", true, 0.f);
  return approxDistanceSensor;
}

Element* Parser::depthImageSensorElement()
{
  DepthImageSensor* depthImageSensor = new DepthImageSensor();
  depthImageSensor->name = getString("name", false);
  depthImageSensor->imageWidth = getIntegerNonZeroPositive("imageWidth", true, 0);
  depthImageSensor->imageHeight = getIntegerNonZeroPositive("imageHeight", false, 1);
  depthImageSensor->angleX = getAngleNonZeroPositive("angleX", true, 0.f);
  depthImageSensor->angleY = getAngleNonZeroPositive("angleY", true, 0.f);
  depthImageSensor->min = getLength("min", false, 0.f);
  depthImageSensor->max = getLength("max", false, 999999.f);

  const std::string& projection = getString("projection", false);
  if(projection == "" || projection == "perspective")
    depthImageSensor->projection = DepthImageSensor::perspectiveProjection;
  else if(projection == "spherical")
  {
    if(depthImageSensor->imageHeight > 1)
      handleError("Spherical projection is currently only supported for 1-D sensors (i.e. with imageHeight=\"1\")");
    else
      depthImageSensor->projection = DepthImageSensor::sphericalProjection;
  }
  else
    handleError("Unexpected projection type \"" + projection + "\" (expected one of \"perspective, spherical\")");

  return depthImageSensor;
}

Element* Parser::userInputElement()
{
  UserInput* userInput = new UserInput();

  userInput->name = getString("name", false);
  std::string type = getString("type", false);
  if(type == "angle")
  {
    userInput->inputPort.unit = QString::fromUtf8("°");
    userInput->inputPort.min = getAngle("min", true, 0.f);
    userInput->inputPort.max = getAngle("max", true, 0.f);
    userInput->inputPort.defaultValue = getAngle("default", false, 0.f);
  }
  else if(type == "angularVelocity")
  {
    userInput->inputPort.unit = QString::fromUtf8("°/s");
    userInput->inputPort.min = getAngularVelocity("min", true, 0.f);
    userInput->inputPort.max = getAngularVelocity("max", true, 0.f);
    userInput->inputPort.defaultValue = getAngularVelocity("default", false, 0.f);
  }
  else if(type == "length" || type == "")
  {
    userInput->inputPort.unit = QString::fromUtf8("m");
    userInput->inputPort.min = getLength("min", true, 0.f);
    userInput->inputPort.max = getLength("max", true, 0.f);
    userInput->inputPort.defaultValue = getLength("default", false, 0.f);
  }
  else if(type == "velocity")
  {
    userInput->inputPort.unit = QString::fromUtf8("m/s");
    userInput->inputPort.min = getVelocity("min", true, 0.f);
    userInput->inputPort.max = getVelocity("max", true, 0.f);
    userInput->inputPort.defaultValue = getVelocity("default", false, 0.f);
  }
  else if(type == "acceleration")
  {
    userInput->inputPort.unit = QString::fromUtf8("m/s^2");
    userInput->inputPort.min = getAcceleration("min", true, 0.f);
    userInput->inputPort.max = getAcceleration("max", true, 0.f);
    userInput->inputPort.defaultValue = getAcceleration("default", false, 0.f);
  }
  else
    handleError("Unexpected user input type \"" + type + "\" (expected one of \"length, velocity, acceleration, angle, angularVelocity\")");

  return userInput;
}

Parser::~Parser()
{
  for(std::unordered_map<std::string, Macro*>::const_iterator iter = macros.begin(), end = macros.end(); iter != end; ++iter)
    delete iter->second;
}

void Parser::parseSimulation()
{
  std::unordered_map<std::string, const ElementInfo*>::const_iterator iter = elementInfos.find("Simulation");
  ASSERT(iter != elementInfos.end());
  const ElementInfo* elementInfo =  iter->second;
  ElementData elementData(0, elementInfo);

  this->elementData = &elementData;
  ASSERT(this->element == 0);

  if(sceneMacro)
  {
    sceneMacro->replaying = true;

    replayingMacroElement = sceneMacro;
    line = replayingMacroElement->line;
    column = replayingMacroElement->column;
    parseMacroElement();
  }

  this->elementData = &elementData;
  checkElements();
}

void Parser::parseMacroElements()
{
  // replay subnodes from a macro element

  MacroElement* parentReplayingMacroElement = replayingMacroElement;

  if(!replayingMacroElement->text.empty())
    (this->*elementData->info->textProc)(replayingMacroElement->text);

  unsigned int parsedChildren = elementData->parsedChildren;
  elementData->parsedChildren = 0;
  for(std::list<MacroElement*>::const_iterator iter = replayingMacroElement->children.begin(), end = replayingMacroElement->children.end(); iter != end; ++iter)
  {
    replayingMacroElement = *iter;

    const ElementInfo* parentInfo = elementData->info;
    const ElementInfo* info = replayingMacroElement->elementInfo;
    if(parsedChildren & info->elementClass && !(parentInfo->repeatableChildren & info->elementClass))
    {
      if(!(elementData->parsedChildren & info->elementClass))
      {
        elementData->parsedChildren |= info->elementClass;
        continue;
      }
    }

    line = replayingMacroElement->line;
    column = replayingMacroElement->column;

    ElementData* parentElementData = elementData;
    Element* parentElement = element;
    parseMacroElement();
    ASSERT(elementData->parent == parentElementData);
    parentElementData->usedPlaceholdersInAttributes |= elementData->usedPlaceholdersInAttributes;
    elementData = parentElementData;
    element = parentElement;
  }
  elementData->parsedChildren |= parsedChildren;

  replayingMacroElement = parentReplayingMacroElement;
  line = replayingMacroElement->endLine;
  column = replayingMacroElement->endColumn;
}

void Parser::parseMacroElement()
{
  ElementData elementData(this->elementData, replayingMacroElement->elementInfo);
  this->elementData = &elementData;
  this->attributes = &replayingMacroElement->attributes;

  // detect unexpected elements
  {
    ElementData* parentElementData = elementData.parent;
    ASSERT(parentElementData);
    const ElementInfo* parentInfo = parentElementData->info;
    const ElementInfo* info = elementData.info;
    if(!((parentInfo->requiredChildren | parentInfo->optionalChildren | parentInfo->repeatableChildren) & info->elementClass) ||
      (parentElementData->parsedChildren & info->elementClass && !(parentInfo->repeatableChildren & info->elementClass)))
    {
      handleError("Unexpected element \"" + std::string(info->name) + "\"");
      return;
    }
    parentElementData->parsedChildren |= info->elementClass;
  }

  // try using an already created instance of the element
  if(replayingMacroElement->element)
  {
    replayingMacroElement->element->addParent(*element);
    return;
  }

  // look for a reference
  const std::string* ref = &getString("ref", false);

  // handle elements without reference
  if(ref->empty())
  {
    Element* parentElement = element;
    Element* childElement = (this->*elementData.info->startElementProc)();
    element = childElement;
    checkAttributes();
    parseMacroElements();
    ASSERT(this->elementData == &elementData);
    ASSERT(element == childElement);
    checkElements();
    if(element)
    {
      if(parentElement)
        element->addParent(*parentElement);

      if(elementData.info->flags & constantFlag && !elementData.usedPlaceholdersInAttributes)
        replayingMacroElement->element = element;
    }
    return;
  }

  // resolve referenced macro
  std::unordered_map<std::string, Macro*>::const_iterator iter = macros.find(*ref + " " + elementData.info->name);
  Macro* const macro = iter == macros.end() ? 0 : iter->second;
  if(!macro || macro->replaying)
  {
    if(macro)
      handleError("Looping reference \"" + *ref + "\"");
    else
      handleError("Unresolvable reference \"" + *ref + "\"");
    return;
  }

  // handle "reference-only" elements (e.g. <BoxAppearance ref="anyBox"></BoxAppearance>)
  const bool isReferenceOnlyElement = attributes->size() == 1 && !replayingMacroElement->hasTextOrChildren();
  if(isReferenceOnlyElement && macro->element )
  { // use an already created "reference-only" instance
    macro->element->addParent(*element);
    replayingMacroElement->element = macro->element;
    return;
  }

  // handle normal macro references
  {
    std::list<Macro*> referencedMacros;
    Attributes* copiedAttributes = 0; // we might need this, since this->attributes is read-only

    // generate list of references macros and a combined attribute set
    macro->replaying = true;
    for(;;)
    {
      referencedMacros.push_back(macro);

      // combine current node attributes and attributes from macro
      for(std::unordered_map<std::string, std::pair<std::string, int> >::const_iterator iter = macro->attributes.begin(), end = macro->attributes.end(); iter != end; ++iter)
        if(attributes->find(iter->first) == attributes->end())
        {
          if(!copiedAttributes)
            attributes = copiedAttributes = new Attributes(*attributes);
          (*copiedAttributes)[iter->first] = std::make_pair(iter->second.first, int(copiedAttributes->size()));
        }

      // find another reference
      std::unordered_map<std::string, std::pair<std::string, int> >::const_iterator refIter = macro->attributes.find("ref");
      if(refIter == macro->attributes.end())
        break;
      ref = &replacePlaceholders(refIter->second.first);

      // resolve next referenced macro
      std::unordered_map<std::string, Macro*>::const_iterator iter = macros.find(*ref + " " + elementData.info->name);
      Macro* macro = iter == macros.end() ? 0 : iter->second;
      if(!macro || macro->replaying)
      {
        if(macro)
          handleError("Looping reference \"" + *ref + "\"");
        else
          handleError("Unresolvable reference \"" + *ref + "\"");
        for(std::list<Macro*>::const_iterator iter = referencedMacros.begin(), end = referencedMacros.end(); iter != end; ++iter)
          (*iter)->replaying = false;
        return;
      }
      macro->replaying = true;
    }

    // create element
    Element* parentElement = element;
    Element* childElement = (this->*elementData.info->startElementProc)();
    element = childElement;
    checkAttributes();
    if(copiedAttributes)
      delete copiedAttributes;

    // parse direct subordinate elements
    parseMacroElements();
    ASSERT(this->elementData == &elementData);
    ASSERT(element == childElement);

    // parse inherited subordinate elements
    int parentLine = line;
    int parentColumn = column;
    MacroElement* parentReplayingMacroElement = replayingMacroElement;
    for(std::list<Macro*>::const_iterator iter = referencedMacros.begin(), end = referencedMacros.end(); iter != end; ++iter)
    {
      Macro* macro = *iter;

      ASSERT(macro->replaying);
      fileName.swap(macro->fileName);
      replayingMacroElement = macro;

      parseMacroElements();
      ASSERT(this->elementData == &elementData);
      ASSERT(element == childElement);

      fileName.swap(macro->fileName);
      macro->replaying = false;
    }
    line = parentLine;
    column = parentColumn;
    replayingMacroElement = parentReplayingMacroElement;

    //
    checkElements();
    ASSERT(element == childElement);
    if(element)
    {
      if(parentElement)
        element->addParent(*parentElement);

      if(elementData.info->flags & constantFlag && !elementData.usedPlaceholdersInAttributes)
      {
        replayingMacroElement->element = element;
        if(isReferenceOnlyElement)
          macro->element = element;
      }
    }
    return;
  }
}

void Parser::handleNode(const std::string& nodeName, Attributes& attributes) // from an input file
{
  std::unordered_map<std::string, const ElementInfo*>::const_iterator iter = elementInfos.find(nodeName);
  const ElementInfo* elementInfo = iter != elementInfos.end() ? iter->second : 0;
  ASSERT(!elementInfo || elementInfo->startElementProc);

  //
  if(!elementInfo || passedSimulationTag == (elementInfo->startElementProc == &Parser::simulationElement))
  {
    handleError("Unexpected element \"" + nodeName + "\"");
    readSubNodes(false);
    return;
  }

  // handle an infrastructural element
  if(elementInfo->elementClass == infrastructureClass)
  {
    ElementData elementData(0, elementInfo);
    this->elementData = &elementData;
    this->attributes = &attributes;

    (this->*elementInfo->startElementProc)();

    if(elementInfo->startElementProc == &Parser::includeElement)
    {
      std::string includeFile;
      includeFile.swap(this->includeFile);
      checkAttributes();
      readSubNodes(true);

      if(!includeFile.empty())
      {
        passedSimulationTag = false;
        size_t preErrorCount = errors->size();
        std::string oldRootDir = parseRootDir;
        std::string fileName;
        if(includeFile[0] != '/' && includeFile[0] != '\\' && // not absolute path on Unix
           (includeFile.size() < 2 || includeFile[1] != ':')) // or Windows
          fileName = parseRootDir + includeFile;
        else
          fileName = includeFile;
        std::string::size_type i = fileName.find_last_of("/\\");
        parseRootDir = i != std::string::npos ? fileName.substr(0, i + 1) : std::string();
        if(!readFile(fileName))
        {
          if(preErrorCount == errors->size())
            handleError("Could not include file \"" + includeFile + "\"");
        }
        parseRootDir = oldRootDir;
        passedSimulationTag = true;
      }
    }
    else
    {
      ASSERT(elementInfo->startElementProc == &Parser::simulationElement);
      checkAttributes();
      readSubNodes(true);
    }
    return;
  }

  if(elementInfo->elementClass == surfaceClass)
  {
    Attributes::iterator iter = attributes.find("diffuseTexture");
    if(iter != attributes.end() && iter->second.first != "" &&
       iter->second.first[0] != '/' && iter->second.first[0] != '\\' && // not absolute path on Unix
       (iter->second.first.size() < 2 || iter->second.first[1] != ':')) // or Windows
      iter->second.first = parseRootDir + iter->second.first;
  }

  // handle macro recording
  if(recordingMacroElement)
  { // since we are currently parsing a macro, add the element to the macro for later instantiation
    MacroElement* newMacroElement = new MacroElement(recordingMacroElement, elementInfo, attributes, line, column);
    recordingMacroElement->children.push_back(newMacroElement);
    recordingMacroElement = newMacroElement;
    readSubNodes(true);
    ASSERT(recordingMacroElement == newMacroElement);
    recordingMacroElement->endLine = line;
    recordingMacroElement->endColumn = column;
    recordingMacroElement = recordingMacroElement->parent;
    return;
  }

  // start recording a new macro
  else
  {
    const bool isScene = elementInfo->startElementProc == &Parser::sceneElement;
    if(isScene && sceneMacro)
    {
      handleError("Unexpected element \"" + nodeName + "\"");
      readSubNodes(false);
      return;
    }

    ElementData elementData(0, elementInfo);
    this->elementData = &elementData;
    this->attributes = &attributes;
    const std::string& name = getString("name", true);

    std::string macroName = name + " " + nodeName;
    if(macros.find(macroName) != macros.end())
    {
      handleError("Duplicated name \"" + name + "\"");
      readSubNodes(false);
      return;
    }
    Macro* macro = new Macro(elementInfo, fileName, attributes, line, column);
    macros[macroName] = macro;
    if(isScene)
      sceneMacro = macro;
    recordingMacroElement = macro;
    readSubNodes(true);
    recordingMacroElement->endLine = line;
    recordingMacroElement->endColumn = column;
    recordingMacroElement = 0;
    return;
  }
}

void Parser::handleText(std::string& text)
{
  if(!recordingMacroElement || !(recordingMacroElement->elementInfo->flags & textFlag))
  {
    handleError("Unexpected text");
    return;
  }

  ASSERT(recordingMacroElement->text.empty());
  recordingMacroElement->text.swap(text);
}

void Parser::handleError(const std::string& message)
{
  std::string fileName = this->fileName.find(parseRootDir) == 0 ? this->fileName.substr(parseRootDir.length()) : this->fileName;
  if(line)
  {
    char lineColumn[128];
    //if(column) // not even line numbers seem to correct
      //sprintf(lineColumn, ":%d:%d: error: ", line, column);
    //else
    sprintf(lineColumn, ":%d: error: ", line);
    errors->push_back(fileName + lineColumn + message);
  }
  else
    errors->push_back(fileName + ": error: " + message);
}

bool Parser::parse(const std::string& fileName, std::list<std::string>& errors)
{
  setlocale(LC_NUMERIC, "C");
  this->errors = &errors;

  ASSERT(!Simulation::simulation->scene);

  std::string::size_type i = fileName.find_last_of("/\\");
  parseRootDir = i != std::string::npos ? fileName.substr(0, i + 1) : std::string();

  // phase #1: reading the file and create "macros"
  size_t preErrorCount = errors.size();
  if(!readFile(fileName) || preErrorCount != errors.size())
    goto error;
  ASSERT(!Simulation::simulation->scene);

  // phase #2: using the macros to generate the scene graph
  parseSimulation();
  if(preErrorCount != errors.size())
    goto error;

  ASSERT(Simulation::simulation->scene);
  return true;

error:
  if(preErrorCount == errors.size())
  {
    this->fileName = fileName;
    this->line = 0;
    this->column = 0;
    handleError("Could not load file");
  }
  if(Simulation::simulation->scene)
  {
    for(std::list<Element*>::const_iterator iter = Simulation::simulation->elements.begin(), end = Simulation::simulation->elements.end(); iter != end; ++iter)
      delete *iter;
    Simulation::simulation->elements.clear();
    Simulation::simulation->scene = 0;
  }
  return false;
}

void Parser::checkAttributes()
{
  // make name attribute optional
  std::unordered_map<std::string, std::pair<std::string, int> >::const_iterator iter = attributes->find("name");
  if(iter != attributes->end())
  {
    const std::pair<std::string, int>& ai = iter->second;
    elementData->parsedAttributes |= 1 << ai.second;
  }

  const unsigned int allAttributes = attributes->empty() ? 0 : (0xffffffff >> (32 - attributes->size()));
  const unsigned int unexpectedAttributes = allAttributes & ~elementData->parsedAttributes;
  if(unexpectedAttributes)
  { // apparently there are unused attributs, find them and produce error messages
    for(std::unordered_map<std::string, std::pair<std::string, int> >::const_iterator iter = attributes->begin(), end = attributes->end(); iter != end; ++iter)
      if(unexpectedAttributes & (1 << iter->second.second))
        handleError("Unexpected attribute \"" + iter->first + "\"");
  }
}

void Parser::checkElements()
{
  const unsigned int missingChildren = elementData->info->requiredChildren & ~elementData->parsedChildren;
  if(missingChildren)
  {
    for(int i = 0; i < 32; ++i)
      if(missingChildren & (1 << i))
      {
        unsigned int missingClass = 1 << i;
        std::string elements;
        int count = 0;
        for(std::unordered_map<std::string, const ElementInfo*>::const_iterator iter = elementInfos.begin(), end = elementInfos.end(); iter != end; ++iter)
          if(iter->second->elementClass == missingClass)
          {
            if(count > 0)
              elements += ", ";
            elements += iter->second->name;
            ++count;
          }
        ASSERT(count);
        if(count <= 1)
          handleError("Expected element \"" + elements + "\"");
        else
          handleError("Expected one of the elements \"" + elements + "\"");
      }
  }
}

const std::string* Parser::resolvePlaceholder(const std::string& name)
{
  ASSERT(element);
  ElementData* elementData = this->elementData;
  elementData->usedPlaceholdersInAttributes = true;
  do
  {
    std::unordered_map<std::string, std::string>::const_iterator iter = elementData->vars.find(name);
    if(iter != elementData->vars.end())
      return &iter->second;
    elementData = elementData->parent;
  } while(elementData);
  return 0;
}

const std::string& Parser::replacePlaceholders(const std::string& str)
{
  const char* src = str.c_str();
  const char* varStart = strchr(src, '$');
  if(!varStart)
    return str;

  std::string& result = placeholderBuffer;
  result.resize(0);
  result += std::string(src, varStart - src);

  const char* varEnd;
  for(;;)
  {
    ++varStart;
    char c = *varStart;
    if(c == '(' || c == '{')
    {
      ++varStart;
      char cEnd = c == '(' ? ')' : '}';
      varEnd = strchr(varStart, cEnd);
      if(!varEnd)
      {
        handleError("Invalid attribute format");
        return str;
      }
      else
      {
        std::string name(varStart, varEnd - varStart);
        const std::string* value = resolvePlaceholder(name);
        if(value)
          result += *value;
        else
          result += std::string("$") + c + name + cEnd;
        ++varEnd;
      }
    }
    else
    {
      varEnd = varStart;
      while(isalnum(*varEnd))
        ++varEnd;
      std::string name(varStart, varEnd - varStart);
      const std::string* value = resolvePlaceholder(name);
      if(value)
        result += *value;
      else
        result += std::string("$") + name;
    }

    varStart = strchr(varEnd, '$');
    if(!varStart)
    {
      result += varEnd;
      return result;
    }
  }
}

bool Parser::getStringRaw(const char* key, bool required, const std::string*& value)
{
  std::unordered_map<std::string, std::pair<std::string, int> >::const_iterator iter = attributes->find(key);
  if(iter == attributes->end())
  {
    if(required)
      handleError("Expected attribute \"" + std::string(key) + "\"");
    return false;
  }
  const std::pair<std::string, int>& ai = iter->second;
  elementData->parsedAttributes |= 1 << ai.second;
  value = &replacePlaceholders(ai.first);
  return true;
}

bool Parser::getFloatRaw(const char* key, bool required, float& value)
{
  const std::string* strvalue;
  if(!getStringRaw(key, required, strvalue))
    return false;
  char* end;
  value = (float) strtod(strvalue->c_str(), &end);
  if(*end)
  {
    handleError("Expected float");
    return false;
  }
  return true;
}

bool Parser::getIntegerRaw(const char* key, bool required, int& value)
{
  const std::string* strvalue;
  if(!getStringRaw(key, required, strvalue))
    return false;
  char* end;
  value = (int) strtol(strvalue->c_str(), &end, 10);
  if(*end)
  {
    handleError("Expected integer");
    return false;
  }
  return true;
}

const std::string& Parser::getString(const char* key, bool required)
{
  const std::string* value;
  if(!getStringRaw(key, required, value))
  {
    static const std::string emptyString;
    return emptyString;
  }
  return *value;
}

bool Parser::getBool(const char* key, bool required, bool defaultValue)
{
  const std::string* value;
  if(!getStringRaw(key, required, value))
    return defaultValue;
  if(*value == "true" || *value == "1" || *value == "on")
    return true;
  if(*value == "false" || *value == "0" || *value == "off")
    return false;
  handleError("Expected boolean value (true or false)");
  return defaultValue;
}

float Parser::getFloat(const char* key, bool required, float defaultValue)
{
  float value;
  return getFloatRaw(key, required, value) ? value : defaultValue;
};

float Parser::getFloatPositive(const char* key, bool required, float defaultValue)
{
  float value;
  if(!getFloatRaw(key, required, value))
    return defaultValue;
  if(value < 0.f)
  {
    char msg[256];
    sprintf(msg, "Expected a positive value instead of %g", value);
    handleError(msg);
    return defaultValue;
  }
  return value;
}

int Parser::getInteger(const char* key, bool required, int defaultValue)
{
  int value;
  return getIntegerRaw(key, required, value) ? value : defaultValue;
}

float Parser::getFloatMinMax(const char* key, bool required, float defaultValue, float min, float max)
{
  float value;
  if(!getFloatRaw(key, required, value))
    return defaultValue;
  if(value < min || value > max)
  {
    char msg[256];
    sprintf(msg, "Expected a value between %g and %g instead of %g", min, max, value);
    handleError(msg);
    return defaultValue;
  }
  return value;
}

bool Parser::getFloatAndUnit(const char* key, bool required, float defaultValue, float& value, char** unit)
{
  const std::string* strvalue;
  if(!getStringRaw(key, required, strvalue))
    return false;
  value = (float) strtod(strvalue->c_str(), unit);
  while(isspace(**unit))
    ++(*unit);
  return true;
}

float Parser::getUnit(const char* key, bool required, float defaultValue)
{
  float result = 1.f;
  const std::string* s;
  if(!getStringRaw(key, required, s))
    return defaultValue;
  if(strcmp(s->c_str(), "mm") == 0)
    result = 0.001f;
  else if(strcmp(s->c_str(), "cm") == 0)
    result = 0.01f;
  else if(strcmp(s->c_str(), "dm") == 0)
    result = 0.1f;
  else if(strcmp(s->c_str(), "km") == 0)
    result = 1000.f;
  else if(strcmp(s->c_str(), "m") != 0)
  {
    handleError("Unexpected unit \"" + *s + " (expected one of \"mm, cm, dm, m, km\")");
    return defaultValue;
  }
  return result;
}

int Parser::getIntegerNonZeroPositive(const char* key, bool required, int defaultValue)
{
  int value;
  if(!getIntegerRaw(key, required, value))
    return defaultValue;
  if(value <= 0)
  {
    char msg[256];
    sprintf(msg, "Expected a positive non-zero value instead of %d", value);
    handleError(msg);
    return defaultValue;
  }
  return value;
}

float Parser::getLength(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "mm") == 0)
      result *= 0.001f;
    else if(strcmp(endPtr, "cm") == 0)
      result *= 0.01f;
    else if(strcmp(endPtr, "dm") == 0)
      result *= 0.1f;
    else if(strcmp(endPtr, "km") == 0)
      result *= 1000.f;
    else if(strcmp(endPtr, "m") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"mm, cm, dm, m, km\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getMassLengthLength(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "g*mm^2") == 0)
      result *= 0.001f * 0.001f * 0.001f; // 1.0 * 10^-9
    else if(strcmp(endPtr, "kg*m^2") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"g*mm^2, kg*m^2\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getMass(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "g") == 0)
      result *= 0.001f;
    else if(strcmp(endPtr, "kg") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"g, kg\")");
      return defaultValue;
    }
  }
  if(result <= 0.f)
    handleError("A mass should be greater than zero");
  return result;
}

float Parser::getAngle(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "degree") == 0)
      result *= float(M_PI) / 180.f;
    else if(strcmp(endPtr, "radian") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"degree, radian\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getAngleNonZeroPositive(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(result <= 0)
  {
    char msg[256];
    sprintf(msg, "Expected a positive non-zero value instead of %g", result);
    handleError(msg);
    return defaultValue;
  }
  if(*endPtr)
  {
    if(strcmp(endPtr, "degree") == 0)
      result *= float(M_PI) / 180.f;
    else if(strcmp(endPtr, "radian") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"degree, radian\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getAngularVelocity(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "degree/s") == 0)
      result *= float(M_PI) / 180.f;
    else if(strcmp(endPtr, "radian/s") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"degree/s, radian/s\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getForce(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "N") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected \"N\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getVelocity(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "mm/s") == 0)
      result *= 0.001f;
    else if(strcmp(endPtr, "cm/s") == 0)
      result *= 0.01f;
    else if(strcmp(endPtr, "dm/s") == 0)
      result *= 0.1f;
    else if(strcmp(endPtr, "km/s") == 0)
      result *= 1000.f;
    else if(strcmp(endPtr, "km/h") == 0)
      result /= 3.6f;
    else if(strcmp(endPtr, "m/s") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"mm/s, cm/s, dm/s, m/s, km/s, km/h\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getAcceleration(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(*endPtr)
  {
    if(strcmp(endPtr, "mm/s^2") == 0)
      result *= 0.001f;
    else if(strcmp(endPtr, "m/s^2") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected one of \"mm/s^2, m/s^2\")");
      return defaultValue;
    }
  }
  return result;
}

float Parser::getTimeNonZeroPositive(const char* key, bool required, float defaultValue)
{
  float result;
  char* endPtr;
  if(!getFloatAndUnit(key, required, defaultValue, result, &endPtr))
    return defaultValue;
  if(result <= 0)
  {
    char msg[256];
    sprintf(msg, "Expected a positive non-zero value instead of %g", result);
    handleError(msg);
    return defaultValue;
  }
  if(*endPtr)
  {
    if(strcmp(endPtr, "s") != 0)
    {
      handleError("Unexpected unit \"" + std::string(endPtr) + " (expected \"s\")");
      return defaultValue;
    }
  }
  return result;
}

bool Parser::getColor(const char* key, bool required, float* color)
{
  const std::string* strvalue;
  if(!getStringRaw(key, required, strvalue))
    return false;
  const char* strclr = strvalue->c_str();
  static const float f1_255 = 1.f / 255.f;
  if(*strclr == '#')
  { // html style color (#rrggbb and #rgb)
    // + self invented #rrggbbaa and #rgba

    ++strclr;
    unsigned int lcol = 0;
    const char* endPtr = strclr;
    for(;;)
    {
      int c = tolower(*endPtr);
      if(c >= '0' && c <= '9')
      {
        lcol <<= 4;
        lcol |= c - '0';
      }
      else if(c >= 'a' && c <= 'f')
      {
        lcol <<= 4;
        lcol |= c - 'a' + 10;
      }
      else if(!c)
        break;
      else
      {
        handleError("Invalid color format");
        return false;
      }
      ++endPtr;
    }
    switch(endPtr - strclr)
    {
      case 3:
        color[0] = float(lcol >> 8) * f1_255;
        color[1] = float((lcol >> 4) & 0xf) * f1_255;
        color[2] = float(lcol & 0xf) * f1_255;
        color[3] = 1.f;
        return true;
      case 4:
        color[0] = float(lcol >> 12) * f1_255;
        color[1] = float((lcol >> 8) & 0xf) * f1_255;
        color[2] = float((lcol >> 4) & 0xf) * f1_255;
        color[3] = float(lcol & 0xf) * f1_255;
        return true;
      case 6:
        color[0] = float(lcol >> 16) * f1_255;
        color[1] = float((lcol >> 8) & 0xff) * f1_255;
        color[2] = float(lcol & 0xff) * f1_255;
        color[3] = 1.f;
        return true;
      case 8:
        color[0] = float(lcol >> 24) * f1_255;
        color[1] = float((lcol >> 16) & 0xff) * f1_255;
        color[2] = float((lcol >> 8) & 0xff) * f1_255;
        color[3] = float(lcol & 0xff) * f1_255;
        return true;
      default:
        handleError("Invalid color format");
        return false;
    }
  }
  else if(strncmp(strclr, "rgb(", 4) == 0)
  { // css style rgb color (rgb(r,g,b) with r,g,b\in[0..255]\cup[0%,..,100%])
    strclr += 4;
    for(int i = 0;; ++i)
    {
      while(isspace(*strclr))
        ++strclr;
      color[i] = (float) strtod(strclr, (char**)&strclr);
      if(*strclr == '%')
      {
        ++strclr;
        color[i] *= 0.01f;
      }
      else
        color[i] *= f1_255;
      while(isspace(*strclr))
        ++strclr;
      if(i >= 2 || *strclr != ',')
        break;
      ++strclr;
    }
    if(strcmp(strclr, ")") != 0)
    {
      handleError("Invalid color format");
      return false;
    }
    color[3] = 1.f;
    return true;
  }
  else if(strncmp(strclr, "rgba(", 5) == 0)
  { // css3 style rgba color (rgba(r,g,b,a) with r,g,b\in[0..255]\cup[0%,..,100%] and a\in[0..1])
    // http://www.w3.org/TR/css3-color/
    strclr += 5;
    for(int i = 0;; ++i)
    {
      while(isspace(*strclr))
        ++strclr;
      color[i] = (float) strtod(strclr, (char**)&strclr);
      if(i < 3)
      {
        if(*strclr == '%')
        {
          ++strclr;
          color[i] *= 0.01f;
        }
        else
          color[i] *= f1_255;
      }
      while(isspace(*strclr))
        ++strclr;
      if(i >= 3 || *strclr != ',')
        break;
      ++strclr;
    }
    if(strcmp(strclr, ")") != 0)
    {
      handleError("Invalid color format");
      return false;
    }
    return true;
  }
  else
  {
    handleError("Invalid color format");
    return false;
  }
  return false;
}
