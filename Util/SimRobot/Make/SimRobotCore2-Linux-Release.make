
.SUFFIXES:
.PHONY: SimRobotCore2

__SOFLAGS := -fpic
__CPPFLAGS := -std=c++11 -pipe -mssse3 -ffast-math -Wall -Wsign-compare -Wno-address -Wno-deprecated -Wno-overloaded-virtual -Wno-reorder -Wno-sign-conversion -Wno-strict-aliasing -Wno-switch -Wno-uninitialized -Wno-unused-parameter -O3 -fomit-frame-pointer
__CFLAGS := -Wall -Os -fomit-frame-pointer
__DEFINES := -DLINUX -DNDEBUG -DQT_SHARED -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL -DdSINGLE -DQT_NO_DEBUG
__INCLUDEPATHS := -I../Src/SimRobotCore2 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/QtOpenGL -I/usr/include/Qt -I/usr/include/libxml2 -I../Util/ode/Linux/include
__OBJECTS := ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/ActuatorsWidget.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/CoreModule.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/SensorWidget.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/SimObjectRenderer.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/SimObjectWidget.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser/Element.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser/Parser.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser/Reader.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform/Assert.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform/OffscreenRenderer.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform/System.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Axis.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Body.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Compound.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/GraphicalObject.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/PhysicalObject.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Scene.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/SimObject.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Simulation.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Actuator.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Hinge.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Joint.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Slider.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/Appearance.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/BoxAppearance.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/CapsuleAppearance.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/ComplexAppearance.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/CylinderAppearance.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/SphereAppearance.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/BoxGeometry.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/CapsuleGeometry.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/CylinderGeometry.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/Geometry.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/SphereGeometry.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/BoxMass.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/InertiaMatrixMass.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/Mass.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/SphereMass.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors/ServoMotor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors/VelocityMotor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Accelerometer.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/ApproxDistanceSensor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Camera.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/CollisionSensor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/DepthImageSensor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Gyroscope.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Sensor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/SingleDistanceSensor.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/Matrix3x3.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/OpenGLTools.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/Texture.o ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/Vector3.o ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.o ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.o ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/moc_SensorWidget.o ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.o
__LINKFLAGS := -s
__LIBPATHS := -L../Util/ode/Linux32/lib
__LIBS := -lrt -lpthread -lode -lGLEW -lxml2 -lQtGui -lQtCore -lQtOpenGL -lGLU -lGL
__CXXLINKER := $(CXX)
__CCLINKER := $(CC)
__ARLINKER := $(AR)

SimRobotCore2: ../Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.cpp ../Build/SimRobotCore2/Linux/Release/moc_SensorWidget.cpp ../Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.cpp ../Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.cpp ../Build/SimRobotCore2/Linux/Release/libSimRobotCore2.so

../Build/SimRobotCore2/Linux/Release/libSimRobotCore2.so: $(__OBJECTS)  | ../Build/SimRobotCore2/Linux/Release
	@echo "Linking SimRobotCore2..."
	@$(__CXXLINKER) -shared $(__SOFLAGS) -o $@ $(__OBJECTS) $(__LINKFLAGS) $(LDFLAGS) $(__LIBPATHS) $(__LIBS)

../Build/SimRobotCore2/Linux/Release:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release:
	@mkdir -p $@

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/ActuatorsWidget.o: ../Src/SimRobotCore2/ActuatorsWidget.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2
	@echo "../Src/SimRobotCore2/ActuatorsWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/CoreModule.o: ../Src/SimRobotCore2/CoreModule.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2
	@echo "../Src/SimRobotCore2/CoreModule.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/SensorWidget.o: ../Src/SimRobotCore2/SensorWidget.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2
	@echo "../Src/SimRobotCore2/SensorWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/SimObjectRenderer.o: ../Src/SimRobotCore2/SimObjectRenderer.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2
	@echo "../Src/SimRobotCore2/SimObjectRenderer.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/SimObjectWidget.o: ../Src/SimRobotCore2/SimObjectWidget.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2
	@echo "../Src/SimRobotCore2/SimObjectWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser/Element.o: ../Src/SimRobotCore2/Parser/Element.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser
	@echo "../Src/SimRobotCore2/Parser/Element.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser/Parser.o: ../Src/SimRobotCore2/Parser/Parser.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser
	@echo "../Src/SimRobotCore2/Parser/Parser.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser/Reader.o: ../Src/SimRobotCore2/Parser/Reader.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Parser
	@echo "../Src/SimRobotCore2/Parser/Reader.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform/Assert.o: ../Src/SimRobotCore2/Platform/Assert.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform
	@echo "../Src/SimRobotCore2/Platform/Assert.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform/OffscreenRenderer.o: ../Src/SimRobotCore2/Platform/OffscreenRenderer.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform
	@echo "../Src/SimRobotCore2/Platform/OffscreenRenderer.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform/System.o: ../Src/SimRobotCore2/Platform/System.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Platform
	@echo "../Src/SimRobotCore2/Platform/System.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Axis.o: ../Src/SimRobotCore2/Simulation/Axis.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/Axis.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Body.o: ../Src/SimRobotCore2/Simulation/Body.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/Body.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Compound.o: ../Src/SimRobotCore2/Simulation/Compound.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/Compound.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/GraphicalObject.o: ../Src/SimRobotCore2/Simulation/GraphicalObject.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/GraphicalObject.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/PhysicalObject.o: ../Src/SimRobotCore2/Simulation/PhysicalObject.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/PhysicalObject.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Scene.o: ../Src/SimRobotCore2/Simulation/Scene.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/Scene.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/SimObject.o: ../Src/SimRobotCore2/Simulation/SimObject.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/SimObject.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Simulation.o: ../Src/SimRobotCore2/Simulation/Simulation.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation
	@echo "../Src/SimRobotCore2/Simulation/Simulation.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Actuator.o: ../Src/SimRobotCore2/Simulation/Actuators/Actuator.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators
	@echo "../Src/SimRobotCore2/Simulation/Actuators/Actuator.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Hinge.o: ../Src/SimRobotCore2/Simulation/Actuators/Hinge.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators
	@echo "../Src/SimRobotCore2/Simulation/Actuators/Hinge.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Joint.o: ../Src/SimRobotCore2/Simulation/Actuators/Joint.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators
	@echo "../Src/SimRobotCore2/Simulation/Actuators/Joint.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators/Slider.o: ../Src/SimRobotCore2/Simulation/Actuators/Slider.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Actuators
	@echo "../Src/SimRobotCore2/Simulation/Actuators/Slider.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/Appearance.o: ../Src/SimRobotCore2/Simulation/Appearances/Appearance.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances
	@echo "../Src/SimRobotCore2/Simulation/Appearances/Appearance.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/BoxAppearance.o: ../Src/SimRobotCore2/Simulation/Appearances/BoxAppearance.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances
	@echo "../Src/SimRobotCore2/Simulation/Appearances/BoxAppearance.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/CapsuleAppearance.o: ../Src/SimRobotCore2/Simulation/Appearances/CapsuleAppearance.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances
	@echo "../Src/SimRobotCore2/Simulation/Appearances/CapsuleAppearance.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/ComplexAppearance.o: ../Src/SimRobotCore2/Simulation/Appearances/ComplexAppearance.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances
	@echo "../Src/SimRobotCore2/Simulation/Appearances/ComplexAppearance.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/CylinderAppearance.o: ../Src/SimRobotCore2/Simulation/Appearances/CylinderAppearance.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances
	@echo "../Src/SimRobotCore2/Simulation/Appearances/CylinderAppearance.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances/SphereAppearance.o: ../Src/SimRobotCore2/Simulation/Appearances/SphereAppearance.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Appearances
	@echo "../Src/SimRobotCore2/Simulation/Appearances/SphereAppearance.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/BoxGeometry.o: ../Src/SimRobotCore2/Simulation/Geometries/BoxGeometry.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries
	@echo "../Src/SimRobotCore2/Simulation/Geometries/BoxGeometry.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/CapsuleGeometry.o: ../Src/SimRobotCore2/Simulation/Geometries/CapsuleGeometry.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries
	@echo "../Src/SimRobotCore2/Simulation/Geometries/CapsuleGeometry.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/CylinderGeometry.o: ../Src/SimRobotCore2/Simulation/Geometries/CylinderGeometry.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries
	@echo "../Src/SimRobotCore2/Simulation/Geometries/CylinderGeometry.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/Geometry.o: ../Src/SimRobotCore2/Simulation/Geometries/Geometry.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries
	@echo "../Src/SimRobotCore2/Simulation/Geometries/Geometry.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries/SphereGeometry.o: ../Src/SimRobotCore2/Simulation/Geometries/SphereGeometry.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Geometries
	@echo "../Src/SimRobotCore2/Simulation/Geometries/SphereGeometry.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/BoxMass.o: ../Src/SimRobotCore2/Simulation/Masses/BoxMass.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses
	@echo "../Src/SimRobotCore2/Simulation/Masses/BoxMass.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/InertiaMatrixMass.o: ../Src/SimRobotCore2/Simulation/Masses/InertiaMatrixMass.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses
	@echo "../Src/SimRobotCore2/Simulation/Masses/InertiaMatrixMass.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/Mass.o: ../Src/SimRobotCore2/Simulation/Masses/Mass.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses
	@echo "../Src/SimRobotCore2/Simulation/Masses/Mass.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses/SphereMass.o: ../Src/SimRobotCore2/Simulation/Masses/SphereMass.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Masses
	@echo "../Src/SimRobotCore2/Simulation/Masses/SphereMass.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors/ServoMotor.o: ../Src/SimRobotCore2/Simulation/Motors/ServoMotor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors
	@echo "../Src/SimRobotCore2/Simulation/Motors/ServoMotor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors/VelocityMotor.o: ../Src/SimRobotCore2/Simulation/Motors/VelocityMotor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Motors
	@echo "../Src/SimRobotCore2/Simulation/Motors/VelocityMotor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Accelerometer.o: ../Src/SimRobotCore2/Simulation/Sensors/Accelerometer.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/Accelerometer.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/ApproxDistanceSensor.o: ../Src/SimRobotCore2/Simulation/Sensors/ApproxDistanceSensor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/ApproxDistanceSensor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Camera.o: ../Src/SimRobotCore2/Simulation/Sensors/Camera.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/Camera.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/CollisionSensor.o: ../Src/SimRobotCore2/Simulation/Sensors/CollisionSensor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/CollisionSensor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/DepthImageSensor.o: ../Src/SimRobotCore2/Simulation/Sensors/DepthImageSensor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/DepthImageSensor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Gyroscope.o: ../Src/SimRobotCore2/Simulation/Sensors/Gyroscope.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/Gyroscope.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/Sensor.o: ../Src/SimRobotCore2/Simulation/Sensors/Sensor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/Sensor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors/SingleDistanceSensor.o: ../Src/SimRobotCore2/Simulation/Sensors/SingleDistanceSensor.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Simulation/Sensors
	@echo "../Src/SimRobotCore2/Simulation/Sensors/SingleDistanceSensor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/Matrix3x3.o: ../Src/SimRobotCore2/Tools/Matrix3x3.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools
	@echo "../Src/SimRobotCore2/Tools/Matrix3x3.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/OpenGLTools.o: ../Src/SimRobotCore2/Tools/OpenGLTools.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools
	@echo "../Src/SimRobotCore2/Tools/OpenGLTools.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/Texture.o: ../Src/SimRobotCore2/Tools/Texture.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools
	@echo "../Src/SimRobotCore2/Tools/Texture.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools/Vector3.o: ../Src/SimRobotCore2/Tools/Vector3.cpp | ../Build/SimRobotCore2/Linux/Release/Src/SimRobotCore2/Tools
	@echo "../Src/SimRobotCore2/Tools/Vector3.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.cpp: ../Src/SimRobotCore2/ActuatorsWidget.h | ../Build/SimRobotCore2/Linux/Release
	@echo "../Src/SimRobotCore2/ActuatorsWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL -DdSINGLE -DQT_NO_DEBUG  -I../Src/SimRobotCore2 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/QtOpenGL -I/usr/include/Qt -I/usr/include/libxml2 -I../Util/ode/Linux/include ../Src/SimRobotCore2/ActuatorsWidget.h -o ../Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.cpp

../Build/SimRobotCore2/Linux/Release/moc_SensorWidget.cpp: ../Src/SimRobotCore2/SensorWidget.h | ../Build/SimRobotCore2/Linux/Release
	@echo "../Src/SimRobotCore2/SensorWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL -DdSINGLE -DQT_NO_DEBUG  -I../Src/SimRobotCore2 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/QtOpenGL -I/usr/include/Qt -I/usr/include/libxml2 -I../Util/ode/Linux/include ../Src/SimRobotCore2/SensorWidget.h -o ../Build/SimRobotCore2/Linux/Release/moc_SensorWidget.cpp

../Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.cpp: ../Src/SimRobotCore2/SimObjectWidget.h | ../Build/SimRobotCore2/Linux/Release
	@echo "../Src/SimRobotCore2/SimObjectWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL -DdSINGLE -DQT_NO_DEBUG  -I../Src/SimRobotCore2 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/QtOpenGL -I/usr/include/Qt -I/usr/include/libxml2 -I../Util/ode/Linux/include ../Src/SimRobotCore2/SimObjectWidget.h -o ../Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.cpp

../Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.cpp: ../Src/SimRobotCore2/SimRobotCore2.qrc ../Src/SimRobotCore2/Icons/arrow_rotate_clockwise.png ../Src/SimRobotCore2/Icons/brick.png ../Src/SimRobotCore2/Icons/bricks.png ../Src/SimRobotCore2/Icons/camera.png ../Src/SimRobotCore2/Icons/chart_line.png ../Src/SimRobotCore2/Icons/DragPlane.png ../Src/SimRobotCore2/Icons/layers.png ../Src/SimRobotCore2/Icons/link.png ../Src/SimRobotCore2/Icons/note.png ../Src/SimRobotCore2/Icons/opening_angle.png ../Src/SimRobotCore2/Icons/slider.png ../Src/SimRobotCore2/Icons/transmit_go.png | ../Build/SimRobotCore2/Linux/Release
	@echo "../Src/SimRobotCore2/SimRobotCore2.qrc (Qt rcc)"
	@rcc -name SimRobotCore2 ../Src/SimRobotCore2/SimRobotCore2.qrc -o ../Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.cpp

../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.o: ../Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.cpp | ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release
	@echo "../Build/SimRobotCore2/Linux/Release/qrc_SimRobotCore2.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.o: ../Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.cpp | ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release
	@echo "../Build/SimRobotCore2/Linux/Release/moc_ActuatorsWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/moc_SensorWidget.o: ../Build/SimRobotCore2/Linux/Release/moc_SensorWidget.cpp | ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release
	@echo "../Build/SimRobotCore2/Linux/Release/moc_SensorWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.o: ../Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.cpp | ../Build/SimRobotCore2/Linux/Release/Build/SimRobotCore2/Linux/Release
	@echo "../Build/SimRobotCore2/Linux/Release/moc_SimObjectWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

-include $(__OBJECTS:%.o=%.d)

%.h: ;
%.hh: ;
%.hxx: ;
%.hpp: ;
%.c: ;
%.cc: ;
%.cxx: ;
%.cpp: ;
%.d: ;
