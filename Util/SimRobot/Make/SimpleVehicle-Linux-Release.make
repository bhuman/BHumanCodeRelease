
.SUFFIXES:
.PHONY: SimpleVehicle

__SOFLAGS := -fpic
__CPPFLAGS := -std=c++11 -pipe -mssse3 -ffast-math -Wall -Wsign-compare -Wno-address -Wno-deprecated -Wno-overloaded-virtual -Wno-reorder -Wno-sign-conversion -Wno-strict-aliasing -Wno-switch -Wno-uninitialized -Wno-unused-parameter -O3 -fomit-frame-pointer
__CFLAGS := -Wall -Os -fomit-frame-pointer
__DEFINES := -DLINUX -DNDEBUG -DQT_NO_DEBUG -DQT_SHARED -DQT_CORE_LIB -DQT_NO_STL
__INCLUDEPATHS := -I../Src/SimRobotCore2 -I../Util/glew/Linux/include -I/usr/include/qt4/QtCore -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/Qt
__OBJECTS := ../Build/SimpleVehicle/Linux/Release/Src/Controllers/SimpleVehicleController.o
__LINKFLAGS := 
__LIBPATHS := 
__LIBS := -lGLEW -lQtCore -lGLU -lGL
__CXXLINKER := $(CXX)
__CCLINKER := $(CC)
__ARLINKER := $(AR)

SimpleVehicle: ../Build/SimpleVehicle/Linux/Release/libSimpleVehicle.so

../Build/SimpleVehicle/Linux/Release/libSimpleVehicle.so: $(__OBJECTS)  | ../Build/SimpleVehicle/Linux/Release
	@echo "Linking SimpleVehicle..."
	@$(__CXXLINKER) -shared $(__SOFLAGS) -o $@ $(__OBJECTS) $(__LINKFLAGS) $(LDFLAGS) $(__LIBPATHS) $(__LIBS)

../Build/SimpleVehicle/Linux/Release:
	@mkdir -p $@

../Build/SimpleVehicle/Linux/Release/Src/Controllers:
	@mkdir -p $@

../Build/SimpleVehicle/Linux/Release/Src/Controllers/SimpleVehicleController.o: ../Src/Controllers/SimpleVehicleController.cpp | ../Build/SimpleVehicle/Linux/Release/Src/Controllers
	@echo "../Src/Controllers/SimpleVehicleController.cpp"
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
