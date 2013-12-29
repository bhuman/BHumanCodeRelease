
.SUFFIXES:
.PHONY: Factory

__SOFLAGS := -fpic
__CPPFLAGS := -std=c++11 -pipe -mssse3 -ffast-math -Wall -Wsign-compare -Wno-address -Wno-deprecated -Wno-overloaded-virtual -Wno-reorder -Wno-sign-conversion -Wno-strict-aliasing -Wno-switch -Wno-uninitialized -Wno-unused-parameter -g
__CFLAGS := -Wall -g
__DEFINES := -DLINUX -D_DEBUG -DQT_NO_DEBUG -DQT_SHARED -DQT_CORE_LIB -DQT_NO_STL
__INCLUDEPATHS := -I../Src/SimRobotCore2 -I../Util/glew/Linux/include -I/usr/include/qt4/QtCore -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/Qt
__OBJECTS := ../Build/Factory/Linux/Debug/Src/Controllers/FactoryController.o
__LINKFLAGS := 
__LIBPATHS := 
__LIBS := -lGLEW -lQtCore -lGLU -lGL
__CXXLINKER := $(CXX)
__CCLINKER := $(CC)
__ARLINKER := $(AR)

Factory: ../Build/Factory/Linux/Debug/libFactory.so

../Build/Factory/Linux/Debug/libFactory.so: $(__OBJECTS)  | ../Build/Factory/Linux/Debug
	@echo "Linking Factory..."
	@$(__CXXLINKER) -shared $(__SOFLAGS) -o $@ $(__OBJECTS) $(__LINKFLAGS) $(LDFLAGS) $(__LIBPATHS) $(__LIBS)

../Build/Factory/Linux/Debug:
	@mkdir -p $@

../Build/Factory/Linux/Debug/Src/Controllers:
	@mkdir -p $@

../Build/Factory/Linux/Debug/Src/Controllers/FactoryController.o: ../Src/Controllers/FactoryController.cpp | ../Build/Factory/Linux/Debug/Src/Controllers
	@echo "../Src/Controllers/FactoryController.cpp"
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
