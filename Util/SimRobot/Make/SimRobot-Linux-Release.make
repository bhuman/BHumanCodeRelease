
.SUFFIXES:
.PHONY: SimRobot

__SOFLAGS := 
__CPPFLAGS := -std=c++11 -pipe -mssse3 -ffast-math -Wall -Wsign-compare -Wno-address -Wno-deprecated -Wno-overloaded-virtual -Wno-reorder -Wno-sign-conversion -Wno-strict-aliasing -Wno-switch -Wno-uninitialized -Wno-unused-parameter -O3 -fomit-frame-pointer
__CFLAGS := -Wall -Os -fomit-frame-pointer
__DEFINES := -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SVG_LIB -DQT_NO_STL -DQT_NO_DEBUG
__INCLUDEPATHS := -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/qt4/QtSvg -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt -I/usr/include/QtSvg
__OBJECTS := ../Build/SimRobot/Linux/Release/Src/SimRobot/Main.o ../Build/SimRobot/Linux/Release/Src/SimRobot/MainWindow.o ../Build/SimRobot/Linux/Release/Src/SimRobot/RegisteredDockWidget.o ../Build/SimRobot/Linux/Release/Src/SimRobot/SceneGraphDockWidget.o ../Build/SimRobot/Linux/Release/Src/SimRobot/StatusBar.o ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/qrc_SimRobot.o ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_MainWindow.o ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.o ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.o ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_StatusBar.o
__LINKFLAGS := -s
__LIBPATHS := 
__LIBS := -lrt -lpthread -lQtGui -lQtCore -lQtSvg
__CXXLINKER := $(CXX)
__CCLINKER := $(CC)
__ARLINKER := $(AR)

SimRobot: ../Build/SimRobot/Linux/Release/moc_MainWindow.cpp ../Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.cpp ../Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.cpp ../Build/SimRobot/Linux/Release/moc_StatusBar.cpp ../Build/SimRobot/Linux/Release/help.qch ../Build/SimRobot/Linux/Release/helpcollection.qhc ../Build/SimRobot/Linux/Release/libSimRobotCore2.so ../Build/SimRobot/Linux/Release/libSimRobotEditor.so ../Build/SimRobot/Linux/Release/libSimRobotHelp.so ../Build/SimRobot/Linux/Release/libFactory.so ../Build/SimRobot/Linux/Release/libSimpleVehicle.so ../Build/SimRobot/Linux/Release/qrc_SimRobot.cpp ../Build/SimRobot/Linux/Release/SimRobot

../Build/SimRobot/Linux/Release/SimRobot: $(__OBJECTS) ../Build/SimRobotCore2/Linux/Release/libSimRobotCore2.so ../Build/SimRobotEditor/Linux/Release/libSimRobotEditor.so ../Build/SimRobotHelp/Linux/Release/libSimRobotHelp.so ../Build/Factory/Linux/Release/libFactory.so ../Build/SimpleVehicle/Linux/Release/libSimpleVehicle.so | ../Build/SimRobot/Linux/Release
	@echo "Linking SimRobot..."
	@$(__CXXLINKER) -o $@ $(__OBJECTS) $(__LINKFLAGS) $(LDFLAGS) $(__LIBPATHS) $(__LIBS)

../Build/SimRobot/Linux/Release:
	@mkdir -p $@

../Build/SimRobot/Linux/Release/Src/SimRobot:
	@mkdir -p $@

../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release:
	@mkdir -p $@

../Build/SimRobot/Linux/Release/Src/SimRobot/Main.o: ../Src/SimRobot/Main.cpp | ../Build/SimRobot/Linux/Release/Src/SimRobot
	@echo "../Src/SimRobot/Main.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Src/SimRobot/MainWindow.o: ../Src/SimRobot/MainWindow.cpp | ../Build/SimRobot/Linux/Release/Src/SimRobot
	@echo "../Src/SimRobot/MainWindow.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Src/SimRobot/RegisteredDockWidget.o: ../Src/SimRobot/RegisteredDockWidget.cpp | ../Build/SimRobot/Linux/Release/Src/SimRobot
	@echo "../Src/SimRobot/RegisteredDockWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Src/SimRobot/SceneGraphDockWidget.o: ../Src/SimRobot/SceneGraphDockWidget.cpp | ../Build/SimRobot/Linux/Release/Src/SimRobot
	@echo "../Src/SimRobot/SceneGraphDockWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Src/SimRobot/StatusBar.o: ../Src/SimRobot/StatusBar.cpp | ../Build/SimRobot/Linux/Release/Src/SimRobot
	@echo "../Src/SimRobot/StatusBar.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/moc_MainWindow.cpp: ../Src/SimRobot/MainWindow.h | ../Build/SimRobot/Linux/Release
	@echo "../Src/SimRobot/MainWindow.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SVG_LIB -DQT_NO_STL -DQT_NO_DEBUG  -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/qt4/QtSvg -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt -I/usr/include/QtSvg ../Src/SimRobot/MainWindow.h -o ../Build/SimRobot/Linux/Release/moc_MainWindow.cpp

../Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.cpp: ../Src/SimRobot/RegisteredDockWidget.h | ../Build/SimRobot/Linux/Release
	@echo "../Src/SimRobot/RegisteredDockWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SVG_LIB -DQT_NO_STL -DQT_NO_DEBUG  -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/qt4/QtSvg -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt -I/usr/include/QtSvg ../Src/SimRobot/RegisteredDockWidget.h -o ../Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.cpp

../Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.cpp: ../Src/SimRobot/SceneGraphDockWidget.h | ../Build/SimRobot/Linux/Release
	@echo "../Src/SimRobot/SceneGraphDockWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SVG_LIB -DQT_NO_STL -DQT_NO_DEBUG  -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/qt4/QtSvg -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt -I/usr/include/QtSvg ../Src/SimRobot/SceneGraphDockWidget.h -o ../Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.cpp

../Build/SimRobot/Linux/Release/moc_StatusBar.cpp: ../Src/SimRobot/StatusBar.h | ../Build/SimRobot/Linux/Release
	@echo "../Src/SimRobot/StatusBar.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SVG_LIB -DQT_NO_STL -DQT_NO_DEBUG  -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/qt4/QtSvg -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt -I/usr/include/QtSvg ../Src/SimRobot/StatusBar.h -o ../Build/SimRobot/Linux/Release/moc_StatusBar.cpp

../Build/SimRobot/Linux/Release/help.qch: ../Build/SimRobotHelp/Linux/Release/help.qch | ../Build/SimRobot/Linux/Release
	@echo "../Build/SimRobotHelp/Linux/Release/help.qch (copy)"
	@cp ../Build/SimRobotHelp/Linux/Release/help.qch ../Build/SimRobot/Linux/Release/help.qch

../Build/SimRobot/Linux/Release/helpcollection.qhc: ../Build/SimRobotHelp/Linux/Release/helpcollection.qhc | ../Build/SimRobot/Linux/Release
	@echo "../Build/SimRobotHelp/Linux/Release/helpcollection.qhc (copy)"
	@cp ../Build/SimRobotHelp/Linux/Release/helpcollection.qhc ../Build/SimRobot/Linux/Release/helpcollection.qhc

../Build/SimRobot/Linux/Release/libSimRobotCore2.so: ../Build/SimRobotCore2/Linux/Release/libSimRobotCore2.so | ../Build/SimRobot/Linux/Release
	@echo "../Build/SimRobotCore2/Linux/Release/libSimRobotCore2.so (copy)"
	@cp ../Build/SimRobotCore2/Linux/Release/libSimRobotCore2.so ../Build/SimRobot/Linux/Release/libSimRobotCore2.so

../Build/SimRobot/Linux/Release/libSimRobotEditor.so: ../Build/SimRobotEditor/Linux/Release/libSimRobotEditor.so | ../Build/SimRobot/Linux/Release
	@echo "../Build/SimRobotEditor/Linux/Release/libSimRobotEditor.so (copy)"
	@cp ../Build/SimRobotEditor/Linux/Release/libSimRobotEditor.so ../Build/SimRobot/Linux/Release/libSimRobotEditor.so

../Build/SimRobot/Linux/Release/libSimRobotHelp.so: ../Build/SimRobotHelp/Linux/Release/libSimRobotHelp.so | ../Build/SimRobot/Linux/Release
	@echo "../Build/SimRobotHelp/Linux/Release/libSimRobotHelp.so (copy)"
	@cp ../Build/SimRobotHelp/Linux/Release/libSimRobotHelp.so ../Build/SimRobot/Linux/Release/libSimRobotHelp.so

../Build/SimRobot/Linux/Release/libFactory.so: ../Build/Factory/Linux/Release/libFactory.so | ../Build/SimRobot/Linux/Release
	@echo "../Build/Factory/Linux/Release/libFactory.so (copy)"
	@cp ../Build/Factory/Linux/Release/libFactory.so ../Build/SimRobot/Linux/Release/libFactory.so

../Build/SimRobot/Linux/Release/libSimpleVehicle.so: ../Build/SimpleVehicle/Linux/Release/libSimpleVehicle.so | ../Build/SimRobot/Linux/Release
	@echo "../Build/SimpleVehicle/Linux/Release/libSimpleVehicle.so (copy)"
	@cp ../Build/SimpleVehicle/Linux/Release/libSimpleVehicle.so ../Build/SimRobot/Linux/Release/libSimpleVehicle.so

../Build/SimRobot/Linux/Release/qrc_SimRobot.cpp: ../Src/SimRobot/SimRobot.qrc ../Src/SimRobot/Icons/application_side_tree.png ../Src/SimRobot/Icons/arrow_redo.png ../Src/SimRobot/Icons/arrow_undo.png ../Src/SimRobot/Icons/control_play_blue.png ../Src/SimRobot/Icons/control_start_blue.png ../Src/SimRobot/Icons/control_step_blue.png ../Src/SimRobot/Icons/cut.png ../Src/SimRobot/Icons/disk.png ../Src/SimRobot/Icons/folder.png ../Src/SimRobot/Icons/folder_page.png ../Src/SimRobot/Icons/page_copy.png ../Src/SimRobot/Icons/page_paste.png ../Src/SimRobot/Icons/SimRobot.png ../Src/SimRobot/Icons/tag_green.png ../Src/SimRobot/Icons/textfield.png | ../Build/SimRobot/Linux/Release
	@echo "../Src/SimRobot/SimRobot.qrc (Qt rcc)"
	@rcc -name SimRobot ../Src/SimRobot/SimRobot.qrc -o ../Build/SimRobot/Linux/Release/qrc_SimRobot.cpp

../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/qrc_SimRobot.o: ../Build/SimRobot/Linux/Release/qrc_SimRobot.cpp | ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release
	@echo "../Build/SimRobot/Linux/Release/qrc_SimRobot.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_MainWindow.o: ../Build/SimRobot/Linux/Release/moc_MainWindow.cpp | ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release
	@echo "../Build/SimRobot/Linux/Release/moc_MainWindow.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.o: ../Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.cpp | ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release
	@echo "../Build/SimRobot/Linux/Release/moc_RegisteredDockWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.o: ../Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.cpp | ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release
	@echo "../Build/SimRobot/Linux/Release/moc_SceneGraphDockWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release/moc_StatusBar.o: ../Build/SimRobot/Linux/Release/moc_StatusBar.cpp | ../Build/SimRobot/Linux/Release/Build/SimRobot/Linux/Release
	@echo "../Build/SimRobot/Linux/Release/moc_StatusBar.cpp"
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
