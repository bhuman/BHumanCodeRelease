
.SUFFIXES:
.PHONY: SimRobotHelp

__SOFLAGS := -fpic
__CPPFLAGS := -std=c++11 -pipe -mssse3 -ffast-math -Wall -Wsign-compare -Wno-address -Wno-deprecated -Wno-overloaded-virtual -Wno-reorder -Wno-sign-conversion -Wno-strict-aliasing -Wno-switch -Wno-uninitialized -Wno-unused-parameter -O3 -fomit-frame-pointer
__CFLAGS := -Wall -Os -fomit-frame-pointer
__DEFINES := -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_HELP_LIB -DQT_NO_STL -DQT_NO_DEBUG
__INCLUDEPATHS := -I../Src/SimRobotHelp -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtHelp -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/QtHelp -I/usr/include/Qt
__OBJECTS := ../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp/HelpModule.o ../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp/HelpWidget.o ../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.o ../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release/moc_HelpWidget.o
__LINKFLAGS := -s
__LIBPATHS := 
__LIBS := -lQtCore -lQtGui -lQtHelp
__CXXLINKER := $(CXX)
__CCLINKER := $(CC)
__ARLINKER := $(AR)

SimRobotHelp: ../Build/SimRobotHelp/Linux/Release/moc_HelpWidget.cpp ../Build/SimRobotHelp/Linux/Release/help.qch ../Build/SimRobotHelp/Linux/Release/helpcollection.qhc ../Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.cpp ../Build/SimRobotHelp/Linux/Release/libSimRobotHelp.so

../Build/SimRobotHelp/Linux/Release/libSimRobotHelp.so: $(__OBJECTS)  | ../Build/SimRobotHelp/Linux/Release
	@echo "Linking SimRobotHelp..."
	@$(__CXXLINKER) -shared $(__SOFLAGS) -o $@ $(__OBJECTS) $(__LINKFLAGS) $(LDFLAGS) $(__LIBPATHS) $(__LIBS)

../Build/SimRobotHelp/Linux/Release:
	@mkdir -p $@

../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp:
	@mkdir -p $@

../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release:
	@mkdir -p $@

../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp/HelpModule.o: ../Src/SimRobotHelp/HelpModule.cpp | ../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp
	@echo "../Src/SimRobotHelp/HelpModule.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp/HelpWidget.o: ../Src/SimRobotHelp/HelpWidget.cpp | ../Build/SimRobotHelp/Linux/Release/Src/SimRobotHelp
	@echo "../Src/SimRobotHelp/HelpWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotHelp/Linux/Release/moc_HelpWidget.cpp: ../Src/SimRobotHelp/HelpWidget.h | ../Build/SimRobotHelp/Linux/Release
	@echo "../Src/SimRobotHelp/HelpWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DNDEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_HELP_LIB -DQT_NO_STL -DQT_NO_DEBUG  -I../Src/SimRobotHelp -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtHelp -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/QtHelp -I/usr/include/Qt ../Src/SimRobotHelp/HelpWidget.h -o ../Build/SimRobotHelp/Linux/Release/moc_HelpWidget.cpp

../Build/SimRobotHelp/Linux/Release/help.qch: ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/index.html ../Src/SimRobotHelp/Help/api/api.html ../Src/SimRobotHelp/Help/basics/architecture.html ../Src/SimRobotHelp/Help/basics/basics.html ../Src/SimRobotHelp/Help/basics/physics.html ../Src/SimRobotHelp/Help/basics/quickstart.html ../Src/SimRobotHelp/Help/basics/sensing.html ../Src/SimRobotHelp/Help/basics/specification.html ../Src/SimRobotHelp/Help/console/commands.html ../Src/SimRobotHelp/Help/examples/examples.html ../Src/SimRobotHelp/Help/faq/faq.html ../Src/SimRobotHelp/Help/interface/console.html ../Src/SimRobotHelp/Help/interface/editmenu.html ../Src/SimRobotHelp/Help/interface/editorwindow.html ../Src/SimRobotHelp/Help/interface/filemenu.html ../Src/SimRobotHelp/Help/interface/helpmenu.html ../Src/SimRobotHelp/Help/interface/interface.html ../Src/SimRobotHelp/Help/interface/menus.html ../Src/SimRobotHelp/Help/interface/objectwindow.html ../Src/SimRobotHelp/Help/interface/sensorwindow.html ../Src/SimRobotHelp/Help/interface/simulationmenu.html ../Src/SimRobotHelp/Help/interface/statusbar.html ../Src/SimRobotHelp/Help/interface/toolbar.html ../Src/SimRobotHelp/Help/interface/treewindow.html ../Src/SimRobotHelp/Help/interface/viewmenu.html ../Src/SimRobotHelp/Help/interface/windowmenu.html ../Src/SimRobotHelp/Help/rosiml/rosi.html ../Src/SimRobotHelp/Help/rosiml/rosi_controller.html ../Src/SimRobotHelp/Help/simrobothelp.css ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Src/SimRobotHelp/Help/Icons/app_exit.bmp ../Src/SimRobotHelp/Help/Icons/app_exit_xp.bmp ../Src/SimRobotHelp/Help/Icons/crystal_toolbar.bmp ../Src/SimRobotHelp/Help/Icons/status_bar.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_camera.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_clock.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_color_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_column_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_copy.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_cut.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_drag_and_drop_plane.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_grid.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_help.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_light.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_line_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_monochrome_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_motion_blur.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_new.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_open.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_paste.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_reset.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_save.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_search.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_shader_interface.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_show_sensors.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_start.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_step.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_stereogram.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_surface_rendering.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_tree_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_update_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_view_item.bmp ../Src/SimRobotHelp/Help/Icons/architecture.png ../Src/SimRobotHelp/Help/Icons/SimRobot.png | ../Build/SimRobotHelp/Linux/Release
	@echo "../Src/SimRobotHelp/Help/help.qhp (qhelpgenerator)"
	@qhelpgenerator ../Src/SimRobotHelp/Help/help.qhp -o ../Build/SimRobotHelp/Linux/Release/help.qch

../Build/SimRobotHelp/Linux/Release/helpcollection.qhc: ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Build/SimRobotHelp/Linux/Release/help.qch | ../Build/SimRobotHelp/Linux/Release
	@echo "../Src/SimRobotHelp/Help/helpcollection.qhcp (qcollectiongenerator)"
	@bash -c "cp ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Build/SimRobotHelp/Linux/Release/helpcollection.qhcp && qcollectiongenerator ../Build/SimRobotHelp/Linux/Release/helpcollection.qhcp -o ../Build/SimRobotHelp/Linux/Release/helpcollection.qhc"

../Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.cpp: ../Src/SimRobotHelp/SimRobotHelp.qrc ../Src/SimRobotHelp/Icons/back.png ../Src/SimRobotHelp/Icons/forward.png ../Src/SimRobotHelp/Icons/help.png ../Src/SimRobotHelp/Icons/home.png ../Src/SimRobotHelp/Icons/locate.png | ../Build/SimRobotHelp/Linux/Release
	@echo "../Src/SimRobotHelp/SimRobotHelp.qrc (Qt rcc)"
	@rcc -name SimRobotHelp ../Src/SimRobotHelp/SimRobotHelp.qrc -o ../Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.cpp

../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.o: ../Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.cpp | ../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release
	@echo "../Build/SimRobotHelp/Linux/Release/qrc_SimRobotHelp.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release/moc_HelpWidget.o: ../Build/SimRobotHelp/Linux/Release/moc_HelpWidget.cpp | ../Build/SimRobotHelp/Linux/Release/Build/SimRobotHelp/Linux/Release
	@echo "../Build/SimRobotHelp/Linux/Release/moc_HelpWidget.cpp"
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
