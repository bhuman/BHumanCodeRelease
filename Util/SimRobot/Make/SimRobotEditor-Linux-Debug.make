
.SUFFIXES:
.PHONY: SimRobotEditor

__SOFLAGS := -fpic
__CPPFLAGS := -std=c++11 -pipe -mssse3 -ffast-math -Wall -Wsign-compare -Wno-address -Wno-deprecated -Wno-overloaded-virtual -Wno-reorder -Wno-sign-conversion -Wno-strict-aliasing -Wno-switch -Wno-uninitialized -Wno-unused-parameter -g
__CFLAGS := -Wall -g
__DEFINES := -DLINUX -D_DEBUG -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL
__INCLUDEPATHS := -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt
__OBJECTS := ../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor/EditorModule.o ../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor/EditorWidget.o ../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor/SyntaxHighlighter.o ../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.o ../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.o ../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.o
__LINKFLAGS := 
__LIBPATHS := 
__LIBS := -lQtCore -lQtGui
__CXXLINKER := $(CXX)
__CCLINKER := $(CC)
__ARLINKER := $(AR)

SimRobotEditor: ../Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.cpp ../Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.cpp ../Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.cpp ../Build/SimRobotEditor/Linux/Debug/libSimRobotEditor.so

../Build/SimRobotEditor/Linux/Debug/libSimRobotEditor.so: $(__OBJECTS)  | ../Build/SimRobotEditor/Linux/Debug
	@echo "Linking SimRobotEditor..."
	@$(__CXXLINKER) -shared $(__SOFLAGS) -o $@ $(__OBJECTS) $(__LINKFLAGS) $(LDFLAGS) $(__LIBPATHS) $(__LIBS)

../Build/SimRobotEditor/Linux/Debug:
	@mkdir -p $@

../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor:
	@mkdir -p $@

../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug:
	@mkdir -p $@

../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor/EditorModule.o: ../Src/SimRobotEditor/EditorModule.cpp | ../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor
	@echo "../Src/SimRobotEditor/EditorModule.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor/EditorWidget.o: ../Src/SimRobotEditor/EditorWidget.cpp | ../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor
	@echo "../Src/SimRobotEditor/EditorWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor/SyntaxHighlighter.o: ../Src/SimRobotEditor/SyntaxHighlighter.cpp | ../Build/SimRobotEditor/Linux/Debug/Src/SimRobotEditor
	@echo "../Src/SimRobotEditor/SyntaxHighlighter.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.cpp: ../Src/SimRobotEditor/EditorWidget.h | ../Build/SimRobotEditor/Linux/Debug
	@echo "../Src/SimRobotEditor/EditorWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -D_DEBUG -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL  -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt ../Src/SimRobotEditor/EditorWidget.h -o ../Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.cpp

../Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.cpp: ../Src/SimRobotEditor/SyntaxHighlighter.h | ../Build/SimRobotEditor/Linux/Debug
	@echo "../Src/SimRobotEditor/SyntaxHighlighter.h (Qt moc)"
	@moc-qt4 -DLINUX -D_DEBUG -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_NO_STL  -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -I/usr/include/QtCore -I/usr/include/QtGui -I/usr/include/Qt ../Src/SimRobotEditor/SyntaxHighlighter.h -o ../Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.cpp

../Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.cpp: ../Src/SimRobotEditor/SimRobotEditor.qrc ../Src/SimRobotEditor/Icons/page_white_stack.png ../Src/SimRobotEditor/Icons/page_white_text.png | ../Build/SimRobotEditor/Linux/Debug
	@echo "../Src/SimRobotEditor/SimRobotEditor.qrc (Qt rcc)"
	@rcc -name SimRobotEditor ../Src/SimRobotEditor/SimRobotEditor.qrc -o ../Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.cpp

../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.o: ../Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.cpp | ../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug
	@echo "../Build/SimRobotEditor/Linux/Debug/qrc_SimRobotEditor.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.o: ../Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.cpp | ../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug
	@echo "../Build/SimRobotEditor/Linux/Debug/moc_EditorWidget.cpp"
	@$(CXX) -MMD $(__SOFLAGS) -o $@ -c $< $(__CPPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) $(__DEFINES) $(__INCLUDEPATHS)

../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.o: ../Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.cpp | ../Build/SimRobotEditor/Linux/Debug/Build/SimRobotEditor/Linux/Debug
	@echo "../Build/SimRobotEditor/Linux/Debug/moc_SyntaxHighlighter.cpp"
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
