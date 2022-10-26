/**
 * @file Global.cpp
 * Implementation of a class that contains pointers to global data.
 * @author Thomas Röfer
 */

#include "Global.h"

thread_local AnnotationManager* Global::theAnnotationManager = nullptr;
thread_local OutMessage* Global::theDebugOut = nullptr;
thread_local Settings* Global::theSettings = nullptr;
thread_local DebugRequestTable* Global::theDebugRequestTable = nullptr;
thread_local DebugDataTable* Global::theDebugDataTable = nullptr;
thread_local DrawingManager* Global::theDrawingManager = nullptr;
thread_local DrawingManager3D* Global::theDrawingManager3D = nullptr;
thread_local TimingManager* Global::theTimingManager = nullptr;
thread_local asmjit::JitRuntime* Global::theAsmjitRuntime = nullptr;
