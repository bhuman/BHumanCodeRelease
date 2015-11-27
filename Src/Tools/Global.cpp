/**
* @file Global.cpp
* Implementation of a class that contains pointers to global data.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Global.h"

PROCESS_LOCAL AnnotationManager* Global::theAnnotationManager = 0;
PROCESS_LOCAL OutMessage* Global::theDebugOut = 0;
PROCESS_LOCAL OutMessage* Global::theTeamOut = 0;
PROCESS_LOCAL Settings* Global::theSettings = 0;
PROCESS_LOCAL DebugRequestTable* Global::theDebugRequestTable = 0;
PROCESS_LOCAL DebugDataTable* Global::theDebugDataTable = 0;
PROCESS_LOCAL StreamHandler* Global::theStreamHandler = 0;
PROCESS_LOCAL DrawingManager* Global::theDrawingManager = 0;
PROCESS_LOCAL DrawingManager3D* Global::theDrawingManager3D = 0;
PROCESS_LOCAL TimingManager* Global::theTimingManager = 0;
