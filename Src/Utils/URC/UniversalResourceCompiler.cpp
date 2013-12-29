/**
* @file UniversalResourceCompiler.cpp
*
* Is used to compile the motion net for the special actions
* and for generating a variety of other files.
*
* @author Uwe Düffert
* @author Martin Lötzsch
*/

#include <cstdio>

#include "MofCompiler.h"

int main(int argc, char* argv[])
{
  char buffer[1000];
  if(compileMofs(buffer, sizeof(buffer)))
    printf("Created 'Config/specialActions.dat' successfully\n");
  else
    printf("%s", buffer);
}
