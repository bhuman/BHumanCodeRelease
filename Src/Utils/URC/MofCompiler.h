/**
* @file MofCompiler.h
*
* This file declares a single function to compile the motion net for special actions.
*
* @author Uwe Düffert
* @author Martin Lötzsch
*/

#pragma once

/**
* The function compiles all mofs.
* @param buffer A buffer that receives any error message output. It may contain
*               several lines separated by \n.
* @param size The length of the buffer.
* @return Success of compilation.
*/
bool compileMofs(char* buffer, size_t size);
