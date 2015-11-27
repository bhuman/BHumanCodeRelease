/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * Threading implementation private header file.                         *
 * Copyright (C) 2011-2012 Oleh Derevenko. All rights reserved.          *
 * e-mail: odar@eleks.com (change all "a" to "e")                        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*
 *  Threading implementation header for library private functions.
 */


#ifndef _ODE__PRIVATE_THREADING_IMPL_H_
#define _ODE__PRIVATE_THREADING_IMPL_H_


#include <ode/threading.h>


// This function has been removed from public headers as there is no need for it
// to be accessible to outer code at this time. In future it is possible 
// it could be published back again.
/**
 * @brief Allocates built-in self-threaded threading implementation object.
 *
 * A self-threaded implementation is a type of implementation that performs 
 * processing of posted calls by means of caller thread itself. This type of 
 * implementation does not need thread pool to serve it.
 * 
 * The processing is arranged in a way to prevent call stack depth growth 
 * as more and more nested calls are posted.
 *
 * @returns ID of object allocated or NULL on failure
 * 
 * @ingroup threading
 * @see dThreadingAllocateMultiThreadedImplementation
 * @see dThreadingFreeImplementation
 */
/*ODE_API */dThreadingImplementationID dThreadingAllocateSelfThreadedImplementation();



#endif // #ifndef _ODE__PRIVATE_THREADING_IMPL_H_
