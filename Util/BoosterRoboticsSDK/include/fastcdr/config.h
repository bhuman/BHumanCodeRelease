// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _FASTCDR_CONFIG_H_
#define _FASTCDR_CONFIG_H_

#define FASTCDR_VERSION_MAJOR 2
#define FASTCDR_VERSION_MINOR 1
#define FASTCDR_VERSION_MICRO 3
#define FASTCDR_VERSION_STR "2.1.3"

// C++11 support defines
#ifndef HAVE_CXX11
#define HAVE_CXX11 1
#endif // ifndef HAVE_CXX11

// Endianness defines
#ifndef FASTCDR_IS_BIG_ENDIAN_TARGET
#define FASTCDR_IS_BIG_ENDIAN_TARGET 0
#endif // ifndef FASTCDR_IS_BIG_ENDIAN_TARGET

#ifndef FASTCDR_HAVE_FLOAT128
#define FASTCDR_HAVE_FLOAT128 1
#endif // ifndef FASTCDR_HAVE_FLOAT128

#ifndef FASTCDR_SIZEOF_LONG_DOUBLE
#define FASTCDR_SIZEOF_LONG_DOUBLE 16
#endif // ifndef FASTCDR_SIZEOF_LONG_DOUBLE

#if defined(__ARM_ARCH) && __ARM_ARCH <= 7
#define FASTCDR_ARM32
#endif // if defined(__ARM_ARCH) && __ARM_ARCH <= 7

#if defined(__GNUC__) && !defined(__clang__)
#define TEMPLATE_SPEC
#else
#define TEMPLATE_SPEC template<>
#endif // if defined(__GNUC__) && !defined(__clang__)

#endif // _FASTCDR_CONFIG_H_
