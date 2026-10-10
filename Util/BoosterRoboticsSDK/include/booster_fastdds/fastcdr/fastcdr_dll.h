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

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTCDR_BOOSTER_FASTCDR_DLL_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTCDR_BOOSTER_FASTCDR_DLL_H_

#include <booster_fastdds/fastcdr/config.h>

// normalize macros
#if !defined(BOOSTER_FASTCDR_DYN_LINK) && !defined(BOOSTER_FASTCDR_STATIC_LINK) \
    && !defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) && !defined(BOOSTER_EPROSIMA_ALL_STATIC_LINK)
#define BOOSTER_FASTCDR_STATIC_LINK
#endif // if !defined(BOOSTER_FASTCDR_DYN_LINK) && !defined(BOOSTER_FASTCDR_STATIC_LINK) && !defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) && !defined(BOOSTER_EPROSIMA_ALL_STATIC_LINK)

#if defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) && !defined(BOOSTER_FASTCDR_DYN_LINK)
#define BOOSTER_FASTCDR_DYN_LINK
#endif // if defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) && !defined(BOOSTER_FASTCDR_DYN_LINK)

#if defined(BOOSTER_FASTCDR_DYN_LINK) && defined(BOOSTER_FASTCDR_STATIC_LINK)
#error Must not define both BOOSTER_FASTCDR_DYN_LINK and BOOSTER_FASTCDR_STATIC_LINK
#endif // if defined(BOOSTER_FASTCDR_DYN_LINK) && defined(BOOSTER_FASTCDR_STATIC_LINK)

#if defined(BOOSTER_EPROSIMA_ALL_NO_LIB) && !defined(BOOSTER_FASTCDR_NO_LIB)
#define BOOSTER_FASTCDR_NO_LIB
#endif // if defined(BOOSTER_EPROSIMA_ALL_NO_LIB) && !defined(BOOSTER_FASTCDR_NO_LIB)

// enable dynamic linking

#if defined(_WIN32)
#if defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) || defined(BOOSTER_FASTCDR_DYN_LINK)
#if defined(fastcdr_EXPORTS)
#define BOOSTER_Cdr_DllAPI __declspec( dllexport )
#else
#define BOOSTER_Cdr_DllAPI __declspec( dllimport )
#endif // BOOSTER_FASTCDR_SOURCE
#else
#define BOOSTER_Cdr_DllAPI
#endif // if defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) || defined(BOOSTER_FASTCDR_DYN_LINK)
#else
#define BOOSTER_Cdr_DllAPI
#endif // _WIN32

// Auto linking.

#if !defined(BOOSTER_FASTCDR_SOURCE) && !defined(BOOSTER_EPROSIMA_ALL_NO_LIB) \
    && !defined(BOOSTER_FASTCDR_NO_LIB)

// Set properties.
#define BOOSTER_EPROSIMA_LIB_NAME fastcdr

#if defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) || defined(BOOSTER_FASTCDR_DYN_LINK)
#define BOOSTER_EPROSIMA_DYN_LINK
#endif // if defined(BOOSTER_EPROSIMA_ALL_DYN_LINK) || defined(BOOSTER_FASTCDR_DYN_LINK)

#include "eProsima_auto_link.h"
#endif // auto-linking disabled

#endif // BOOSTER_FASTDDS__BOOSTER_FASTCDR_BOOSTER_FASTCDR_DLL_H_
