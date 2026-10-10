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

/*
   Expected defines.

   - BOOSTER_EPROSIMA_LIB_NAME
   - BOOSTER_FASTCDR_VERSION_MAJOR
   - BOOSTER_FASTCDR_VERSION_MINOR
 */

#if defined(_MSC_VER)
    #define BOOSTER_EPROSIMA_STRINGIZE(X) BOOSTER_EPROSIMA_DO_STRINGIZE(X)
    #define BOOSTER_EPROSIMA_DO_STRINGIZE(X) #X

    #if defined(_DEBUG)
        #define BOOSTER_EPROSIMA_LIB_DEBUG_TAG "d"
    #else
        #define BOOSTER_EPROSIMA_LIB_DEBUG_TAG
    #endif // _DEBUG

// Select linkage option.
    #if (defined(_DLL) || defined(_RTLDLL)) && defined(BOOSTER_EPROSIMA_DYN_LINK)
        #define BOOSTER_EPROSIMA_LIB_PREFIX
    #elif defined(BOOSTER_EPROSIMA_DYN_LINK)
        #error "Mixing a dll eprosima library with a static runtime is a bad idea"
    #else
        #define BOOSTER_EPROSIMA_LIB_PREFIX "lib"
    #endif // if (defined(_DLL) || defined(_RTLDLL)) && defined(BOOSTER_EPROSIMA_DYN_LINK)

// Include library
    #if defined(BOOSTER_EPROSIMA_LIB_NAME) \
    && defined(BOOSTER_EPROSIMA_LIB_PREFIX) \
    && defined(BOOSTER_EPROSIMA_LIB_DEBUG_TAG) \
    && defined(BOOSTER_FASTCDR_VERSION_MAJOR) \
    && defined(BOOSTER_FASTCDR_VERSION_MINOR)
        #pragma \
    comment(lib, BOOSTER_EPROSIMA_LIB_PREFIX BOOSTER_EPROSIMA_STRINGIZE(BOOSTER_EPROSIMA_LIB_NAME) BOOSTER_EPROSIMA_LIB_DEBUG_TAG "-" BOOSTER_EPROSIMA_STRINGIZE(BOOSTER_FASTCDR_VERSION_MAJOR) "." BOOSTER_EPROSIMA_STRINGIZE(BOOSTER_FASTCDR_VERSION_MINOR) ".lib")
    #else
    #error "Some required macros where not defined"
    #endif // if defined(BOOSTER_EPROSIMA_LIB_NAME) && defined(BOOSTER_EPROSIMA_LIB_PREFIX) && defined(BOOSTER_EPROSIMA_LIB_DEBUG_TAG) && defined(BOOSTER_FASTCDR_VERSION_MAJOR) && defined(BOOSTER_FASTCDR_VERSION_MINOR)

#endif // _MSC_VER

// Undef macros
#ifdef BOOSTER_EPROSIMA_LIB_PREFIX
#undef BOOSTER_EPROSIMA_LIB_PREFIX
#endif // ifdef BOOSTER_EPROSIMA_LIB_PREFIX

#ifdef BOOSTER_EPROSIMA_LIB_NAME
#undef BOOSTER_EPROSIMA_LIB_NAME
#endif // ifdef BOOSTER_EPROSIMA_LIB_NAME

#ifdef BOOSTER_EPROSIMA_LIB_DEBUG_TAG
#undef BOOSTER_EPROSIMA_LIB_DEBUG_TAG
#endif // ifdef BOOSTER_EPROSIMA_LIB_DEBUG_TAG
