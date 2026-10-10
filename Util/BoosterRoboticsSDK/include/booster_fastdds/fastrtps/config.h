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

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTRTPS_CONFIG_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTRTPS_CONFIG_H_

#define BOOSTER_FASTRTPS_VERSION_MAJOR 2
#define BOOSTER_FASTRTPS_VERSION_MINOR 13
#define BOOSTER_FASTRTPS_VERSION_MICRO 1
#define BOOSTER_FASTRTPS_VERSION_STR "2.13.1"

#define BOOSTER_FASTDDS_GEN_API_VER 2

// C++20 support defines
#ifndef BOOSTER_FASTDDS_HAVE_CXX20
#define BOOSTER_FASTDDS_HAVE_CXX20 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_CXX20 */

// C++17 support defines
#ifndef BOOSTER_FASTDDS_HAVE_CXX17
#define BOOSTER_FASTDDS_HAVE_CXX17 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_CXX17 */

// C++14 support defines
#ifndef BOOSTER_FASTDDS_HAVE_CXX14
#define BOOSTER_FASTDDS_HAVE_CXX14 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_CXX14 */

// C++1Y support defines
#ifndef BOOSTER_FASTDDS_HAVE_CXX1Y
#define BOOSTER_FASTDDS_HAVE_CXX1Y 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_CXX1Y */

// C++11 support defines
#ifndef BOOSTER_FASTDDS_HAVE_CXX11
#define BOOSTER_FASTDDS_HAVE_CXX11 1
#endif /* ifndef BOOSTER_FASTDDS_HAVE_CXX11 */

// Endianness defines
#ifndef BOOSTER_FASTDDS_IS_BIG_ENDIAN_TARGET
#define BOOSTER_FASTDDS_IS_BIG_ENDIAN_TARGET 0
#endif /* ifndef BOOSTER_FASTDDS_IS_BIG_ENDIAN_TARGET */

// Security
#ifndef BOOSTER_FASTDDS_HAVE_SECURITY
#define BOOSTER_FASTDDS_HAVE_SECURITY 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_SECURITY */

//Sqlite3 support
#ifndef BOOSTER_FASTDDS_HAVE_SQLITE3
#define BOOSTER_FASTDDS_HAVE_SQLITE3 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_SQLITE3 */

// Using thirdparty shared_mutex
#define BOOSTER_FASTDDS_USE_THIRDPARTY_SHARED_MUTEX 1

// TLS support
#ifndef BOOSTER_FASTDDS_TLS_FOUND
#define BOOSTER_FASTDDS_TLS_FOUND 0
#endif /* ifndef BOOSTER_FASTDDS_TLS_FOUND */

// Strict real-time
#ifndef BOOSTER_FASTDDS_HAVE_STRICT_REALTIME
#define BOOSTER_FASTDDS_HAVE_STRICT_REALTIME 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_STRICT_REALTIME */

/* Log Macros */

// Enable compilation for eProsima Log Macros
#ifndef BOOSTER_FASTDDS_ENABLE_OLD_LOG_MACROS_
#define BOOSTER_FASTDDS_ENABLE_OLD_LOG_MACROS_ 1
#endif /* ifndef BOOSTER_FASTDDS_ENABLE_OLD_LOG_MACROS_ */

// Log Info
#ifndef BOOSTER_FASTDDS_ENFORCE_LOG_INFO
/* #undef BOOSTER_FASTDDS_ENFORCE_LOG_INFO */
#endif
#ifndef BOOSTER_FASTDDS_HAVE_LOG_NO_INFO
#define BOOSTER_FASTDDS_HAVE_LOG_NO_INFO 1
#endif /* ifndef BOOSTER_FASTDDS_HAVE_LOG_NO_INFO */

// Log Warning
#ifndef BOOSTER_FASTDDS_HAVE_LOG_NO_WARNING
#define BOOSTER_FASTDDS_HAVE_LOG_NO_WARNING 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_LOG_NO_WARNING */

// Log Error
#ifndef BOOSTER_FASTDDS_HAVE_LOG_NO_ERROR
#define BOOSTER_FASTDDS_HAVE_LOG_NO_ERROR 0
#endif /* ifndef BOOSTER_FASTDDS_HAVE_LOG_NO_ERROR */

// Statistics
/* #undef BOOSTER_FASTDDS_STATISTICS */

// Deprecated macro
#if __cplusplus >= 201402L
#define BOOSTER_FASTRTPS_DEPRECATED(msg) [[ deprecated(msg) ]]
#elif defined(__GNUC__) || defined(__clang__)
#define BOOSTER_FASTRTPS_DEPRECATED(msg) __attribute__ ((deprecated(msg)))
#elif defined(_MSC_VER)
#define BOOSTER_FASTRTPS_DEPRECATED(msg) __declspec(deprecated(msg))
#else
#define BOOSTER_FASTRTPS_DEPRECATED(msg)
#endif /* if __cplusplus >= 201402L */

// Deprecation with version
#define BOOSTER_FASTDDS_DEPRECATED_UNTIL(major, entity_name, msg)                                                           \
    static_assert(BOOSTER_FASTRTPS_VERSION_MAJOR < major, #entity_name " reached deprecation version " #major);             \
    BOOSTER_FASTRTPS_DEPRECATED(#entity_name " has been deprecated and will be removed on major version " #major ". " msg)

#define BOOSTER_FASTDDS_TODO_BEFORE(major, minor, msg)                                          \
    static_assert((BOOSTER_FASTRTPS_VERSION_MAJOR < major) ||                                   \
            (BOOSTER_FASTRTPS_VERSION_MAJOR == major && BOOSTER_FASTRTPS_VERSION_MINOR < minor),  \
            "TODO before version " #major "." #minor " : " #msg);

#if BOOSTER_FASTCDR_VERSION_MAJOR > 1
#define BOOSTER_FASTDDS_SER_METHOD_DEPRECATED(major, entity_name, msg) BOOSTER_FASTDDS_DEPRECATED_UNTIL(major, entity_name, msg)
#else
#define BOOSTER_FASTDDS_SER_METHOD_DEPRECATED(major, entity_name, msg)
#endif

#endif // BOOSTER_FASTDDS__BOOSTER_FASTRTPS_CONFIG_H_
