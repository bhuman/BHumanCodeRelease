// Copyright (C) 2015-2023 Jonathan Müller and foonathan/memory contributors
// SPDX-License-Identifier: Zlib

/// \file
/// Configuration macros.

#ifndef BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_CONFIG_HPP_INCLUDED
#define BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_CONFIG_HPP_INCLUDED

#include <cstddef>

#if !defined(DOXYGEN)
#define BOOSTER_FOONATHAN_MEMORY_IMPL_IN_CONFIG_HPP
#include "config_impl.hpp"
#undef BOOSTER_FOONATHAN_MEMORY_IMPL_IN_CONFIG_HPP
#endif

// exception support
#ifndef BOOSTER_FOONATHAN_HAS_EXCEPTION_SUPPORT
#if defined(__GNUC__) && !defined(__EXCEPTIONS)
#define BOOSTER_FOONATHAN_HAS_EXCEPTION_SUPPORT 0
#elif defined(_MSC_VER) && !_HAS_EXCEPTIONS
#define BOOSTER_FOONATHAN_HAS_EXCEPTION_SUPPORT 0
#else
#define BOOSTER_FOONATHAN_HAS_EXCEPTION_SUPPORT 1
#endif
#endif

#if BOOSTER_FOONATHAN_HAS_EXCEPTION_SUPPORT
#define BOOSTER_FOONATHAN_THROW(Ex) throw(Ex)
#else
#include <cstdlib>
#define BOOSTER_FOONATHAN_THROW(Ex) ((Ex), std::abort())
#endif

// hosted implementation
#ifndef BOOSTER_FOONATHAN_HOSTED_IMPLEMENTATION
#if !_MSC_VER && !__STDC_HOSTED__
#define BOOSTER_FOONATHAN_HOSTED_IMPLEMENTATION 0
#else
#define BOOSTER_FOONATHAN_HOSTED_IMPLEMENTATION 1
#endif
#endif

// log prefix
#define BOOSTER_FOONATHAN_MEMORY_LOG_PREFIX "booster_foonathan::memory"

// version
#define BOOSTER_FOONATHAN_MEMORY_VERSION                                                                   \
    (BOOSTER_FOONATHAN_MEMORY_VERSION_MAJOR * 100 + BOOSTER_FOONATHAN_MEMORY_VERSION_MINOR)

// use this macro to mark implementation-defined types
// gives it more semantics and useful with doxygen
// add PREDEFINED: BOOSTER_FOONATHAN_IMPL_DEFINED():=implementation_defined
#ifndef BOOSTER_FOONATHAN_IMPL_DEFINED
#define BOOSTER_FOONATHAN_IMPL_DEFINED(...) __VA_ARGS__
#endif

// use this macro to mark base class which only purpose is EBO
// gives it more semantics and useful with doxygen
// add PREDEFINED: BOOSTER_FOONATHAN_EBO():=
#ifndef BOOSTER_FOONATHAN_EBO
#define BOOSTER_FOONATHAN_EBO(...) __VA_ARGS__
#endif

#ifndef BOOSTER_FOONATHAN_ALIAS_TEMPLATE
// defines a template alias
// usage:
// template <typename T>
// BOOSTER_FOONATHAN_ALIAS_TEMPLATE(bar, foo<T, int>);
// useful for doxygen
#ifdef DOXYGEN
#define BOOSTER_FOONATHAN_ALIAS_TEMPLATE(Name, ...)                                                        \
    class Name : public __VA_ARGS__                                                                \
    {                                                                                              \
    }
#else
#define BOOSTER_FOONATHAN_ALIAS_TEMPLATE(Name, ...) using Name = __VA_ARGS__
#endif
#endif

#ifdef DOXYGEN
// dummy definitions of config macros for doxygen

/// The major version number.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_VERSION_MAJOR 1

/// The minor version number.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_VERSION_MINOR 1

/// The total version number of the form \c Mmm.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_VERSION                                                                   \
    (BOOSTER_FOONATHAN_MEMORY_VERSION_MAJOR * 100 + BOOSTER_FOONATHAN_MEMORY_VERSION_MINOR)

/// Whether or not the allocation size will be checked,
/// i.e. the \ref booster_foonathan::memory::bad_allocation_size thrown.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_CHECK_ALLOCATION_SIZE 1

/// Whether or not internal assertions in the library are enabled.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_DEBUG_ASSERT 1

/// Whether or not allocated memory will be filled with special values.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_DEBUG_FILL 1

/// The size of the fence memory, it has no effect if \ref BOOSTER_FOONATHAN_MEMORY_DEBUG_FILL is \c false.
/// \note For most allocators, the actual value doesn't matter and they use appropriate defaults to ensure alignment etc.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_DEBUG_FENCE 1

/// Whether or not leak checking is enabled.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_DEBUG_LEAK_CHECK 1

/// Whether or not the deallocation functions will check for pointers that were never allocated by an allocator.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_DEBUG_POINTER_CHECK 1

/// Whether or not the deallocation functions will check for double free errors.
/// This option makes no sense if \ref BOOSTER_FOONATHAN_MEMORY_DEBUG_POINTER_CHECK is \c false.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_DEBUG_DOUBLE_DEALLOC_CHECK 1

/// Whether or not everything is in namespace <tt>booster_foonathan::memory</tt>.
/// If \c false, a namespace alias <tt>namespace memory = booster_foonathan::memory</tt> is automatically inserted into each header,
/// allowing to qualify everything with <tt>booster_foonathan::</tt>.
/// \note This option breaks in combination with using <tt>using namespace booster_foonathan;</tt>.
/// \ingroup core
#define BOOSTER_FOONATHAN_MEMORY_NAMESPACE_PREFIX 1

/// The mode of the automatic \ref booster_foonathan::memory::temporary_stack creation.
/// Set to `2` to enable automatic lifetime management of the per-thread stack through nifty counter.
/// Then all memory will be freed upon program termination automatically.
/// Set to `1` to disable automatic lifetime managment of the per-thread stack,
/// requires managing it through the \ref booster_foonathan::memory::temporary_stack_initializer.
/// Set to `0` to disable the per-thread stack completely.
/// \ref booster_foonathan::memory::get_temporary_stack() will abort the program upon call.
/// \ingroup allocator
#define BOOSTER_FOONATHAN_MEMORY_TEMPORARY_STACK_MODE 2
#endif

#endif // BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_CONFIG_HPP_INCLUDED
