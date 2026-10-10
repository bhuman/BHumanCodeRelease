// Copyright (C) 2015-2023 Jonathan Müller and foonathan/memory contributors
// SPDX-License-Identifier: Zlib

#ifndef BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_DETAIL_ASSERT_HPP_INCLUDED
#define BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_DETAIL_ASSERT_HPP_INCLUDED

#include <cstdlib>

#include "../config.hpp"

namespace booster_foonathan
{
    namespace memory
    {
        namespace detail
        {
            // handles a failed assertion
            void handle_failed_assert(const char* msg, const char* file, int line,
                                      const char* fnc) noexcept;

            void handle_warning(const char* msg, const char* file, int line,
                                const char* fnc) noexcept;

// note: debug assertion macros don't use fully qualified name
// because they should only be used in this library, where the whole namespace is available
// can be override via command line definitions
#if BOOSTER_FOONATHAN_MEMORY_DEBUG_ASSERT && !defined(BOOSTER_FOONATHAN_MEMORY_ASSERT)
#define BOOSTER_FOONATHAN_MEMORY_ASSERT(Expr)                                                              \
    static_cast<void>((Expr)                                                                       \
                      || (detail::handle_failed_assert("Assertion \"" #Expr "\" failed", __FILE__, \
                                                       __LINE__, __func__),                        \
                          true))

#define BOOSTER_FOONATHAN_MEMORY_ASSERT_MSG(Expr, Msg)                                                     \
    static_cast<void>((Expr)                                                                       \
                      || (detail::handle_failed_assert("Assertion \"" #Expr "\" failed: " Msg,     \
                                                       __FILE__, __LINE__, __func__),              \
                          true))

#define BOOSTER_FOONATHAN_MEMORY_UNREACHABLE(Msg)                                                          \
    detail::handle_failed_assert("Unreachable code reached: " Msg, __FILE__, __LINE__, __func__)

#define BOOSTER_FOONATHAN_MEMORY_WARNING(Msg) detail::handle_warning(Msg, __FILE__, __LINE__, __func__)

#elif !defined(BOOSTER_FOONATHAN_MEMORY_ASSERT)
#define BOOSTER_FOONATHAN_MEMORY_ASSERT(Expr)
#define BOOSTER_FOONATHAN_MEMORY_ASSERT_MSG(Expr, Msg)
#define BOOSTER_FOONATHAN_MEMORY_UNREACHABLE(Msg) std::abort()
#define BOOSTER_FOONATHAN_MEMORY_WARNING(Msg)
#endif
        } // namespace detail
    }     // namespace memory
} // namespace booster_foonathan

#endif // BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_DETAIL_ASSERT_HPP_INCLUDED

