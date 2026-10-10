// Copyright (C) 2015-2023 Jonathan Müller and foonathan/memory contributors
// SPDX-License-Identifier: Zlib

#ifndef BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_DEFAULT_ALLOCATOR_HPP_INCLUDED
#define BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_DEFAULT_ALLOCATOR_HPP_INCLUDED

/// \file
/// The typedef \ref booster_foonathan::memory::default_allocator.

#include "config.hpp"
#include "heap_allocator.hpp"
#include "new_allocator.hpp"
#include "static_allocator.hpp"
#include "virtual_memory.hpp"

#if BOOSTER_FOONATHAN_HOSTED_IMPLEMENTATION
#include "malloc_allocator.hpp"
#endif

namespace booster_foonathan
{
    namespace memory
    {
        /// The default \concept{concept_rawallocator,RawAllocator} that will be used as \concept{concept_blockallocator,BlockAllocator} in memory arenas.
        /// Arena allocators like \ref memory_stack or \ref memory_pool allocate memory by subdividing a huge block.
        /// They get a \concept{concept_blockallocator,BlockAllocator} that will be used for their internal allocation,
        /// this type is the default value.
        /// \requiredbe Its type can be changed via the CMake option \c BOOSTER_FOONATHAN_MEMORY_DEFAULT_ALLCOATOR,
        /// but it must be one of the following: \ref heap_allocator, \ref new_allocator, \ref malloc_allocator, \ref static_allocator, \ref virtual_memory_allocator.
        /// \defaultbe The default is \ref heap_allocator.
        /// \ingroup allocator
        using default_allocator = BOOSTER_FOONATHAN_IMPL_DEFINED(BOOSTER_FOONATHAN_MEMORY_IMPL_DEFAULT_ALLOCATOR);
    } // namespace memory
} // namespace booster_foonathan

#endif // BOOSTER_FASTDDS_BOOSTER_FOONATHAN_MEMORY_DEFAULT_ALLOCATOR_HPP_INCLUDED
