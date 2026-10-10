// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

/**
 * @file CdrSerialization.hpp
 */

#ifndef BOOSTER_FASTDDS_BOOSTER_FASTDDS_RTPS_COMMON_CDRSERIALIZATION_HPP
#define BOOSTER_FASTDDS_BOOSTER_FASTDDS_RTPS_COMMON_CDRSERIALIZATION_HPP

#include <booster_fastdds/fastcdr/Cdr.h>

#if BOOSTER_FASTCDR_VERSION_MAJOR == 1

#include <booster_fastdds/fastcdr/exceptions/BadParamException.h>
#include <booster_fastdds/fastrtps/utils/fixed_size_string.hpp>

namespace booster_eprosima {
namespace fastcdr {

namespace CdrVersion {
const booster_eprosima::fastcdr::Cdr::CdrType DDS_CDR = booster_eprosima::fastcdr::Cdr::CdrType::DDS_CDR;
const booster_eprosima::fastcdr::Cdr::CdrType XCDRv1 = booster_eprosima::fastcdr::Cdr::CdrType::DDS_CDR;
const booster_eprosima::fastcdr::Cdr::CdrType XCDRv2 = booster_eprosima::fastcdr::Cdr::CdrType::DDS_CDR;
} // namespace CdrVersion

class CdrSizeCalculator;

template<class _T>
extern size_t calculate_serialized_size(
        booster_eprosima::fastcdr::CdrSizeCalculator& calculator,
        const _T& data,
        size_t& current_alignment);

template<class _T>
extern void serialize(
        Cdr&,
        const _T&);

template<class _T>
extern void deserialize(
        Cdr&,
        _T&);

// Dummy class
class CdrSizeCalculator
{
public:

    CdrSizeCalculator(
            booster_eprosima::fastcdr::Cdr::CdrType)
    {
    }

    template<class _T>
    size_t calculate_serialized_size(
            const _T& data,
            size_t current_alignment)
    {
        return booster_eprosima::fastcdr::calculate_serialized_size(*this, data, current_alignment);
    }

};

template<size_t MAX_CHARS>
using fixed_string = booster_eprosima::fastrtps::fixed_string<MAX_CHARS>;

} // namespace fastcdr
} // namespace booster_eprosima

namespace booster_eprosima {
namespace fastdds {
namespace rtps {
//! Default XCDR encoding version used in Fast DDS.
constexpr booster_eprosima::fastcdr::Cdr::CdrType DEFAULT_XCDR_VERSION {booster_eprosima::fastcdr::Cdr::CdrType::DDS_CDR};
} // namespace rtps
} // namespace fastdds
} // namespace booster_eprosima

#else

#include <booster_fastdds/fastcdr/CdrSizeCalculator.hpp>

namespace booster_eprosima {
namespace fastdds {
namespace rtps {
//! Default XCDR encoding version used in Fast DDS.
constexpr booster_eprosima::fastcdr::CdrVersion DEFAULT_XCDR_VERSION {booster_eprosima::fastcdr::CdrVersion::XCDRv1};
} // namespace rtps
} // namespace fastdds
} // namespace booster_eprosima

#endif //BOOSTER_FASTCDR_VERSION_MAJOR == 1

#endif // BOOSTER_FASTDDS_BOOSTER_FASTDDS_RTPS_COMMON_CDRSERIALIZATION_HPP
