// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file RequestHeader.hpp
 *
 */

#ifndef BOOSTER_FASTDDS_REQUESTHEADER_HPP
#define BOOSTER_FASTDDS_REQUESTHEADER_HPP

#include <booster_fastdds/fastrtps/rtps/common/SampleIdentity.h>
#include <booster_fastdds/fastdds/dds/builtin/common/Types.hpp>

namespace booster_eprosima {

namespace fastcdr {
class Cdr;
} // fastcdr

namespace fastdds {
namespace dds {
namespace rpc {

struct RequestHeader
{
    booster_eprosima::fastrtps::rtps::SampleIdentity requestId;

    InstanceName instanceName;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
    BOOSTER_FASTDDS_SER_METHOD_DEPRECATED(3, "booster_eprosima::fastdds::dds::rpc::RequestHeader::getCdrSerializedSize()",
            "In favor of version using booster_eprosima::fastcdr::calculate_serialized_size.")
    BOOSTER_RTPS_DllAPI static size_t getCdrSerializedSize(
            const RequestHeader& data,
            size_t current_alignment = 0);

    BOOSTER_FASTDDS_SER_METHOD_DEPRECATED(3, "booster_eprosima::fastdds::dds::rpc::RequestHeader::serialize()",
            "In favor of version using booster_eprosima::fastcdr::serialize.")
    BOOSTER_RTPS_DllAPI void serialize(
            booster_eprosima::fastcdr::Cdr& cdr) const;

    BOOSTER_FASTDDS_SER_METHOD_DEPRECATED(3, "booster_eprosima::fastdds::dds::rpc::RequestHeader::deserialize()",
            "In favor of version using booster_eprosima::fastcdr::deserialize.")
    BOOSTER_RTPS_DllAPI void deserialize(
            booster_eprosima::fastcdr::Cdr& cdr);
#endif // DOXYGEN_SHOULD_SKIP_THIS

    BOOSTER_RTPS_DllAPI static bool isKeyDefined()
    {
        return false;
    }

};

} // namespace rpc
} // namespace dds
} // namespace fastdds
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS_REQUESTHEADER_HPP
