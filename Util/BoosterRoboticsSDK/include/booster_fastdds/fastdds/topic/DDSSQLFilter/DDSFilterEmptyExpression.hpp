// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DDSFilterEmptyExpression.hpp
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_TOPIC_DDSSQLFILTER_DDSFILTEREMPTYEXPRESSION_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_TOPIC_DDSSQLFILTER_DDSFILTEREMPTYEXPRESSION_HPP_

#include <booster_fastdds/fastdds/dds/topic/IContentFilter.hpp>

namespace booster_eprosima {
namespace fastdds {
namespace dds {
namespace DDSSQLFilter {

/**
 * An IContentFilter for empty expressions that always evaluates to true.
 */
class DDSFilterEmptyExpression final : public IContentFilter
{

public:

    bool evaluate(
            const SerializedPayload& payload,
            const FilterSampleInfo& sample_info,
            const GUID_t& reader_guid) const final
    {
        static_cast<void>(payload);
        static_cast<void>(sample_info);
        static_cast<void>(reader_guid);

        return true;
    }

};

}  // namespace DDSSQLFilter
}  // namespace dds
}  // namespace fastdds
}  // namespace booster_eprosima

#endif  // BOOSTER_FASTDDS__BOOSTER_FASTDDS_TOPIC_DDSSQLFILTER_DDSFILTEREMPTYEXPRESSION_HPP_
