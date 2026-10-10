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
 * @file IStatusQueryable.hpp
 *
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_MONITOR_SERVICE_INTERFACES_ISTATUSQUERYABLE_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_MONITOR_SERVICE_INTERFACES_ISTATUSQUERYABLE_HPP_

#include <booster_fastdds/fastdds/dds/core/status/BaseStatus.hpp>
#include <booster_fastdds/fastdds/dds/core/status/DeadlineMissedStatus.hpp>
#include <booster_fastdds/fastdds/dds/core/status/IncompatibleQosStatus.hpp>
#include <booster_fastdds/fastdds/dds/core/status/LivelinessChangedStatus.hpp>
#include <booster_fastdds/fastdds/rtps/common/Guid.h>

namespace booster_eprosima {
namespace fastdds {
namespace statistics {
namespace rtps {

struct DDSEntityStatus : public booster_eprosima::fastdds::dds::IncompatibleQosStatus,
    public booster_eprosima::fastdds::dds::BaseStatus,
    public booster_eprosima::fastdds::dds::LivelinessChangedStatus,
    public booster_eprosima::fastdds::dds::DeadlineMissedStatus
{

};

struct IStatusQueryable
{
    /**
     * @brief Interface for requesting the IncompatibleQosStatus
     * of and entity identified by its guid.
     *
     * @param[in] guid The GUID_t identifying the target entity
     * @param[in] status_kind The monitor service status kind that has changed
     * @param[out] status The requested entity status
     * @return Whether the operation succeeded or not
     */
    virtual bool get_monitoring_status(
            const fastrtps::rtps::GUID_t& guid,
            const uint32_t& status_kind,
            DDSEntityStatus*& status) = 0;

};

} // rtps
} // statistics
} // fastdds
} // eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_MONITOR_SERVICE_INTERFACES_ISTATUSQUERYABLE_HPP_

