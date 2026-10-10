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
 * @file IConnectionsObserver.hpp
 *
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_MONITOR_SERVICE_INTERFACES_ICONNECTIONSOBSERVER_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_MONITOR_SERVICE_INTERFACES_ICONNECTIONSOBSERVER_HPP_

#include <vector>

#include <booster_fastdds/fastdds/rtps/common/Guid.h>

namespace booster_eprosima {
namespace fastdds {
namespace statistics {
namespace rtps {

using namespace booster_eprosima::fastdds::statistics;

class Connection;

struct IConnectionsObserver
{
    /**
     * @brief Interface use to notify any connection change
     * in an entity
     *
     * @param guid The GUID_t identifying the target entity
     * @return Whether the implementor has been properly notified
     */
    virtual bool on_local_entity_connections_change(
            const fastrtps::rtps::GUID_t& guid) const = 0;
};

} // rtps
} // statistics
} // fastdds
} // eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_MONITOR_SERVICE_INTERFACES_ICONNECTIONSOBSERVER_HPP_

