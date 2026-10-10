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
 * @file ReaderDataLifecycleQosPolicy.hpp
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_READERDATALIFECYCLEQOSPOLICY_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_READERDATALIFECYCLEQOSPOLICY_HPP_

#include <booster_fastdds/fastdds/rtps/common/Time_t.h>

#include <booster_fastdds/fastdds/dds/core/policy/QosPolicies.hpp>


namespace booster_eprosima {
namespace fastdds {
namespace dds {

//! @brief Specifies the behavior of the DataReader with regards to the lifecycle of the data-instances it manages.
//! @warning This Qos Policy will be implemented in future releases.
//! @note Mutable Qos Policy
class ReaderDataLifecycleQosPolicy
{
public:

    /**
     * @brief Constructor
     */
    ReaderDataLifecycleQosPolicy()
        : autopurge_no_writer_samples_delay(BOOSTER_FASTDDS_TIME_T_INFINITE_SECONDS, BOOSTER_FASTDDS_TIME_T_INFINITE_NANOSECONDS)
        , autopurge_disposed_samples_delay(BOOSTER_FASTDDS_TIME_T_INFINITE_SECONDS, BOOSTER_FASTDDS_TIME_T_INFINITE_NANOSECONDS)
    {
    }

    /**
     * @brief Destructor
     */
    virtual BOOSTER_RTPS_DllAPI ~ReaderDataLifecycleQosPolicy()
    {
    }

    bool operator ==(
            const ReaderDataLifecycleQosPolicy& b) const
    {
        return (this->autopurge_no_writer_samples_delay == b.autopurge_no_writer_samples_delay) &&
               (this->autopurge_disposed_samples_delay == b.autopurge_disposed_samples_delay);
    }

    inline void clear()
    {
        ReaderDataLifecycleQosPolicy reset = ReaderDataLifecycleQosPolicy();
        std::swap(*this, reset);
    }

public:

    /**
     * @brief Indicates the duration the DataReader must retain information regarding instances that have the
     * instance_state NOT_ALIVE_NO_WRITERS. <br>
     * By default, c_TimeInfinite.
     */
    ::booster_eprosima::fastrtps::Duration_t autopurge_no_writer_samples_delay;
    /**
     * @brief Indicates the duration the DataReader must retain information regarding instances that have the
     * instance_state NOT_ALIVE_DISPOSED. <br>
     * By default, c_TimeInfinite.
     */
    ::booster_eprosima::fastrtps::Duration_t autopurge_disposed_samples_delay;
};

} //namespace dds
} //namespace fastdds
} //namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_READERDATALIFECYCLEQOSPOLICY_HPP_
