// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file DataWriterQos.hpp
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_DDS_PUBLISHER_QOS_DATAWRITERQOS_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_DDS_PUBLISHER_QOS_DATAWRITERQOS_HPP_

#include <booster_fastdds/fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <booster_fastdds/fastrtps/fastrtps_dll.h>

namespace booster_eprosima {
namespace fastdds {
namespace statistics {
namespace dds {

/**
 * Class DataWriterQos: extends standard DDS DataWriterQos class to include specific default constructor for the
 * recommended DataWriterQos profile.
 * @ingroup STATISTICS_MODULE
 */
class DataWriterQos : public booster_eprosima::fastdds::dds::DataWriterQos
{
public:

    /**
     * @brief Constructor
     */
    BOOSTER_RTPS_DllAPI DataWriterQos();
};

//! Constant to access default Statistics DataWriter Qos
const booster_eprosima::fastdds::statistics::dds::DataWriterQos STATISTICS_DATAWRITER_QOS;

} // dds
} // statistics
} // fastdds
} // eprosima

#endif /* BOOSTER_FASTDDS__BOOSTER_FASTDDS_STATISTICS_DDS_PUBLISHER_QOS_DATAWRITERQOS_HPP_ */
