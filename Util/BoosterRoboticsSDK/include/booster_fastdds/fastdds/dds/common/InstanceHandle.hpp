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
 * @file InstanceHandle.hpp
 *
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_DDS_COMMON_INSTANCEHANDLE_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_DDS_COMMON_INSTANCEHANDLE_HPP_

#include <booster_fastdds/fastdds/rtps/common/InstanceHandle.h>
#include <booster_fastdds/fastrtps/fastrtps_dll.h>


namespace booster_eprosima {
namespace fastdds {
namespace dds {

using InstanceHandle_t = booster_eprosima::fastrtps::rtps::InstanceHandle_t;

extern BOOSTER_RTPS_DllAPI const InstanceHandle_t HANDLE_NIL;

} // namespace dds
} // namespace fastdds
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_DDS_COMMON_INSTANCEHANDLE_HPP_
