// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file ReaderQos.h
 *
 */

#ifndef BOOSTER_FASTDDS_READERQOS_H_
#define BOOSTER_FASTDDS_READERQOS_H_

#include <booster_fastdds/fastdds/dds/subscriber/qos/ReaderQos.hpp>

#include <booster_fastdds/fastrtps/qos/QosPolicies.h>  // Needed for old enum constant values

namespace booster_eprosima {
namespace fastrtps {

using ReaderQos = fastdds::dds::ReaderQos;

} /* namespace  */
} /* namespace booster_eprosima */

#endif /* BOOSTER_FASTDDS_READERQOS_H_ */
