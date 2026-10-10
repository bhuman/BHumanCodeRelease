// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file System.h
 *
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_EPROSIMA_SYSTEM_UTILS_H
#define BOOSTER_FASTDDS__BOOSTER_EPROSIMA_SYSTEM_UTILS_H

#include "../fastrtps_dll.h"

namespace booster_eprosima {
namespace fastrtps {

/**
 * Class System, to provide helper functions to access system information.
 * @ingroup UTILITIES_MODULE
 */
class System
{
public:

    //! Returns current process identifier.
    BOOSTER_FASTDDS_DEPRECATED_UNTIL(3, "booster_eprosima::fastrtps::System::GetPID", "")
    BOOSTER_RTPS_DllAPI static int GetPID();
};

} /* namespace fastrtps */
} /* namespace booster_eprosima */

#endif /* BOOSTER_FASTDDS__BOOSTER_EPROSIMA_SYSTEM_UTILS_H */
