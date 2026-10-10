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
 * @file LocatorWithMask.hpp
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_COMMON_LOCATORWITHMASK_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_COMMON_LOCATORWITHMASK_HPP_

#include <booster_fastdds/fastrtps/fastrtps_dll.h>

#include <booster_fastdds/fastdds/rtps/common/Locator.h>

namespace booster_eprosima {
namespace fastdds {
namespace rtps {

/**
 * A Locator with a mask that defines the number of significant bits of its address.
 */
class BOOSTER_RTPS_DllAPI LocatorWithMask : public Locator
{
public:

    /**
     * Get the number of significant bits on the address of this locator.
     *
     * @return number of significant bits on the address of this locator.
     */
    uint8_t mask() const
    {
        return mask_;
    }

    /**
     * Set the number of significant bits on the address of this locator.
     *
     * @param mask number of significant bits on the address of this locator.
     */
    void mask(
            uint8_t mask)
    {
        mask_ = mask;
    }

private:

    uint8_t mask_ = 24;
};

} // namespace rtps
} // namespace fastdds
} // namespace booster_eprosima

#endif /* BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_COMMON_LOCATORWITHMASK_HPP_ */
