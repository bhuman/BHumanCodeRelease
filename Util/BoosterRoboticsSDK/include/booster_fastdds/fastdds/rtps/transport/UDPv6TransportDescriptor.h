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

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_UDPV6_TRANSPORT_DESCRIPTOR_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_UDPV6_TRANSPORT_DESCRIPTOR_

#include <booster_fastdds/fastdds/rtps/transport/UDPTransportDescriptor.h>

namespace booster_eprosima {
namespace fastdds {
namespace rtps {

class TransportInterface;

/**
 * UDPv6 Transport configuration
 * The kind value for UDPv6TransportDescriptor is given by \c booster_eprosima::fastrtps::rtps::BOOSTER_FASTDDS_LOCATOR_KIND_UDPv6.
 *
 * @ingroup TRANSPORT_MODULE
 */
struct UDPv6TransportDescriptor : public UDPTransportDescriptor
{
    //! Destructor
    virtual ~UDPv6TransportDescriptor() = default;

    virtual TransportInterface* create_transport() const override;

    //! Constructor
    BOOSTER_RTPS_DllAPI UDPv6TransportDescriptor();

    //! Copy constructor
    BOOSTER_RTPS_DllAPI UDPv6TransportDescriptor(
            const UDPv6TransportDescriptor& t) = default;

    //! Copy assignment
    BOOSTER_RTPS_DllAPI UDPv6TransportDescriptor& operator =(
            const UDPv6TransportDescriptor& t) = default;

    BOOSTER_RTPS_DllAPI bool operator ==(
            const UDPv6TransportDescriptor& t) const;
};

} // namespace rtps
} // namespace fastdds
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_UDPV6_TRANSPORT_DESCRIPTOR_
