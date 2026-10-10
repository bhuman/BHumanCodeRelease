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

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_TCPV6_TRANSPORT_DESCRIPTOR_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_TCPV6_TRANSPORT_DESCRIPTOR_

#include <booster_fastdds/fastdds/rtps/transport/TCPTransportDescriptor.h>

namespace booster_eprosima {
namespace fastdds {
namespace rtps {

/**
 * TCPv6 Transport configuration
 * The kind value for TCPv6TransportDescriptor is given by \c booster_eprosima::fastrtps::rtps::BOOSTER_FASTDDS_LOCATOR_KIND_TCPv6.
 *
 * @ingroup TRANSPORT_MODULE
 */
struct TCPv6TransportDescriptor : public TCPTransportDescriptor
{
    //! Destructor
    virtual ~TCPv6TransportDescriptor() = default;

    virtual TransportInterface* create_transport() const override;

    //! Constructor
    BOOSTER_RTPS_DllAPI TCPv6TransportDescriptor();

    //! Copy constructor
    BOOSTER_RTPS_DllAPI TCPv6TransportDescriptor(
            const TCPv6TransportDescriptor& t);

    //! Copy assignment
    BOOSTER_RTPS_DllAPI TCPv6TransportDescriptor& operator =(
            const TCPv6TransportDescriptor& t) = default;

    //! Comparison operator
    BOOSTER_RTPS_DllAPI bool operator ==(
            const TCPv6TransportDescriptor& t) const;
};

} // namespace rtps
} // namespace fastdds
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_TCPV6_TRANSPORT_DESCRIPTOR_
