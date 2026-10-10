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

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_TRANSPORT_DESCRIPTOR_INTERFACE_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_TRANSPORT_DESCRIPTOR_INTERFACE_H_

#include <cstdint>
#include <vector>

#include <booster_fastdds/fastrtps/fastrtps_dll.h>

namespace booster_eprosima {
namespace fastdds {
namespace rtps {

class TransportInterface;

/**
 * Virtual base class for the data type used to define transport configuration.
 * It acts as a builder for a given transport meaning that it allows to configure
 * the transport, and then a new Transport can be built according to this configuration
 * using its create_transport() factory member function.
 *
 * - maxMessageSize: maximum size of a single message in the transport.
 *
 * - maxInitialPeersRange: number of channels opened with each initial remote peer.
 *
 * @ingroup RTPS_MODULE
 * */
struct TransportDescriptorInterface
{
    //! Constructor
    BOOSTER_RTPS_DllAPI TransportDescriptorInterface(
            uint32_t maximumMessageSize,
            uint32_t maximumInitialPeersRange)
        : maxMessageSize(maximumMessageSize)
        , maxInitialPeersRange(maximumInitialPeersRange)
    {
    }

    //! Copy constructor
    BOOSTER_RTPS_DllAPI TransportDescriptorInterface(
            const TransportDescriptorInterface& t) = default;

    //! Copy assignment
    BOOSTER_RTPS_DllAPI TransportDescriptorInterface& operator =(
            const TransportDescriptorInterface& t) = default;

    //! Destructor
    virtual BOOSTER_RTPS_DllAPI ~TransportDescriptorInterface() = default;

    /**
     * Factory method pattern. It will create and return a TransportInterface
     * corresponding to this descriptor. This provides an interface to the NetworkFactory
     * to create the transports without the need to know about their type
     */
    virtual BOOSTER_RTPS_DllAPI TransportInterface* create_transport() const = 0;

    //! Returns the minimum size required for a send operation.
    virtual BOOSTER_RTPS_DllAPI uint32_t min_send_buffer_size() const = 0;

    //! Returns the maximum size expected for received messages.
    virtual BOOSTER_RTPS_DllAPI uint32_t max_message_size() const
    {
        return maxMessageSize;
    }

    /** Returns the maximum number of opened channels for each initial remote peer
     * (maximum number of guessed initial peers to try to connect)
     */
    virtual BOOSTER_RTPS_DllAPI uint32_t max_initial_peers_range() const
    {
        return maxInitialPeersRange;
    }

    //! Comparison operator
    BOOSTER_RTPS_DllAPI bool operator ==(
            const TransportDescriptorInterface& t) const
    {
        return (this->maxMessageSize == t.max_message_size() &&
               this->maxInitialPeersRange == t.max_initial_peers_range());
    }

    //! Maximum size of a single message in the transport
    uint32_t maxMessageSize;

    //! Number of channels opened with each initial remote peer.
    uint32_t maxInitialPeersRange;
};

} // namespace rtps
} // namespace fastdds
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_TRANSPORT_DESCRIPTOR_INTERFACE_H_
