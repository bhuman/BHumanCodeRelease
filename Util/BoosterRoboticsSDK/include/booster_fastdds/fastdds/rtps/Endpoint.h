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
 * @file Endpoint.h
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_ENDPOINT_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_ENDPOINT_H_

#include <booster_fastdds/fastdds/rtps/attributes/EndpointAttributes.h>

#include <booster_fastdds/fastdds/rtps/common/Guid.h>
#include <booster_fastdds/fastdds/rtps/common/Locator.h>
#include <booster_fastdds/fastdds/rtps/common/Types.h>

#include <booster_fastdds/fastdds/rtps/history/IChangePool.h>
#include <booster_fastdds/fastdds/rtps/history/IPayloadPool.h>

#include <booster_fastdds/fastrtps/utils/TimedMutex.hpp>

namespace booster_eprosima {
namespace fastrtps {
namespace rtps {

class RTPSParticipantImpl;
class ResourceEvent;


/**
 * Class Endpoint, all entities of the RTPS network derive from this class.
 * Although the RTPSParticipant is also defined as an endpoint in the RTPS specification, in this implementation
 * the RTPSParticipant class **does not** inherit from the endpoint class. Each Endpoint object owns a pointer to the
 * RTPSParticipant it belongs to.
 * @ingroup COMMON_MODULE
 */
class Endpoint
{
    friend class RTPSParticipantImpl;

protected:

    Endpoint() = default;

    Endpoint(
            RTPSParticipantImpl* pimpl,
            const GUID_t& guid,
            const EndpointAttributes& att)
        : mp_RTPSParticipant(pimpl)
        , m_guid(guid)
        , m_att(att)
    {
    }

    virtual ~Endpoint()
    {
        // As releasing the change pool will delete the cache changes it owns,
        // the payload pool may be called to release their payloads, so we should
        // ensure that the payload pool is destroyed after the change pool.
        change_pool_.reset();
        payload_pool_.reset();
    }

public:

    /**
     * Get associated GUID
     * @return Associated GUID
     */
    BOOSTER_RTPS_DllAPI inline const GUID_t& getGuid() const
    {
        return m_guid;
    }

    /**
     * Get mutex
     * @return Associated Mutex
     */
    BOOSTER_RTPS_DllAPI inline RecursiveTimedMutex& getMutex()
    {
        return mp_mutex;
    }

    /**
     * Get associated attributes
     * @return Endpoint attributes
     */
    BOOSTER_RTPS_DllAPI inline EndpointAttributes& getAttributes()
    {
        return m_att;
    }

#if BOOSTER_FASTDDS_HAVE_SECURITY
    bool supports_rtps_protection()
    {
        return supports_rtps_protection_;
    }

#endif // if BOOSTER_FASTDDS_HAVE_SECURITY

protected:

    //!Pointer to the RTPSParticipant containing this endpoint.
    RTPSParticipantImpl* mp_RTPSParticipant;

    //!Endpoint GUID
    const GUID_t m_guid;

    //!Endpoint Attributes
    EndpointAttributes m_att;

    //!Endpoint Mutex
    mutable RecursiveTimedMutex mp_mutex;

    //!Pool of serialized payloads.
    std::shared_ptr<IPayloadPool> payload_pool_;

    //!Pool of cache changes.
    std::shared_ptr<IChangePool> change_pool_;

    //!Fixed size of payloads
    uint32_t fixed_payload_size_ = 0;

private:

    Endpoint& operator =(
            const Endpoint&) = delete;

#if BOOSTER_FASTDDS_HAVE_SECURITY
    bool supports_rtps_protection_ = true;
#endif // if BOOSTER_FASTDDS_HAVE_SECURITY
};


} // namespace rtps
} /* namespace rtps */
} /* namespace booster_eprosima */

#endif //BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_ENDPOINT_H_
