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
 * @file fastdds/rtps/attributes/EndpointAttributes.h
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_ENDPOINTATTRIBUTES_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_ENDPOINTATTRIBUTES_H_

#include <booster_fastdds/fastdds/rtps/attributes/ExternalLocators.hpp>
#include <booster_fastdds/fastdds/rtps/attributes/PropertyPolicy.h>
#include <booster_fastdds/fastrtps/qos/QosPolicies.h>

#include <booster_fastdds/fastdds/rtps/common/Guid.h>
#include <booster_fastdds/fastdds/rtps/common/Locator.h>
#include <booster_fastdds/fastdds/rtps/common/Types.h>

#if BOOSTER_FASTDDS_HAVE_SECURITY
#include <booster_fastdds/fastdds/rtps/security/accesscontrol/EndpointSecurityAttributes.h>
#endif  // BOOSTER_FASTDDS_HAVE_SECURITY

namespace booster_eprosima {
namespace fastrtps {
namespace rtps {

/**
 * Structure EndpointAttributes, describing the attributes associated with an RTPS Endpoint.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class EndpointAttributes
{
public:

    //! Endpoint kind, default value WRITER
    EndpointKind_t endpointKind = EndpointKind_t::WRITER;

    //! Topic kind, default value NO_KEY
    TopicKind_t topicKind = TopicKind_t::NO_KEY;

    //! Reliability kind, default value BEST_EFFORT
    ReliabilityKind_t reliabilityKind = ReliabilityKind_t::BEST_EFFORT;

    //! Durability kind, default value VOLATILE
    DurabilityKind_t durabilityKind = DurabilityKind_t::VOLATILE;

    //! GUID used for persistence
    GUID_t persistence_guid;

    //! The collection of external locators to use for communication.
    fastdds::rtps::ExternalLocators external_unicast_locators;

    //! Whether locators that don't match with the announced locators should be kept.
    bool ignore_non_matching_locators = false;

    //! Unicast locator list
    LocatorList_t unicastLocatorList;

    //! Multicast locator list
    LocatorList_t multicastLocatorList;

    //! Remote locator list.
    LocatorList_t remoteLocatorList;

    //! Properties
    PropertyPolicy properties;

    //!Ownership
    OwnershipQosPolicyKind ownershipKind = SHARED_OWNERSHIP_QOS;

    EndpointAttributes()
    {
        datasharing_.off();
    }

    virtual ~EndpointAttributes() = default;

    /**
     * Get the user defined ID
     * @return User defined ID
     */
    inline int16_t getUserDefinedID() const
    {
        return m_userDefinedID;
    }

    /**
     * Get the entity defined ID
     * @return Entity ID
     */
    inline int16_t getEntityID() const
    {
        return m_entityID;
    }

    /**
     * Set the user defined ID
     * @param id User defined ID to be set
     */
    inline void setUserDefinedID(
            int16_t id)
    {
        m_userDefinedID = id;
    }

    /**
     * Set the entity ID
     * @param id Entity ID to be set
     */
    inline void setEntityID(
            int16_t id)
    {
        m_entityID = id;
    }

    /**
     * Set the DataSharing configuration
     * @param cfg Configuration to be set
     */
    inline void set_data_sharing_configuration(
            DataSharingQosPolicy cfg)
    {
        datasharing_ = cfg;
    }

    /**
     * Get the DataSharing configuration
     * @return Configuration of data sharing
     */
    inline const DataSharingQosPolicy& data_sharing_configuration() const
    {
        return datasharing_;
    }

#if BOOSTER_FASTDDS_HAVE_SECURITY
    const security::EndpointSecurityAttributes& security_attributes() const
    {
        return security_attributes_;
    }

    security::EndpointSecurityAttributes& security_attributes()
    {
        return security_attributes_;
    }

#endif // BOOSTER_FASTDDS_HAVE_SECURITY

private:

    //! User Defined ID, used for StaticEndpointDiscovery, default value -1.
    int16_t m_userDefinedID = -1;

    //! Entity ID, if the user want to specify the EntityID of the enpoint, default value -1.
    int16_t m_entityID = -1;

#if BOOSTER_FASTDDS_HAVE_SECURITY
    //! Security attributes
    security::EndpointSecurityAttributes security_attributes_;
#endif // BOOSTER_FASTDDS_HAVE_SECURITY

    //! Settings for datasharing
    DataSharingQosPolicy datasharing_;
};

} /* namespace rtps */
} /* namespace fastrtps */
} /* namespace booster_eprosima */

#endif /* BOOSTER_FASTDDS__BOOSTER_FASTDDS_ENDPOINTATTRIBUTES_H_ */
