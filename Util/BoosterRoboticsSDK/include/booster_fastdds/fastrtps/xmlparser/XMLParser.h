// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
//
#ifndef BOOSTER_FASTDDS_XML_PARSER_H_
#define BOOSTER_FASTDDS_XML_PARSER_H_

#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

#include <booster_fastdds/fastdds/dds/core/policy/QosPolicies.hpp>
#include <booster_fastdds/fastdds/dds/domain/qos/DomainParticipantFactoryQos.hpp>
#include <booster_fastdds/fastdds/rtps/attributes/ThreadSettings.hpp>
#include <booster_fastdds/fastdds/rtps/transport/PortBasedTransportDescriptor.hpp>
#include <booster_fastdds/fastdds/rtps/transport/SocketTransportDescriptor.h>
#include <booster_fastdds/fastrtps/attributes/LibrarySettingsAttributes.h>
#include <booster_fastdds/fastrtps/attributes/ParticipantAttributes.h>
#include <booster_fastdds/fastrtps/attributes/PublisherAttributes.h>
#include <booster_fastdds/fastrtps/attributes/ReplierAttributes.hpp>
#include <booster_fastdds/fastrtps/attributes/RequesterAttributes.hpp>
#include <booster_fastdds/fastrtps/attributes/SubscriberAttributes.h>
#include <booster_fastdds/fastrtps/transport/TransportDescriptorInterface.h>
#include <booster_fastdds/fastrtps/types/DynamicTypeBuilderPtr.h>
#include <booster_fastdds/fastrtps/xmlparser/XMLParserCommon.h>

namespace booster_tinyxml2 {
class XMLElement;
class XMLDocument;
} // namespace booster_tinyxml2

namespace booster_eprosima {
namespace fastrtps {
namespace xmlparser {

class BaseNode;
template <class T> class DataNode;

typedef std::unique_ptr<BaseNode>              up_base_node_t;
typedef std::vector<up_base_node_t>            up_base_node_vector_t;
typedef std::map<std::string, std::string>     node_att_map_t;
typedef node_att_map_t::iterator node_att_map_it_t;
typedef node_att_map_t::const_iterator node_att_map_cit_t;

typedef std::shared_ptr<fastdds::rtps::TransportDescriptorInterface> sp_transport_t;
typedef std::map<std::string, sp_transport_t>  sp_transport_map_t;
typedef types::DynamicTypeBuilder*             p_dynamictypebuilder_t;
typedef std::map<std::string, p_dynamictypebuilder_t> p_dynamictype_map_t;

typedef std::unique_ptr<fastdds::dds::DomainParticipantFactoryQos> up_participantfactory_t;
typedef DataNode<fastdds::dds::DomainParticipantFactoryQos>        node_participantfactory_t;
typedef node_participantfactory_t*                                 p_node_participantfactory_t;
typedef std::unique_ptr<node_participantfactory_t>                 up_node_participantfactory_t;

typedef std::unique_ptr<ParticipantAttributes> up_participant_t;
typedef DataNode<ParticipantAttributes>        node_participant_t;
typedef node_participant_t*                    p_node_participant_t;
typedef std::unique_ptr<node_participant_t>    up_node_participant_t;

typedef std::unique_ptr<PublisherAttributes>   up_publisher_t;
typedef DataNode<PublisherAttributes>          node_publisher_t;
typedef node_publisher_t*                      p_node_publisher_t;
typedef std::unique_ptr<node_publisher_t>      up_node_publisher_t;

typedef std::unique_ptr<SubscriberAttributes>  up_subscriber_t;
typedef DataNode<SubscriberAttributes>         node_subscriber_t;
typedef node_subscriber_t*                     p_node_subscriber_t;
typedef std::unique_ptr<node_subscriber_t>     up_node_subscriber_t;

typedef std::unique_ptr<TopicAttributes>       up_topic_t;
typedef DataNode<TopicAttributes>              node_topic_t;
typedef node_topic_t*                          p_node_topic_t;
typedef std::unique_ptr<node_topic_t>          up_node_topic_t;

typedef std::unique_ptr<RequesterAttributes>   up_requester_t;
typedef DataNode<RequesterAttributes>          node_requester_t;
typedef node_requester_t*                      p_node_requester_t;
typedef std::unique_ptr<node_requester_t>      up_node_requester_t;

typedef std::unique_ptr<ReplierAttributes>     up_replier_t;
typedef DataNode<ReplierAttributes>            node_replier_t;
typedef node_replier_t*                        p_node_replier_t;
typedef std::unique_ptr<node_replier_t>        up_node_replier_t;

/**
 * Class XMLParser, used to load XML data.
 * @ingroup XMLPARSER_MODULE
 */
class XMLParser
{

public:

    /**
     * Load the default XML file.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret loadDefaultXMLFile(
            up_base_node_t& root);

    /**
     * Load a XML file.
     * @param filename Name for the file to be loaded.
     * @param root Root node.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret loadXML(
            const std::string& filename,
            up_base_node_t& root);

    /**
     * Load a XML data from buffer.
     * @param data XML data to load.
     * @param length Length of the XML data.
     * @param root Root node.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret loadXML(
            const char* data,
            size_t length,
            up_base_node_t& root);

    /**
     * Load a XML node.
     * @param xmlDoc Node to be loaded.
     * @param root Root node.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret loadXML(
            booster_tinyxml2::XMLDocument& xmlDoc,
            up_base_node_t& root);

    /**
     * Load a XML node.
     * @param profiles Node to be loaded.
     * @param root Root node.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret loadXMLProfiles(
            booster_tinyxml2::XMLElement& profiles,
            up_base_node_t& root);

    /**
     * Load a XML node.
     * @param types Node to be loaded.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret loadXMLDynamicTypes(
            booster_tinyxml2::XMLElement& types);

protected:

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXML(
            booster_tinyxml2::XMLDocument& xmlDoc,
            up_base_node_t& root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLProfiles(
            booster_tinyxml2::XMLElement& profiles,
            up_base_node_t& root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseProfiles(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& profilesNode);


    /**
     * Load a XML log node and parses it. It applies the configuration of the node directly.
     * @param p_root Node to be loaded.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret parseLogConfig(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLLibrarySettings(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLTransportsProf(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLDomainParticipantFactoryProf(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLParticipantProf(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLPublisherProf(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLSubscriberProf(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLTopicData(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLRequesterProf(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLReplierProf(
            booster_tinyxml2::XMLElement* p_root,
            BaseNode& rootNode);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLTransportData(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret validateXMLTransportElements(
            booster_tinyxml2::XMLElement& p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLCommonTransportData(
            booster_tinyxml2::XMLElement* p_root,
            sp_transport_t p_transport);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLPortBasedTransportData(
            booster_tinyxml2::XMLElement* p_root,
            std::shared_ptr<fastdds::rtps::PortBasedTransportDescriptor> p_transport);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLSocketTransportData(
            booster_tinyxml2::XMLElement* p_root,
            std::shared_ptr<fastdds::rtps::SocketTransportDescriptor> p_transport);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLCommonTCPTransportData(
            booster_tinyxml2::XMLElement* p_root,
            sp_transport_t p_transport);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLCommonSharedMemTransportData(
            booster_tinyxml2::XMLElement* p_root,
            sp_transport_t p_transport);

    BOOSTER_RTPS_DllAPI static XMLP_ret parse_tls_config(
            booster_tinyxml2::XMLElement* p_root,
            sp_transport_t tcp_transport);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLReceptionThreads(
            booster_tinyxml2::XMLElement& p_root,
            fastdds::rtps::PortBasedTransportDescriptor::ReceptionThreadsConfigMap& reception_threads);

    /**
     * Load a XML consumer node and parses it. Adds the parsed consumer to Log directly.
     * @param consumer Node to be loaded.
     * @return XMLP_ret::XML_OK on success, XMLP_ret::XML_ERROR in other case.
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLConsumer(
            booster_tinyxml2::XMLElement& consumer);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLDynamicTypes(
            booster_tinyxml2::XMLElement& types);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseDynamicTypes(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLTypes(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLStructDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLUnionDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLEnumDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLAliasDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLBitsetDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLBitmaskDynamicType(
            booster_tinyxml2::XMLElement* p_root);

    BOOSTER_RTPS_DllAPI static p_dynamictypebuilder_t parseXMLBitfieldDynamicType(
            booster_tinyxml2::XMLElement* p_root,
            p_dynamictypebuilder_t p_dynamictype,
            types::MemberId mId,
            uint16_t& position);

    BOOSTER_RTPS_DllAPI static XMLP_ret parseXMLBitvalueDynamicType(
            booster_tinyxml2::XMLElement* p_root,
            p_dynamictypebuilder_t p_dynamictype,
            uint16_t& position);

    BOOSTER_RTPS_DllAPI static p_dynamictypebuilder_t parseXMLMemberDynamicType(
            booster_tinyxml2::XMLElement* p_root,
            p_dynamictypebuilder_t p_dynamictype,
            types::MemberId mId);

    BOOSTER_RTPS_DllAPI static p_dynamictypebuilder_t parseXMLMemberDynamicType(
            booster_tinyxml2::XMLElement* p_root,
            p_dynamictypebuilder_t p_dynamictype,
            types::MemberId mId,
            const std::string& values);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* p_profile,
            DataNode<fastdds::dds::DomainParticipantFactoryQos>& factory_node);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* p_profile,
            DataNode<ParticipantAttributes>& participant_node);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* p_profile,
            DataNode<PublisherAttributes>& publisher_node);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* p_profile,
            DataNode<SubscriberAttributes>& subscriber_node);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* node,
            DataNode<TopicAttributes>& topic_node);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* node,
            DataNode<RequesterAttributes>& requester_node);

    BOOSTER_RTPS_DllAPI static XMLP_ret fillDataNode(
            booster_tinyxml2::XMLElement* node,
            DataNode<ReplierAttributes>& replier_node);

    template <typename T>
    BOOSTER_RTPS_DllAPI static void addAllAttributes(
            booster_tinyxml2::XMLElement* p_profile,
            DataNode<T>& node);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLEnum(
            booster_tinyxml2::XMLElement* elem,
            fastrtps::IntraprocessDeliveryType* e,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLPropertiesPolicy(
            booster_tinyxml2::XMLElement* elem,
            rtps::PropertyPolicy& propertiesPolicy,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLHistoryMemoryPolicy(
            booster_tinyxml2::XMLElement* elem,
            rtps::MemoryManagementPolicy_t& historyMemoryPolicy,
            uint8_t ident);

    static XMLP_ret getXMLExternalLocatorList(
            booster_tinyxml2::XMLElement* elem,
            fastdds::rtps::ExternalLocators& external_locators,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLocatorList(
            booster_tinyxml2::XMLElement* elem,
            rtps::LocatorList_t& locatorList,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLocatorUDPv4(
            booster_tinyxml2::XMLElement* elem,
            rtps::Locator_t& locator,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLocatorUDPv6(
            booster_tinyxml2::XMLElement* elem,
            rtps::Locator_t& locator,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLocatorTCPv4(
            booster_tinyxml2::XMLElement* elem,
            rtps::Locator_t& locator,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLocatorTCPv6(
            booster_tinyxml2::XMLElement* elem,
            rtps::Locator_t& locator,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLWriterTimes(
            booster_tinyxml2::XMLElement* elem,
            rtps::WriterTimes& times,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLReaderTimes(
            booster_tinyxml2::XMLElement* elem,
            rtps::ReaderTimes& times,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDuration(
            booster_tinyxml2::XMLElement* elem,
            Duration_t& duration,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLWriterQosPolicies(
            booster_tinyxml2::XMLElement* elem,
            WriterQos& qos,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLReaderQosPolicies(
            booster_tinyxml2::XMLElement* elem,
            ReaderQos& qos,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLPublishModeQos(
            booster_tinyxml2::XMLElement* elem,
            PublishModeQosPolicy& publishMode,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLGroupDataQos(
            booster_tinyxml2::XMLElement* elem,
            GroupDataQosPolicy& groupData,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLTopicDataQos(
            booster_tinyxml2::XMLElement* elem,
            TopicDataQosPolicy& topicData,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLPartitionQos(
            booster_tinyxml2::XMLElement* elem,
            PartitionQosPolicy& partition,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLPresentationQos(
            booster_tinyxml2::XMLElement* elem,
            PresentationQosPolicy& presentation,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDestinationOrderQos(
            booster_tinyxml2::XMLElement* elem,
            DestinationOrderQosPolicy& destinationOrder,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLOwnershipStrengthQos(
            booster_tinyxml2::XMLElement* elem,
            OwnershipStrengthQosPolicy& ownershipStrength,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLOwnershipQos(
            booster_tinyxml2::XMLElement* elem,
            OwnershipQosPolicy& ownership,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLTimeBasedFilterQos(
            booster_tinyxml2::XMLElement* elem,
            TimeBasedFilterQosPolicy& timeBasedFilter,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLUserDataQos(
            booster_tinyxml2::XMLElement* elem,
            UserDataQosPolicy& userData,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLifespanQos(
            booster_tinyxml2::XMLElement* elem,
            LifespanQosPolicy& lifespan,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLReliabilityQos(
            booster_tinyxml2::XMLElement* elem,
            ReliabilityQosPolicy& reliability,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLivelinessQos(
            booster_tinyxml2::XMLElement* elem,
            LivelinessQosPolicy& liveliness,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLLatencyBudgetQos(
            booster_tinyxml2::XMLElement* elem,
            LatencyBudgetQosPolicy& latencyBudget,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDeadlineQos(
            booster_tinyxml2::XMLElement* elem,
            DeadlineQosPolicy& deadline,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDurabilityServiceQos(
            booster_tinyxml2::XMLElement* elem,
            DurabilityServiceQosPolicy& durabilityService,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDurabilityQos(
            booster_tinyxml2::XMLElement* elem,
            DurabilityQosPolicy& durability,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLTopicAttributes(
            booster_tinyxml2::XMLElement* elem,
            TopicAttributes& topic,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLHistoryQosPolicy(
            booster_tinyxml2::XMLElement* elem,
            HistoryQosPolicy& historyQos,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLResourceLimitsQos(
            booster_tinyxml2::XMLElement* elem,
            ResourceLimitsQosPolicy& resourceLimitsQos,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLContainerAllocationConfig(
            booster_tinyxml2::XMLElement* elem,
            ResourceLimitedContainerConfig& resourceLimitsQos,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLThroughputController(
            booster_tinyxml2::XMLElement* elem,
            rtps::ThroughputControllerDescriptor& throughputController,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLPortParameters(
            booster_tinyxml2::XMLElement* elem,
            rtps::PortParameters& port,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLParticipantAllocationAttributes(
            booster_tinyxml2::XMLElement* elem,
            rtps::RTPSParticipantAllocationAttributes& allocation,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLRemoteLocatorsAllocationAttributes(
            booster_tinyxml2::XMLElement* elem,
            rtps::RemoteLocatorsAllocationAttributes& allocation,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLSendBuffersAllocationAttributes(
            booster_tinyxml2::XMLElement* elem,
            rtps::SendBuffersAllocationAttributes& allocation,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDiscoverySettings(
            booster_tinyxml2::XMLElement* elem,
            rtps::DiscoverySettings& settings,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLTypeLookupSettings(
            booster_tinyxml2::XMLElement* elem,
            rtps::TypeLookupSettings& settings,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLInitialAnnouncementsConfig(
            booster_tinyxml2::XMLElement* elem,
            rtps::InitialAnnouncementConfig& config,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLBuiltinAttributes(
            booster_tinyxml2::XMLElement* elem,
            rtps::BuiltinAttributes& builtin,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLOctetVector(
            booster_tinyxml2::XMLElement* elem,
            std::vector<rtps::octet>& octet_vector,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLInt(
            booster_tinyxml2::XMLElement* elem,
            int* i,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLUint(
            booster_tinyxml2::XMLElement* elem,
            unsigned int* ui,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLUint(
            booster_tinyxml2::XMLElement* elem,
            uint16_t* ui16,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLUint(
            booster_tinyxml2::XMLElement* elem,
            uint64_t* ui64,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLBool(
            booster_tinyxml2::XMLElement* elem,
            bool* b,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLEnum(
            booster_tinyxml2::XMLElement* elem,
            rtps::DiscoveryProtocol_t* e,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLList(
            booster_tinyxml2::XMLElement* elem,
            booster_eprosima::fastdds::rtps::RemoteServerList_t& list,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLEnum(
            booster_tinyxml2::XMLElement* elem,
            rtps::ParticipantFilteringFlags_t* e,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLRemoteServer(
            booster_tinyxml2::XMLElement* elem,
            booster_eprosima::fastdds::rtps::RemoteServerAttributes& server,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLString(
            booster_tinyxml2::XMLElement* elem,
            std::string* s,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLTransports(
            booster_tinyxml2::XMLElement* elem,
            std::vector<std::shared_ptr<fastdds::rtps::TransportDescriptorInterface>>& transports,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDisablePositiveAcksQos(
            booster_tinyxml2::XMLElement* elem,
            DisablePositiveACKsQosPolicy& disablePositiveAcks,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDataSharingQos(
            booster_tinyxml2::XMLElement* elem,
            DataSharingQosPolicy& data_sharing,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLguidPrefix(
            booster_tinyxml2::XMLElement* elem,
            rtps::GuidPrefix_t& prefix,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLDomainParticipantFactoryQos(
            booster_tinyxml2::XMLElement& elem,
            fastdds::dds::DomainParticipantFactoryQos& qos);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLPublisherAttributes(
            booster_tinyxml2::XMLElement* elem,
            PublisherAttributes& publisher,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLSubscriberAttributes(
            booster_tinyxml2::XMLElement* elem,
            SubscriberAttributes& subscriber,
            uint8_t ident);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLThreadSettings(
            booster_tinyxml2::XMLElement& elem,
            fastdds::rtps::ThreadSettings& thread_setting);

    /*
        Return XMLP_ret::XML_OK when OK, XMLP_ret::XML_NOK when port attribute is not present, and
        XMLP_ret::XML_ERROR if error
     */
    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLThreadSettingsWithPort(
            booster_tinyxml2::XMLElement& elem,
            fastdds::rtps::ThreadSettings& thread_setting,
            uint32_t& port);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLEntityFactoryQos(
            booster_tinyxml2::XMLElement& elem,
            fastdds::dds::EntityFactoryQosPolicy& entity_factory);

    BOOSTER_RTPS_DllAPI static XMLP_ret getXMLBuiltinTransports(
            booster_tinyxml2::XMLElement* elem,
            booster_eprosima::fastdds::rtps::BuiltinTransports* bt,
            uint8_t ident);
};

} // namespace xmlparser
} // namespace fastrtps
} // namespace booster_eprosima

#endif // ifndef BOOSTER_FASTDDS_XML_PARSER_H_
