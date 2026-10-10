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
 * @file QosConverters.h
 */

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_UTILS_QOS_CONVERTERS_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_UTILS_QOS_CONVERTERS_HPP_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <booster_fastdds/fastdds/dds/domain/DomainParticipant.hpp>
#include <booster_fastdds/fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <booster_fastdds/fastdds/rtps/attributes/RTPSParticipantAttributes.h>
#include <booster_fastdds/fastrtps/attributes/PublisherAttributes.h>
#include <booster_fastdds/fastrtps/attributes/SubscriberAttributes.h>
#include <booster_fastdds/fastrtps/attributes/TopicAttributes.h>

namespace booster_eprosima {
namespace fastdds {
namespace dds {
namespace utils {

using fastrtps::PublisherAttributes;
using fastrtps::SubscriberAttributes;
using fastrtps::TopicAttributes;

/**
 * Obtains the DataWriterQos from the PublisherAttributes provided.
 *
 * @param[out] qos Pointer to the QoS to write on
 * @param[in] attr Pointer to the attributes from which to obtain data
 */
void set_qos_from_attributes(
        DataWriterQos& qos,
        const PublisherAttributes& attr);

/**
 * Obtains the DataReaderQos from the SubscriberAttributes provided.
 *
 * @param[out] qos Pointer to the QoS to write on
 * @param[in] attr Pointer to the attributes from which to obtain data
 */
void set_qos_from_attributes(
        DataReaderQos& qos,
        const SubscriberAttributes& attr);

/**
 * @brief Fill DomainParticipantQos from a given attributes RTPSParticipantAttributes object
 *
 * For the case of the non-binary properties, instead of the RTPSParticipantAttributes overriding the
 * property list in the DomainParticipantQos, a merge is performed in the following manner:
 *
 * - If any property from the RTPSParticipantAttributes is not in the DomainParticipantQos, then it is appended
 *   to the DomainParticipantQos.
 * - If any property from the RTPSParticipantAttributes property is also in the DomainParticipantQos, then the
 *   value in the DomainParticipantQos is overridden with that of the RTPSParticipantAttributes.
 *
 * @param[in, out] qos The DomainParticipantQos to set
 * @param[in] attr The RTPSParticipantAttributes from which the @c qos is set.
 */
void set_qos_from_attributes(
        DomainParticipantQos& qos,
        const booster_eprosima::fastrtps::rtps::RTPSParticipantAttributes& attr);

/**
 * Obtains the RTPSParticipantAttributes from the DomainParticipantQos provided.
 *
 * @param[out] attr Pointer to the attributes from which to obtain data
 * @param[in] qos Pointer to the QoS to write on
 */
void set_attributes_from_qos(
        fastrtps::rtps::RTPSParticipantAttributes& attr,
        const DomainParticipantQos& qos);

/**
 * Obtains the TopicQos from the TopicAttributes provided.
 *
 * @param[out] qos Pointer to the QoS to write on
 * @param[in] attr Pointer to the attributes from which to obtain data
 */
void set_qos_from_attributes(
        TopicQos& qos,
        const TopicAttributes& attr);

/**
 * Obtains the SubscriberQos from the SubscriberAttributes provided.
 *
 * @param[out] qos Pointer to the QoS to write on
 * @param[in] attr Pointer to the attributes from which to obtain data
 */
void set_qos_from_attributes(
        SubscriberQos& qos,
        const SubscriberAttributes& attr);

/**
 * Obtains the PublisherQos from the PublisherAttributes provided.
 *
 * @param[out] qos Pointer to the QoS to write on
 * @param[in] attr Pointer to the attributes from which to obtain data
 */
void set_qos_from_attributes(
        PublisherQos& qos,
        const PublisherAttributes& attr);

} /* namespace utils */
} /* namespace dds */
} /* namespace fastdds */
} /* namespace booster_eprosima */
#endif // ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#endif /* BOOSTER_FASTDDS__BOOSTER_FASTDDS_UTILS_QOS_CONVERTERS_HPP_ */
