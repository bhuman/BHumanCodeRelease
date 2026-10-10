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
 * @file rtps_all.h
 *
 */

#ifndef BOOSTER_FASTDDS_BOOSTER_FASTRTPS_ALL_H_
#define BOOSTER_FASTDDS_BOOSTER_FASTRTPS_ALL_H_

//USER THIS HEADER TO CREATE RAPID PROTOTYPES AND TESTS
//DO NOT INCLUDE IN PROJETCTS WERE COMPILATION TIME OR SIZE IS REVELANT
//SINCE IT INCLUDES ALL NECESSARY HEADERS.

#include <booster_fastdds/fastdds/rtps/common/all_common.h>

#include <booster_fastdds/fastrtps/Domain.h>

#include <booster_fastdds/fastrtps/participant/Participant.h>
#include <booster_fastdds/fastrtps/participant/ParticipantListener.h>
#include <booster_fastdds/fastrtps/publisher/Publisher.h>
#include <booster_fastdds/fastrtps/subscriber/Subscriber.h>
#include <booster_fastdds/fastrtps/publisher/PublisherListener.h>
#include <booster_fastdds/fastrtps/subscriber/SubscriberListener.h>


#include <booster_fastdds/fastrtps/attributes/ParticipantAttributes.h>
#include <booster_fastdds/fastrtps/attributes/PublisherAttributes.h>
#include <booster_fastdds/fastrtps/attributes/SubscriberAttributes.h>

#include <booster_fastdds/fastrtps/subscriber/SampleInfo.h>
#include <booster_fastdds/fastrtps/TopicDataType.h>

#include <booster_fastdds/fastrtps/utils/IPFinder.h>
#include <booster_fastdds/fastrtps/utils/TimeConversion.h>

#include <booster_fastdds/fastrtps/qos/QosPolicies.h>

#include <booster_fastdds/fastrtps/log/Log.h>


#endif /* BOOSTER_FASTDDS_BOOSTER_FASTRTPS_ALL_H_ */
