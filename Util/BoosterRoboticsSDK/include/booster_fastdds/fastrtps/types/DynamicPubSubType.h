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

#ifndef BOOSTER_FASTDDS_TYPES_DYNAMIC_PUB_SUB_TYPE_H
#define BOOSTER_FASTDDS_TYPES_DYNAMIC_PUB_SUB_TYPE_H

#include <booster_fastdds/fastrtps/types/TypesBase.h>
#include <booster_fastdds/fastdds/dds/topic/TopicDataType.hpp>
#include <booster_fastdds/fastrtps/types/DynamicTypePtr.h>
#include <booster_fastdds/fastrtps/types/DynamicDataPtr.h>
#include <booster_fastdds/fastrtps/utils/md5.h>

namespace booster_eprosima {
namespace fastrtps {
namespace types {

class DynamicPubSubType : public booster_eprosima::fastdds::dds::TopicDataType
{
protected:

    void UpdateDynamicTypeInfo();

    DynamicType_ptr dynamic_type_;
    BoosterFastddsMD5 m_md5;
    unsigned char* m_keyBuffer;

    enum
    {
        FINAL,
        APPENDABLE,
        MUTABLE
    }
    extensibility_ {APPENDABLE};

public:

    BOOSTER_RTPS_DllAPI DynamicPubSubType();

    BOOSTER_RTPS_DllAPI DynamicPubSubType(
            DynamicType_ptr pDynamicType);

    BOOSTER_RTPS_DllAPI virtual ~DynamicPubSubType();

    BOOSTER_RTPS_DllAPI void* createData() override;

    BOOSTER_RTPS_DllAPI void deleteData (
            void* data) override;

    BOOSTER_RTPS_DllAPI bool deserialize (
            booster_eprosima::fastrtps::rtps::SerializedPayload_t* payload,
            void* data) override;

    BOOSTER_RTPS_DllAPI bool getKey(
            void* data,
            booster_eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
            bool force_md5 = false) override;

    BOOSTER_RTPS_DllAPI std::function<uint32_t()> getSerializedSizeProvider(
            void* data) override
    {
        return getSerializedSizeProvider(data, fastdds::dds::DEFAULT_DATA_REPRESENTATION);
    }

    BOOSTER_RTPS_DllAPI std::function<uint32_t()> getSerializedSizeProvider(
            void* data,
            fastdds::dds::DataRepresentationId_t data_representation) override;

    BOOSTER_RTPS_DllAPI bool serialize(
            void* data,
            booster_eprosima::fastrtps::rtps::SerializedPayload_t* payload) override
    {
        return serialize(data, payload, fastdds::dds::DEFAULT_DATA_REPRESENTATION);
    }

    BOOSTER_RTPS_DllAPI bool serialize(
            void* data,
            booster_eprosima::fastrtps::rtps::SerializedPayload_t* payload,
            fastdds::dds::DataRepresentationId_t data_representation) override;

    BOOSTER_RTPS_DllAPI void CleanDynamicType();

    BOOSTER_RTPS_DllAPI DynamicType_ptr GetDynamicType() const;

    BOOSTER_RTPS_DllAPI ReturnCode_t SetDynamicType(
            DynamicData_ptr pData);

    BOOSTER_RTPS_DllAPI ReturnCode_t SetDynamicType(
            DynamicType_ptr pType);
};

} // namespace types
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS_TYPES_DYNAMIC_PUB_SUB_TYPE_H
