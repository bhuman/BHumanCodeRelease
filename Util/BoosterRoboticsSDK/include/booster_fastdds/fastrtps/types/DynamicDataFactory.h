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

#ifndef BOOSTER_FASTDDS_TYPES_DYNAMIC_DATA_FACTORY_H
#define BOOSTER_FASTDDS_TYPES_DYNAMIC_DATA_FACTORY_H

#include <booster_fastdds/fastrtps/types/TypesBase.h>
#include <booster_fastdds/fastrtps/types/DynamicTypePtr.h>
#include <booster_fastdds/fastrtps/types/DynamicTypeBuilder.h>
#include <booster_fastdds/fastrtps/types/DynamicType.h>
#include <booster_fastdds/fastrtps/types/DynamicData.h>
#include <mutex>

//#define DISABLE_DYNAMIC_MEMORY_CHECK

namespace booster_eprosima {
namespace fastrtps {
namespace types {

class DynamicDataFactory
{
protected:
    DynamicDataFactory();

    ReturnCode_t create_members(
            DynamicData* pData,
            DynamicType_ptr pType);

#ifndef DISABLE_DYNAMIC_MEMORY_CHECK
    std::vector<DynamicData*> dynamic_datas_;
    mutable std::recursive_mutex mutex_;
#endif

public:
    ~DynamicDataFactory();

    BOOSTER_RTPS_DllAPI static DynamicDataFactory* get_instance();

    BOOSTER_RTPS_DllAPI static ReturnCode_t delete_instance();

    BOOSTER_RTPS_DllAPI DynamicData* create_data(DynamicTypeBuilder* pBuilder);

    BOOSTER_RTPS_DllAPI DynamicData* create_data(DynamicType_ptr pType);

    BOOSTER_RTPS_DllAPI DynamicData* create_copy(const DynamicData* pData);

    BOOSTER_RTPS_DllAPI ReturnCode_t delete_data(DynamicData* pData);

    BOOSTER_RTPS_DllAPI bool is_empty() const;
};


} // namespace types
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS_TYPES_DYNAMIC_DATA_FACTORY_H
