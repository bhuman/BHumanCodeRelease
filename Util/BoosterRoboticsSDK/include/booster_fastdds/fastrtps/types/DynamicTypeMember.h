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

#ifndef BOOSTER_FASTDDS_TYPES_DYNAMIC_TYPE_MEMBER_H
#define BOOSTER_FASTDDS_TYPES_DYNAMIC_TYPE_MEMBER_H

#include <booster_fastdds/fastrtps/types/TypesBase.h>
#include <booster_fastdds/fastrtps/types/MemberDescriptor.h>

namespace booster_eprosima {
namespace fastrtps {
namespace types {

class AnnotationDescriptor;
class DynamicType;

class DynamicTypeMember
{
protected:

    DynamicType* parent_;
    MemberDescriptor descriptor_;
    MemberId id_;

    uint32_t get_index() const;

    void set_index(
            uint32_t index);

    void set_parent(
            DynamicType* pType);

    friend class DynamicTypeBuilder;
    friend class DynamicType;
    friend class DynamicData;

public:

    BOOSTER_RTPS_DllAPI DynamicTypeMember();

    BOOSTER_RTPS_DllAPI DynamicTypeMember(
            const DynamicTypeMember* other);

    BOOSTER_RTPS_DllAPI DynamicTypeMember(
            const MemberDescriptor* descriptor,
            MemberId id);

    ~DynamicTypeMember();

    BOOSTER_RTPS_DllAPI ReturnCode_t apply_annotation(
            AnnotationDescriptor& descriptor);

    BOOSTER_RTPS_DllAPI ReturnCode_t apply_annotation(
            const std::string& annotation_name,
            const std::string& key,
            const std::string& value);

    BOOSTER_RTPS_DllAPI bool equals(
            const DynamicTypeMember*) const;

    BOOSTER_RTPS_DllAPI ReturnCode_t get_annotation(
            AnnotationDescriptor& descriptor,
            uint32_t idx);

    BOOSTER_RTPS_DllAPI uint32_t get_annotation_count();

    BOOSTER_RTPS_DllAPI bool key_annotation() const;

    BOOSTER_RTPS_DllAPI std::vector<uint64_t> get_union_labels() const;

    BOOSTER_RTPS_DllAPI ReturnCode_t get_descriptor(
            MemberDescriptor* descriptor) const;

    BOOSTER_RTPS_DllAPI MemberId get_id() const;

    BOOSTER_RTPS_DllAPI std::string get_name() const;

    BOOSTER_RTPS_DllAPI bool is_default_union_value() const;

    BOOSTER_RTPS_DllAPI const MemberDescriptor* get_descriptor() const
    {
        return &descriptor_;
    }

};

} // namespace types
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS_TYPES_DYNAMIC_TYPE_MEMBER_H
