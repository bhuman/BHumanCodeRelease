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

#ifndef BOOSTER_FASTDDS_TYPES_MEMBER_DESCRIPTOR_H
#define BOOSTER_FASTDDS_TYPES_MEMBER_DESCRIPTOR_H

#include <booster_fastdds/fastrtps/types/TypesBase.h>
#include <booster_fastdds/fastrtps/types/DynamicTypePtr.h>

namespace booster_eprosima{
namespace fastrtps{
namespace types{

class DynamicType;
class AnnotationDescriptor;

class MemberDescriptor
{
protected:
    std::string name_;                  // Name of the member
    MemberId id_;                       // MemberId, it should be filled automatically when the member is added.
    DynamicType_ptr type_;              // Member's Type.
    std::string default_value_;         // Default value of the member in string.
    uint32_t index_;                    // Definition order of the member inside it's parent.
    std::vector<uint64_t> labels_;      // Case Labels for unions.
    bool default_label_;                // TRUE if it's the default option of a union.

    std::vector<AnnotationDescriptor*> annotation_; // Annotations to apply

    friend class DynamicTypeBuilderFactory;
    friend class DynamicData;
    friend class DynamicTypeMember;
    friend class TypeObjectFactory;

    bool is_default_value_consistent(const std::string& sDefaultValue) const;

    bool is_type_name_consistent(const std::string& sName) const;

public:
    BOOSTER_RTPS_DllAPI MemberDescriptor();

    BOOSTER_RTPS_DllAPI MemberDescriptor(
            uint32_t index,
            const std::string& name);

    BOOSTER_RTPS_DllAPI MemberDescriptor(
            MemberId id,
            const std::string& name,
            DynamicType_ptr type_);

    BOOSTER_RTPS_DllAPI MemberDescriptor(
            MemberId id,
            const std::string& name,
            DynamicType_ptr type_,
            const std::string& defaultValue);

    BOOSTER_RTPS_DllAPI MemberDescriptor(
            MemberId id,
            const std::string& name,
            DynamicType_ptr type_,
            const std::string& defaultValue,
            const std::vector<uint64_t>& unionLabels,
            bool isDefaultLabel);

    BOOSTER_RTPS_DllAPI MemberDescriptor(const MemberDescriptor* descriptor);

    BOOSTER_RTPS_DllAPI ~MemberDescriptor();

    bool check_union_labels(const std::vector<uint64_t>& labels) const;

    BOOSTER_RTPS_DllAPI ReturnCode_t copy_from(const MemberDescriptor* other);

    BOOSTER_RTPS_DllAPI bool equals(const MemberDescriptor* other) const;

    BOOSTER_RTPS_DllAPI TypeKind get_kind() const;

    BOOSTER_RTPS_DllAPI MemberId get_id() const;

    BOOSTER_RTPS_DllAPI  uint32_t get_index() const;

    BOOSTER_RTPS_DllAPI std::string get_name() const;

    BOOSTER_RTPS_DllAPI std::vector<uint64_t> get_union_labels() const;

    BOOSTER_RTPS_DllAPI std::string get_default_value() const
    {
        if (!default_value_.empty())
        {
            return default_value_;
        }
        // Try annotation
        return annotation_get_default();
    }

    BOOSTER_RTPS_DllAPI bool is_default_union_value() const;

    BOOSTER_RTPS_DllAPI bool is_consistent(TypeKind parentKind) const;

    BOOSTER_RTPS_DllAPI void add_union_case_index(uint64_t value);

    BOOSTER_RTPS_DllAPI void set_id(MemberId id);

    BOOSTER_RTPS_DllAPI void set_index(uint32_t index);

    BOOSTER_RTPS_DllAPI void set_name(const std::string& name);

    BOOSTER_RTPS_DllAPI void set_type(DynamicType_ptr type);

    BOOSTER_RTPS_DllAPI DynamicType_ptr get_type() const
    {
        return type_;
    }

    BOOSTER_RTPS_DllAPI void set_default_union_value(bool bDefault);

    BOOSTER_RTPS_DllAPI void set_default_value(const std::string& value)
    {
        default_value_ = value;
    }

    // Annotations
    ReturnCode_t apply_annotation(AnnotationDescriptor& descriptor);

    ReturnCode_t apply_annotation(
            const std::string& annotation_name,
            const std::string& key,
            const std::string& value);

    AnnotationDescriptor* get_annotation(const std::string& name) const;

    // Annotations application
    BOOSTER_RTPS_DllAPI bool annotation_is_optional() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_key() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_must_understand() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_non_serialized() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_value() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_default_literal() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_position() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_bit_bound() const;

    // Annotations getters
    BOOSTER_RTPS_DllAPI bool annotation_get_key() const;

    BOOSTER_RTPS_DllAPI std::string annotation_get_value() const;

    BOOSTER_RTPS_DllAPI std::string annotation_get_default() const;

    BOOSTER_RTPS_DllAPI uint16_t annotation_get_position() const;

    BOOSTER_RTPS_DllAPI uint16_t annotation_get_bit_bound() const;

    // Annotations setters
    BOOSTER_RTPS_DllAPI void annotation_set_optional(bool optional);

    BOOSTER_RTPS_DllAPI void annotation_set_key(bool key);

    BOOSTER_RTPS_DllAPI void annotation_set_must_understand(bool must_understand);

    BOOSTER_RTPS_DllAPI void annotation_set_non_serialized(bool non_serialized);

    BOOSTER_RTPS_DllAPI void annotation_set_value(const std::string& value);

    BOOSTER_RTPS_DllAPI void annotation_set_default(const std::string& default_value);

    BOOSTER_RTPS_DllAPI void annotation_set_default_literal();

    BOOSTER_RTPS_DllAPI void annotation_set_position(uint16_t position);

    BOOSTER_RTPS_DllAPI void annotation_set_bit_bound(uint16_t bit_bound);
};

} // namespace types
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS_TYPES_MEMBER_DESCRIPTOR_H
