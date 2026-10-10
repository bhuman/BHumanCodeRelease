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

#ifndef BOOSTER_FASTDDS_TYPES_TYPE_DESCRIPTOR_H
#define BOOSTER_FASTDDS_TYPES_TYPE_DESCRIPTOR_H

#include <booster_fastdds/fastrtps/types/AnnotationDescriptor.h>
#include <booster_fastdds/fastrtps/types/DynamicTypePtr.h>
#include <booster_fastdds/fastrtps/types/TypesBase.h>

class MemberDescriptor;
class DynamicType;

namespace booster_eprosima {
namespace fastrtps {
namespace types {

class TypeDescriptor
{
protected:

    TypeKind kind_;                         // Type Kind.
    std::string name_;                      // Type Name.
    DynamicType_ptr base_type_;             // SuperType of an structure or base type of an alias type.
    DynamicType_ptr discriminator_type_;    // Discrimination type for a union.
    std::vector<uint32_t> bound_;           // Length for strings, arrays, sequences, maps and bitmasks.
    DynamicType_ptr element_type_;          // Value Type for arrays, sequences, maps, bitmasks.
    DynamicType_ptr key_element_type_;      // Key Type for maps.
    std::vector<AnnotationDescriptor*> annotation_; // Annotations to apply

    BOOSTER_RTPS_DllAPI void clean();

    static bool is_type_name_consistent(
            const std::string& sName);

    friend class DynamicTypeBuilderFactory;
    friend class TypeObjectFactory;
    friend class DynamicType;
    friend class MemberDescriptor;
    friend class DynamicDataHelper;

public:

    BOOSTER_RTPS_DllAPI TypeDescriptor();

    BOOSTER_RTPS_DllAPI TypeDescriptor(
            const TypeDescriptor* other);

    BOOSTER_RTPS_DllAPI TypeDescriptor(
            const std::string& name,
            TypeKind kind);

    BOOSTER_RTPS_DllAPI ~TypeDescriptor();

    BOOSTER_RTPS_DllAPI ReturnCode_t copy_from(
            const TypeDescriptor* descriptor);

    BOOSTER_RTPS_DllAPI bool equals(
            const TypeDescriptor* descriptor) const;

    BOOSTER_RTPS_DllAPI bool is_consistent() const;

    BOOSTER_RTPS_DllAPI DynamicType_ptr get_base_type() const;

    BOOSTER_RTPS_DllAPI uint32_t get_bounds(
            uint32_t index = 0) const;

    BOOSTER_RTPS_DllAPI uint32_t get_bounds_size() const;

    BOOSTER_RTPS_DllAPI DynamicType_ptr get_discriminator_type() const;

    BOOSTER_RTPS_DllAPI DynamicType_ptr get_element_type() const;

    BOOSTER_RTPS_DllAPI DynamicType_ptr get_key_element_type() const;

    BOOSTER_RTPS_DllAPI TypeKind get_kind() const;

    BOOSTER_RTPS_DllAPI std::string get_name() const;

    BOOSTER_RTPS_DllAPI uint32_t get_total_bounds() const;

    BOOSTER_RTPS_DllAPI void set_kind(
            TypeKind kind);

    BOOSTER_RTPS_DllAPI void set_name(
            std::string name);

    BOOSTER_RTPS_DllAPI ReturnCode_t apply_annotation(
            AnnotationDescriptor& descriptor);

    BOOSTER_RTPS_DllAPI ReturnCode_t apply_annotation(
            const std::string& annotation_name,
            const std::string& key,
            const std::string& value);

    BOOSTER_RTPS_DllAPI AnnotationDescriptor* get_annotation(
            const std::string& name) const;

    // Annotations application
    BOOSTER_RTPS_DllAPI bool annotation_is_extensibility() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_mutable() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_final() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_appendable() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_nested() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_bit_bound() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_key() const;

    BOOSTER_RTPS_DllAPI bool annotation_is_non_serialized() const;

    // Annotation getters
    BOOSTER_RTPS_DllAPI std::string annotation_get_extensibility() const;

    BOOSTER_RTPS_DllAPI bool annotation_get_nested() const;

    BOOSTER_RTPS_DllAPI uint16_t annotation_get_bit_bound() const;

    BOOSTER_RTPS_DllAPI bool annotation_get_key() const;

    // Annotation setters
    BOOSTER_RTPS_DllAPI void annotation_set_extensibility(
            const std::string& extensibility);

    BOOSTER_RTPS_DllAPI void annotation_set_mutable();

    BOOSTER_RTPS_DllAPI void annotation_set_final();

    BOOSTER_RTPS_DllAPI void annotation_set_appendable();

    BOOSTER_RTPS_DllAPI void annotation_set_nested(
            bool nested);

    BOOSTER_RTPS_DllAPI void annotation_set_bit_bound(
            uint16_t bit_bound);

    BOOSTER_RTPS_DllAPI void annotation_set_key(
            bool key);

    BOOSTER_RTPS_DllAPI void annotation_set_non_serialized(
            bool non_serialized);
};

} // namespace types
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS_TYPES_TYPE_DESCRIPTOR_H
