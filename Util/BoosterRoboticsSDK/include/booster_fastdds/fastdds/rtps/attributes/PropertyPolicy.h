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

/*!
 * @file PropertyPolicy.h
 */
#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_ATTRIBUTES_PROPERTYPOLICY_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_ATTRIBUTES_PROPERTYPOLICY_H_

#include <booster_fastdds/fastdds/rtps/common/Property.h>
#include <booster_fastdds/fastdds/rtps/common/BinaryProperty.h>
#include <booster_fastdds/fastrtps/fastrtps_dll.h>

namespace booster_eprosima {
namespace fastrtps {
namespace rtps {

class PropertyPolicy
{
    public:
        BOOSTER_RTPS_DllAPI PropertyPolicy() {}

        BOOSTER_RTPS_DllAPI PropertyPolicy(const PropertyPolicy& property_policy) :
            properties_(property_policy.properties_),
            binary_properties_(property_policy.binary_properties_) {}

        BOOSTER_RTPS_DllAPI PropertyPolicy(PropertyPolicy&& property_policy) :
            properties_(std::move(property_policy.properties_)),
            binary_properties_(std::move(property_policy.binary_properties_)) {}

        BOOSTER_RTPS_DllAPI PropertyPolicy& operator=(const PropertyPolicy& property_policy)
        {
            properties_ = property_policy.properties_;
            binary_properties_ = property_policy.binary_properties_;
            return *this;
        }

        BOOSTER_RTPS_DllAPI PropertyPolicy& operator=(PropertyPolicy&& property_policy)
        {
            properties_ = std::move(property_policy.properties_);
            binary_properties_= std::move(property_policy.binary_properties_);
            return *this;
        }

        BOOSTER_RTPS_DllAPI bool operator==(const PropertyPolicy& b) const
        {
            return (this->properties_ == b.properties_) &&
                    (this->binary_properties_ == b.binary_properties_);
        }

        //!Get properties
        BOOSTER_RTPS_DllAPI const PropertySeq& properties() const
        {
            return properties_;
        }

        //!Set properties
        BOOSTER_RTPS_DllAPI PropertySeq& properties()
        {
            return properties_;
        }

        //!Get binary_properties
        BOOSTER_RTPS_DllAPI const BinaryPropertySeq& binary_properties() const
        {
            return binary_properties_;
        }

        //!Set binary_properties
        BOOSTER_RTPS_DllAPI BinaryPropertySeq& binary_properties()
        {
            return binary_properties_;
        }

    private:
        PropertySeq properties_;

        BinaryPropertySeq binary_properties_;
};

class PropertyPolicyHelper
{
    public:
        /*!
        * @brief Returns only the properties whose name starts with the prefix.
        * Prefix is removed in returned properties.
        * @param property_policy PropertyPolicy where properties will be searched.
        * @param prefix Prefix used to search properties.
        * @return A copy of properties whose name starts with the prefix.
        */
        BOOSTER_RTPS_DllAPI static PropertyPolicy get_properties_with_prefix(
                const PropertyPolicy& property_policy,
                const std::string& prefix);

        //!Get the length of the property_policy
        BOOSTER_RTPS_DllAPI static size_t length(const PropertyPolicy& property_policy);

        //!Look for a property_policy by name
        BOOSTER_RTPS_DllAPI static std::string* find_property(
                PropertyPolicy& property_policy,
                const std::string& name);

        //!Retrieves a property_policy by name
        BOOSTER_RTPS_DllAPI static const std::string* find_property(
                const PropertyPolicy& property_policy,
                const std::string& name);
};

} //namespace rtps
} //namespace fastrtps
} //namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_ATTRIBUTES_PROPERTYPOLICY_H_
