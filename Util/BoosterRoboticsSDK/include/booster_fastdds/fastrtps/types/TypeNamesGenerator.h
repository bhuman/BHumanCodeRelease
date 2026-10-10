#ifndef BOOSTER_FASTDDS__TYPE_NAMES_GENERATOR_
#define BOOSTER_FASTDDS__TYPE_NAMES_GENERATOR_

#include <booster_fastdds/fastrtps/fastrtps_dll.h>

#include <cstdint>
#include <string>
#include <vector>

namespace booster_eprosima {
namespace fastrtps {
namespace types {

class TypeNamesGenerator
{
public:

    BOOSTER_RTPS_DllAPI static std::string get_string_type_name(
            uint32_t bound,
            bool wide,
            bool generate_identifier = true);

    BOOSTER_RTPS_DllAPI static std::string get_sequence_type_name(
            const std::string& type_name,
            uint32_t bound,
            bool generate_identifier = true);

    BOOSTER_RTPS_DllAPI static std::string get_array_type_name(
            const std::string& type_name,
            const std::vector<uint32_t>& bound,
            bool generate_identifier = true);

    BOOSTER_RTPS_DllAPI static std::string get_array_type_name(
            const std::string& type_name,
            const std::vector<uint32_t>& bound,
            uint32_t& ret_size,
            bool generate_identifier = true);

    BOOSTER_RTPS_DllAPI static std::string get_map_type_name(
            const std::string& key_type_name,
            const std::string& value_type_name,
            uint32_t bound,
            bool generate_identifier = true);
};

} // namespace types
} // namespace fastrtps
} // namespace booster_eprosima

#endif //BOOSTER_FASTDDS__TYPE_NAMES_GENERATOR_
