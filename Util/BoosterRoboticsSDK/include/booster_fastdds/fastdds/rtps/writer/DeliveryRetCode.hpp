#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_WRITER_DELIVERYRETCODE_HPP_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_WRITER_DELIVERYRETCODE_HPP_

#include <cstdint>

namespace booster_eprosima {
namespace fastrtps {
namespace rtps {

enum class DeliveryRetCode : uint32_t
{
    DELIVERED,
    NOT_DELIVERED,
    EXCEEDED_LIMIT
};

} // namespace rtps
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_WRITER_DELIVERYRETCODE_HPP_
