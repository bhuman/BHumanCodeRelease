#ifndef __BOOSTER_ROBOTICS_SDK_DDS_CALLBACK_HPP__
#define __BOOSTER_ROBOTICS_SDK_DDS_CALLBACK_HPP__

#include <chrono>
#include <memory>
#include <functional>

using DdsMessageHandler = std::function<void(const void *)>;

namespace booster {
namespace common {

struct DdsMessageContext {
    std::chrono::steady_clock::time_point listener_enter_steady_time{};
    std::chrono::system_clock::time_point listener_enter_system_time{};
    std::chrono::steady_clock::time_point dds_rx_steady_time{};
    std::chrono::system_clock::time_point dds_rx_system_time{};
    std::chrono::steady_clock::time_point callback_start_steady_time{};
    std::chrono::system_clock::time_point callback_start_system_time{};
    std::chrono::steady_clock::duration listener_enter_to_dds_rx_delay{};
    std::chrono::steady_clock::duration dds_rx_to_callback_delay{};
    size_t queue_size_before_callback{0};
    size_t queue_size_after_pop{0};
    bool has_source_timestamp{false};
    std::chrono::system_clock::time_point source_timestamp_system_time{};
    bool has_reception_timestamp{false};
    std::chrono::system_clock::time_point reception_timestamp_system_time{};
};

const DdsMessageContext *GetCurrentDdsMessageContext();

class DdsReaderCallback {
public:
    DdsReaderCallback() = default;
    explicit DdsReaderCallback(const DdsMessageHandler &handler) :
        handler_(handler){};
    DdsReaderCallback(const DdsReaderCallback &other) = default;
    DdsReaderCallback &operator=(const DdsReaderCallback &other) = default;
    ~DdsReaderCallback() = default;

    bool HasMessageHandler() const;
    void OnDataAvailable(const void *data);
    void OnDataAvailable(const void *data, const DdsMessageContext &context);

private:
    DdsMessageHandler handler_;
};


}
} // namespace booster::common

#endif // __BOOSTER_ROBOTICS_SDK_DDS_CALLBACK_HPP__
