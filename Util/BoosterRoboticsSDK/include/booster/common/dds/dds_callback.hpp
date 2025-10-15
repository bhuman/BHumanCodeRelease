#ifndef __BOOSTER_ROBOTICS_SDK_DDS_CALLBACK_HPP__
#define __BOOSTER_ROBOTICS_SDK_DDS_CALLBACK_HPP__

#include <memory>
#include <functional>

using DdsMessageHandler = std::function<void(const void *)>;

namespace booster {
namespace common {

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

private:
    DdsMessageHandler handler_;
};


}
} // namespace booster::common

#endif // __BOOSTER_ROBOTICS_SDK_DDS_CALLBACK_HPP__