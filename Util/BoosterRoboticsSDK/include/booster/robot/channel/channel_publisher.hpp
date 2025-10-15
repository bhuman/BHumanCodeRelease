#ifndef __BOOSTER_ROBOTICS_SDK_CHANNEL_PUBLISHER_HPP__
#define __BOOSTER_ROBOTICS_SDK_CHANNEL_PUBLISHER_HPP__

#include <string>
#include <memory>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/rpc/request.hpp>

namespace booster {
namespace robot {

template <typename MSG>
class ChannelPublisher {
public:
    explicit ChannelPublisher(const std::string &channel_name) :
        channel_name_(channel_name) {
    }

    void InitChannel() {
        channel_ptr_ = ChannelFactory::Instance()->CreateSendChannel<MSG>(channel_name_);
    }

    bool Write(MSG *msg) {
        if (channel_ptr_) {
            return channel_ptr_->Write(msg);
        }
        return false;
    }

    void CloseChannel() {
        if (channel_ptr_) {
            ChannelFactory::Instance()->CloseWriter(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    std::string channel_name_;
    ChannelPtr<MSG> channel_ptr_;
};

template <typename MSG>
using ChannelPublisherPtr = std::shared_ptr<ChannelPublisher<MSG>>;

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_CHANNEL_PUBLISHER_HPP__