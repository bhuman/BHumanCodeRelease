#ifndef __BOOSTER_ROBOTICS_SDK_CHANNEL_PUBLISHER_HPP__
#define __BOOSTER_ROBOTICS_SDK_CHANNEL_PUBLISHER_HPP__

#include <memory>
#include <string>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/rpc/request.hpp>

namespace booster {
namespace robot {

template <typename MSG>
class ChannelPublisher {
public:
    explicit ChannelPublisher(const std::string &channel_name, bool reliable = false) :
        channel_name_(channel_name),
        reliable_(reliable) {
    }

    void InitChannel() {
        channel_ptr_ = ChannelFactory::Instance()->CreateSendChannel<MSG>(channel_name_, reliable_);
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

    size_t GetMatchedSubscriptionsCount() const {
        if (channel_ptr_ == nullptr) {
            return 0;
        }
        return channel_ptr_->GetMatchedSubscriptionsCount();
    }

private:
    std::string channel_name_;
    bool reliable_{false};
    ChannelPtr<MSG> channel_ptr_;
};

template <typename MSG>
using ChannelPublisherPtr = std::shared_ptr<ChannelPublisher<MSG>>;

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_CHANNEL_PUBLISHER_HPP__
