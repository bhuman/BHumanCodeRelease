#ifndef __BOOSTER_ROBOTICS_SDK_CHANNEL_FACTORY_HPP__
#define __BOOSTER_ROBOTICS_SDK_CHANNEL_FACTORY_HPP__

#include <mutex>

#include <booster/common/dds/dds_factory_model.hpp>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

template <typename MSG>
using Channel = booster::common::DdsTopicChannel<MSG>;

template <typename MSG>
using ChannelPtr = booster::common::DdsTopicChannelPtr<MSG>;

using TopicDataTypePtr = booster::common::DdsTopicDataTypePtr;
using TopicPtr = booster::common::DdsTopicPtr;

class ChannelFactory {
public:
    static ChannelFactory *Instance() {
        static ChannelFactory instance;
        return &instance;
    }

    void Init(int32_t domain_id, const std::string &network_interface = "");
    void Init(const nlohmann::json &config);

    void CloseWriter(const std::string &channel_name);
    void CloseReader(const std::string &channel_name);
    void CloseTopic(TopicPtr topic);

    template <typename MSG>
    ChannelPtr<MSG> CreateSendChannel(const std::string &name) {
        ChannelPtr<MSG> channel_ptr = dds_factory_model_->CreateTopicChannel<MSG>(name);
        dds_factory_model_->SetWriter(channel_ptr);
        return channel_ptr;
    }

    template <typename MSG>
    ChannelPtr<MSG> CreateRecvChannel(const std::string &name, std::function<void(const void *)> handler) {
        ChannelPtr<MSG> channel_ptr = dds_factory_model_->CreateTopicChannel<MSG>(name);
        dds_factory_model_->SetReader(channel_ptr, handler);
        return channel_ptr;
    }

private:
    bool initialized_ = false;
    std::mutex mutex_;
    common::DdsFactoryModelPtr dds_factory_model_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_CHANNEL_FACTORY_HPP__