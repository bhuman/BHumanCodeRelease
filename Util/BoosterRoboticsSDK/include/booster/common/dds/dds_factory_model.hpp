#ifndef __BOOSTER_ROBOTICS_SDK_DDS_FACTORY_MODEL_HPP__
#define __BOOSTER_ROBOTICS_SDK_DDS_FACTORY_MODEL_HPP__

#include <booster/common/dds/dds_topic_channel.hpp>
#include <booster/third_party/nlohmann_json/json.hpp>

#include <map>

#include <iostream>

namespace booster {
namespace common {

class DdsFactoryModel {
public:
    DdsFactoryModel();
    ~DdsFactoryModel();

    void Init(uint32_t domain_id, const std::string &network_interface = "");
    void Init(const nlohmann::json &config);

    void CloseWriter(const std::string &channel_name) {
        publisher_->delete_datawriter(publisher_->lookup_datawriter(channel_name.c_str()));
    }

    void CloseReader(const std::string &channel_name) {
        subscriber_->delete_datareader(subscriber_->lookup_datareader(channel_name.c_str()));
    }

    void CloseTopic(DdsTopicPtr topic) {
        participant_->delete_topic(topic.get());
    }

    DdsTopicPtr GetTopic(const std::string &topic_name) {
        auto it = topic_map_.find(topic_name);
        if (it != topic_map_.end()) {
            return it->second;
        }
        return nullptr;
    }

    template <typename MSG>
    DdsTopicChannelPtr<MSG> CreateTopicChannel(
        const std::string &topic_name) {
        TypeSupport type_support(new MSG());
        DdsTopicChannelPtr<MSG> topic_channel = std::make_shared<DdsTopicChannel<MSG>>();
        if (participant_ == nullptr) {
            std::cerr << "Failed to create participant." << std::endl;
            return nullptr;
        }
        auto topic = GetTopic(topic_name);
        if (topic == nullptr) {
            type_support.register_type(participant_.get());
            topic = DdsTopicPtr(
                participant_->create_topic(topic_name, type_support.get_type_name(), TOPIC_QOS_DEFAULT),
                [](DdsTopic *topic) {});
            if (topic == nullptr) {
                std::cerr << "Failed to create topic." << std::endl;
                return nullptr;
            }
            topic_map_[topic_name] = topic;
        }

        topic_channel->SetTopic(topic);
        return topic_channel;
    }

    template <typename MSG>
    void SetWriter(DdsTopicChannelPtr<MSG> topic_channel) {
        topic_channel->SetWriter(publisher_, writer_qos_);
    }

    template <typename MSG>
    void SetReader(
        DdsTopicChannelPtr<MSG> topic_channel,
        const std::function<void(const void *)> &handler) {
        DdsReaderCallback cb(handler);
        topic_channel->SetReader(subscriber_, reader_qos_, cb);
    }

private:
    DdsParticipantPtr participant_;
    DdsPublisherPtr publisher_;
    DdsSubscriberPtr subscriber_;

    std::map<std::string, DdsTopicPtr> topic_map_;

    DomainParticipantQos participant_qos_;
    TopicQos topic_qos_;
    PublisherQos publisher_qos_;
    SubscriberQos subscriber_qos_;
    DataWriterQos writer_qos_;
    DataReaderQos reader_qos_;
};

using DdsFactoryModelPtr = std::shared_ptr<DdsFactoryModel>;

}
} // namespace booster::common

#endif // __BOOSTER_ROBOTICS_SDK_DDS_FACTORY_MODEL_HPP__