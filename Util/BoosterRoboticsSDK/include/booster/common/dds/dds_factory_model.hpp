#ifndef __BOOSTER_ROBOTICS_SDK_DDS_FACTORY_MODEL_HPP__
#define __BOOSTER_ROBOTICS_SDK_DDS_FACTORY_MODEL_HPP__

#include <booster/common/dds/dds_topic_channel.hpp>
#include <booster/third_party/nlohmann_json/json.hpp>

#include <iostream>
#include <map>

namespace booster {
namespace common {

class DdsFactoryModel {
public:
    DdsFactoryModel();
    ~DdsFactoryModel();

    void Init(uint32_t domain_id, const std::string &network_interface = "");
    void Init(const nlohmann::json &config);
    void InitDefault(int32_t domain_id);
    void InitWithConfigPath(int32_t domain_id, const std::string &config_file_path);

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
    DdsTopicChannelPtr<MSG> CreateTopicChannel(const std::string &topic_name) {
        DdsTypeSupport type_support(new MSG());
        DdsTopicChannelPtr<MSG> topic_channel = std::make_shared<DdsTopicChannel<MSG>>();
        if (participant_ == nullptr) {
            std::cerr << "Failed to create participant." << std::endl;
            return nullptr;
        }
        auto topic = GetTopic(topic_name);
        if (topic == nullptr) {
            type_support.register_type(participant_.get());
            topic = DdsTopicPtr(
                participant_->create_topic(topic_name, type_support.get_type_name(), booster_eprosima::fastdds::dds::TOPIC_QOS_DEFAULT),
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
    void SetWriter(
        DdsTopicChannelPtr<MSG> topic_channel,
        bool reliable = false) {
        if (topic_channel == nullptr) {
            std::cerr << "Failed to set writer: topic channel is null." << std::endl;
            return;
        }
        auto writer_qos = writer_qos_;
        if (reliable) {
            writer_qos.reliability().kind = booster_eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
        }
        topic_channel->SetWriter(publisher_, writer_qos);
    }

    template <typename MSG>
    void SetReader(
        DdsTopicChannelPtr<MSG> topic_channel,
        const std::function<void(const void *)> &handler,
        bool reliable = false,
        const DdsReaderExecutorOptions &executor_options = {}) {
        if (topic_channel == nullptr) {
            std::cerr << "Failed to set reader: topic channel is null." << std::endl;
            return;
        }
        DdsReaderCallback cb(handler);
        auto reader_qos = reader_qos_;
        if (reliable) {
            reader_qos.reliability().kind = booster_eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
        }
        topic_channel->SetReader(subscriber_, reader_qos, cb, executor_options);
    }

private:
    DdsParticipantPtr participant_;
    DdsPublisherPtr publisher_;
    DdsSubscriberPtr subscriber_;

    std::map<std::string, DdsTopicPtr> topic_map_;

    DdsDomainParticipantQos participant_qos_;
    DdsTopicQos topic_qos_;
    DdsPublisherQos publisher_qos_;
    DdsSubscriberQos subscriber_qos_;
    DdsDataWriterQos writer_qos_;
    DdsDataReaderQos reader_qos_;
};

using DdsFactoryModelPtr = std::shared_ptr<DdsFactoryModel>;

}
} // namespace booster::common

#endif // __BOOSTER_ROBOTICS_SDK_DDS_FACTORY_MODEL_HPP__
