#ifndef __BOOSTER_DDS_TOPIC_CHANNEL_HPP__
#define __BOOSTER_DDS_TOPIC_CHANNEL_HPP__

#include <iostream>

#include <booster/common/dds/dds_entity.hpp>

using namespace eprosima::fastdds::dds;

namespace booster {
namespace common {

template <typename MSG>
class DdsTopicChannel {
public:
    DdsTopicChannel() = default;
    ~DdsTopicChannel() = default;

    void SetTopic(DdsTopicPtr topic) {
        topic_ = topic;
    }

    void SetWriter(
        const DdsPublisherPtr &publisher,
        const DataWriterQos &qos) {
        auto raw_writer = publisher->create_datawriter(topic_.get(), qos);
        if (raw_writer == nullptr) {
            std::cerr << "Failed to create writer." << std::endl;
            return;
        }
        writer_ = DdsWriterPtr(raw_writer, [](DdsWriter *writer) {});
    }

    void SetReader(
        const DdsSubscriberPtr &subscriber,
        const DataReaderQos &qos,
        const DdsReaderCallback &cb) {
        listener_ = std::make_shared<DdsReaderListener<MSG>>();
        listener_->SetCallback(cb);
        reader_ = DdsReaderPtr(subscriber->create_datareader(
                                   topic_.get(),
                                   qos,
                                   listener_.get()),
                               [](DdsReader *reader) {});
        if (reader_ == nullptr) {
            std::cerr << "Failed to create reader." << std::endl;
            return;
        }
    }

    DdsWriterPtr GetWriter() const {
        return writer_;
    }

    DdsReaderPtr GetReader() const {
        return reader_;
    }

    bool Write(MSG *msg) {
        // const MSG *const_msg_ptr = &msg;
        // MSG *non_const_msg_ptr = const_cast<MSG *>(const_msg_ptr);
        // return writer_->write(static_cast<void *>(&non_const_msg_ptr));
        return writer_->write(msg);
    }

private:
    DdsWriterPtr writer_;
    DdsReaderPtr reader_;
    DdsTopicPtr topic_;
    DdsReaderListenerPtr<MSG> listener_;
};

template <typename MSG>
using DdsTopicChannelPtr = std::shared_ptr<DdsTopicChannel<MSG>>;

}
} // namespace booster::common

#endif // __BOOSTER_DDS_TOPIC_CHANNEL_HPP__