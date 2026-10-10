#ifndef __BOOSTER_DDS_TOPIC_CHANNEL_HPP__
#define __BOOSTER_DDS_TOPIC_CHANNEL_HPP__

#include <iostream>

#include <booster/common/dds/dds_entity.hpp>

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
        const DdsDataWriterQos &qos) {
        if (publisher == nullptr || topic_ == nullptr) {
            std::cerr << "Failed to create writer: publisher or topic is not initialized." << std::endl;
            return;
        }
        auto raw_writer = publisher->create_datawriter(topic_.get(), qos);
        if (raw_writer == nullptr) {
            std::cerr << "Failed to create writer." << std::endl;
            return;
        }
        writer_ = DdsWriterPtr(raw_writer, [](DdsWriter *writer) {});
    }

    void SetReader(
        const DdsSubscriberPtr &subscriber,
        const DdsDataReaderQos &qos,
        const DdsReaderCallback &cb,
        const DdsReaderExecutorOptions &executor_options = {}) {
        if (subscriber == nullptr || topic_ == nullptr) {
            std::cerr << "Failed to create reader: subscriber or topic is not initialized." << std::endl;
            return;
        }
        listener_ = std::make_shared<DdsReaderListener<MSG>>();
        listener_->SetExecutorOptions(executor_options);
        listener_->SetCallback(cb);
        reader_ = DdsReaderPtr(
            subscriber->create_datareader(topic_.get(), qos, listener_.get()),
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

    DdsReaderExecutorMetrics GetReaderExecutorMetrics() const {
        if (listener_ == nullptr) {
            return DdsReaderExecutorMetrics();
        }
        return listener_->GetExecutorMetrics();
    }

    size_t GetMatchedSubscriptionsCount() const {
        if (writer_ == nullptr) {
            return 0;
        }

        DdsPublicationMatchedStatus status;
        if (writer_->get_publication_matched_status(status) != ReturnCode_t::RETCODE_OK) {
            return 0;
        }

        return status.current_count > 0 ? static_cast<size_t>(status.current_count) : 0;
    }

    size_t GetMatchedPublicationsCount() const {
        if (reader_ == nullptr) {
            return 0;
        }

        DdsSubscriptionMatchedStatus status;
        if (reader_->get_subscription_matched_status(status) != ReturnCode_t::RETCODE_OK) {
            return 0;
        }

        return status.current_count > 0 ? static_cast<size_t>(status.current_count) : 0;
    }

    bool Write(MSG *msg) {
        if (writer_ == nullptr) {
            std::cerr << "Write failed: writer is not initialized." << std::endl;
            return false;
        }
        if (msg == nullptr) {
            std::cerr << "Write failed: message is null." << std::endl;
            return false;
        }
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
