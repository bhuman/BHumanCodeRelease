#ifndef __BOOSTER_DDS_ENTITY_HPP__
#define __BOOSTER_DDS_ENTITY_HPP__

#include <memory>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <booster/common/dds/dds_callback.hpp>

namespace booster {
namespace common {

using namespace eprosima::fastdds::dds;

// class DdsParticipant {
// public:
//     using NATIVE_TYPE = dds::domain::DomainParticipant;

// }

// class DdsReaderListener {
// public:
//     explicit

// };

using DdsParticipantPtr = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>;
using DdsWriterPtr = std::shared_ptr<eprosima::fastdds::dds::DataWriter>;
using DdsWriter = eprosima::fastdds::dds::DataWriter;
using DdsReaderPtr = std::shared_ptr<eprosima::fastdds::dds::DataReader>;
using DdsReader = eprosima::fastdds::dds::DataReader;
using DdsPublisherPtr = std::shared_ptr<eprosima::fastdds::dds::Publisher>;
using DdsSubscriberPtr = std::shared_ptr<eprosima::fastdds::dds::Subscriber>;
using DdsTopicPtr = std::shared_ptr<eprosima::fastdds::dds::Topic>;
using DdsTopic = eprosima::fastdds::dds::Topic;
using DdsTopicDataTypePtr = std::shared_ptr<eprosima::fastdds::dds::TopicDataType>;
using DdsReaderCallbackPtr = std::shared_ptr<DdsReaderCallback>;

template <typename MSG>
class DdsReaderListener : public DataReaderListener {
public:
    DdsReaderListener() = default;
    ~DdsReaderListener() override = default;

    void SetCallback(const DdsReaderCallback &cb) {
        if (!cb.HasMessageHandler()) {
            std::cerr << "Listener Set Callback: invalid hanlder" << std::endl;
            return;
        }
        cb_ = std::make_shared<DdsReaderCallback>(cb);
    }

    void on_data_available(DataReader *reader) override {
        MSG st;
        SampleInfo info;
        if (reader->take_next_sample(&st, &info) == ReturnCode_t::RETCODE_OK) {
            if (info.valid_data) {
                cb_->OnDataAvailable(&st);
            }
        }
    }

private:
    DdsReaderCallbackPtr cb_;
};

template <typename MSG>
using DdsReaderListenerPtr = std::shared_ptr<DdsReaderListener<MSG>>;
}
} // namespace booster::common

#endif