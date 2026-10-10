#ifndef __BOOSTER_DDS_ENTITY_HPP__
#define __BOOSTER_DDS_ENTITY_HPP__

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <iostream>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include <booster_fastdds/fastdds/dds/domain/DomainParticipant.hpp>
#include <booster_fastdds/fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <booster_fastdds/fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <booster_fastdds/fastdds/dds/core/policy/QosPolicies.hpp>
#include <booster_fastdds/fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <booster_fastdds/fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <booster_fastdds/fastdds/dds/publisher/Publisher.hpp>
#include <booster_fastdds/fastdds/dds/publisher/DataWriter.hpp>
#include <booster_fastdds/fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <booster_fastdds/fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/Subscriber.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/DataReader.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/DataReaderListener.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/SampleInfo.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <booster_fastdds/fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <booster_fastdds/fastdds/dds/topic/Topic.hpp>
#include <booster_fastdds/fastdds/dds/topic/TopicDataType.hpp>
#include <booster_fastdds/fastdds/dds/topic/TypeSupport.hpp>
#include <booster_fastdds/fastdds/dds/topic/qos/TopicQos.hpp>

#include <booster/common/dds/dds_callback.hpp>

namespace booster {
namespace common {

// class DdsParticipant {
// public:
//     using NATIVE_TYPE = dds::domain::DomainParticipant;

// }

// class DdsReaderListener {
// public:
//     explicit

// };

using DdsParticipant = booster_eprosima::fastdds::dds::DomainParticipant;
using DdsParticipantPtr = std::shared_ptr<DdsParticipant>;
using DdsWriterPtr = std::shared_ptr<booster_eprosima::fastdds::dds::DataWriter>;
using DdsWriter = booster_eprosima::fastdds::dds::DataWriter;
using DdsReaderPtr = std::shared_ptr<booster_eprosima::fastdds::dds::DataReader>;
using DdsReader = booster_eprosima::fastdds::dds::DataReader;
using DdsPublisher = booster_eprosima::fastdds::dds::Publisher;
using DdsPublisherPtr = std::shared_ptr<DdsPublisher>;
using DdsSubscriber = booster_eprosima::fastdds::dds::Subscriber;
using DdsSubscriberPtr = std::shared_ptr<DdsSubscriber>;
using DdsTopicPtr = std::shared_ptr<booster_eprosima::fastdds::dds::Topic>;
using DdsTopic = booster_eprosima::fastdds::dds::Topic;
using DdsTopicDataTypePtr = std::shared_ptr<booster_eprosima::fastdds::dds::TopicDataType>;
using DdsTypeSupport = booster_eprosima::fastdds::dds::TypeSupport;
using DdsDataReaderListener = booster_eprosima::fastdds::dds::DataReaderListener;
using DdsSampleInfo = booster_eprosima::fastdds::dds::SampleInfo;
using DdsDataWriterQos = booster_eprosima::fastdds::dds::DataWriterQos;
using DdsDataReaderQos = booster_eprosima::fastdds::dds::DataReaderQos;
using DdsDomainParticipantQos = booster_eprosima::fastdds::dds::DomainParticipantQos;
using DdsTopicQos = booster_eprosima::fastdds::dds::TopicQos;
using DdsPublisherQos = booster_eprosima::fastdds::dds::PublisherQos;
using DdsSubscriberQos = booster_eprosima::fastdds::dds::SubscriberQos;
using DdsPublicationMatchedStatus = booster_eprosima::fastdds::dds::PublicationMatchedStatus;
using DdsSubscriptionMatchedStatus = booster_eprosima::fastdds::dds::SubscriptionMatchedStatus;
using DdsReaderCallbackPtr = std::shared_ptr<DdsReaderCallback>;
using ReturnCode_t = booster_eprosima::fastrtps::types::ReturnCode_t;

enum class DdsExecutorOverflowPolicy {
    kDropNewest,
    kDropOldest,
    kLatestOnly,
};

enum class DdsExecutorDispatchMode {
    kShared,
    kDedicated,
};

struct DdsReaderExecutorOptions {
    // 0 means unbounded. Generic subscribers use a bounded queue by default.
    size_t queue_capacity{64};
    DdsExecutorOverflowPolicy overflow_policy{DdsExecutorOverflowPolicy::kDropOldest};
    bool enable_metrics{false};
    DdsExecutorDispatchMode dispatch_mode{DdsExecutorDispatchMode::kShared};
    bool enable_realtime_listener{false};
    int realtime_listener_priority{80};
    uint64_t realtime_listener_cpu_affinity_mask{0};
    bool enable_realtime_dedicated_executor{false};
    int realtime_dedicated_executor_priority{70};
    uint64_t realtime_dedicated_executor_cpu_affinity_mask{0};
};

struct DdsReaderExecutorMetrics {
    uint64_t samples_received{0};
    uint64_t samples_dropped{0};
    uint64_t callbacks_executed{0};
    size_t current_queue_size{0};
    size_t queue_high_watermark{0};
    uint64_t max_queue_latency_us{0};
    uint64_t max_callback_duration_us{0};
};

class DdsCallbackExecutor {
public:
    static DdsCallbackExecutor &Instance() {
        static DdsCallbackExecutor instance;
        return instance;
    }

    void Submit(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace_back(std::move(task));
        }
        cv_.notify_one();
    }

private:
    DdsCallbackExecutor() {
        const auto thread_count = std::min<size_t>(
            4,
            std::max<size_t>(1, std::thread::hardware_concurrency()));
        executors_.reserve(thread_count);
        for (size_t i = 0; i < thread_count; ++i) {
            executors_.emplace_back([this]() { Run(); });
        }
    }

    ~DdsCallbackExecutor() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_requested_ = true;
        }
        cv_.notify_all();
        for (auto &executor : executors_) {
            if (executor.joinable()) {
                executor.join();
            }
        }
    }

    void Run() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] {
                    return stop_requested_ || !tasks_.empty();
                });

                if (tasks_.empty()) {
                    if (stop_requested_) {
                        return;
                    }
                    continue;
                }

                task = std::move(tasks_.front());
                tasks_.pop_front();
            }

            task();
        }
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> tasks_;
    std::vector<std::thread> executors_;
    bool stop_requested_{false};
};

class DdsDedicatedCallbackExecutor {
public:
    explicit DdsDedicatedCallbackExecutor(const DdsReaderExecutorOptions &options) :
        options_(options),
        executor_([this]() { Run(); }) {
    }

    ~DdsDedicatedCallbackExecutor() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_requested_ = true;
        }
        cv_.notify_all();
        if (executor_.joinable()) {
            executor_.join();
        }
    }

    void Submit(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace_back(std::move(task));
        }
        cv_.notify_one();
    }

    static void TryConfigureThread(
        const char *name,
        int priority,
        uint64_t cpu_affinity_mask) {
#if defined(__linux__)
        if (cpu_affinity_mask != 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            for (size_t cpu_index = 0; cpu_index < sizeof(cpu_affinity_mask) * 8; ++cpu_index) {
                if ((cpu_affinity_mask & (uint64_t(1) << cpu_index)) != 0) {
                    CPU_SET(static_cast<int>(cpu_index), &cpuset);
                }
            }
            const int affinity_rc = pthread_setaffinity_np(
                pthread_self(),
                sizeof(cpuset),
                &cpuset);
            if (affinity_rc != 0) {
                std::cerr << name << ": pthread_setaffinity_np(mask=0x"
                          << std::hex << cpu_affinity_mask << std::dec
                          << ") failed, rc=" << affinity_rc << std::endl;
            }
        }

        if (priority > 0) {
            sched_param params{};
            params.sched_priority = priority;
            const int priority_rc = pthread_setschedparam(
                pthread_self(),
                SCHED_FIFO,
                &params);
            if (priority_rc != 0) {
                std::cerr << name << ": pthread_setschedparam(SCHED_FIFO, "
                          << priority << ") failed, rc=" << priority_rc << std::endl;
            }
        }
#else
        (void)name;
        (void)priority;
        (void)cpu_affinity_mask;
#endif
    }

    void Run() {
        if (options_.enable_realtime_dedicated_executor) {
            TryConfigureThread(
                "DdsDedicatedCallbackExecutor",
                options_.realtime_dedicated_executor_priority,
                options_.realtime_dedicated_executor_cpu_affinity_mask);
        }
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] {
                    return stop_requested_ || !tasks_.empty();
                });

                if (tasks_.empty()) {
                    if (stop_requested_) {
                        return;
                    }
                    continue;
                }

                task = std::move(tasks_.front());
                tasks_.pop_front();
            }

            task();
        }
    }

private:
    DdsReaderExecutorOptions options_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> tasks_;
    std::thread executor_;
    bool stop_requested_{false};
};

template <typename MSG>
class DdsReaderListener : public DdsDataReaderListener, public std::enable_shared_from_this<DdsReaderListener<MSG>> {
public:
    DdsReaderListener() = default;
    ~DdsReaderListener() override {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
        pending_messages_.clear();
        callback_scheduled_ = false;
    }

    void SetCallback(const DdsReaderCallback &cb) {
        if (!cb.HasMessageHandler()) {
            std::cerr << "Listener Set Callback: invalid hanlder" << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        cb_ = std::make_shared<DdsReaderCallback>(cb);
    }

    void SetExecutorOptions(const DdsReaderExecutorOptions &options) {
        std::lock_guard<std::mutex> lock(mutex_);
        executor_options_ = options;
        if (executor_options_.dispatch_mode == DdsExecutorDispatchMode::kDedicated) {
            if (dedicated_executor_ == nullptr) {
                dedicated_executor_ = std::make_unique<DdsDedicatedCallbackExecutor>(
                    executor_options_);
            }
        } else {
            dedicated_executor_.reset();
        }
    }

    DdsReaderExecutorMetrics GetExecutorMetrics() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto metrics = metrics_;
        metrics.current_queue_size = pending_messages_.size();
        return metrics;
    }

    void on_data_available(DdsReader *reader) override {
        if (reader == nullptr || cb_ == nullptr) {
            return;
        }

        ApplyRealtimeListenerThreadConfigOnce();

        std::deque<PendingMessage> ready_messages;
        DdsSampleInfo info;
        const auto listener_enter_steady = std::chrono::steady_clock::now();
        const auto listener_enter_system = std::chrono::system_clock::now();
        while (true) {
            MSG st;
            if (reader->take_next_sample(&st, &info) != ReturnCode_t::RETCODE_OK) {
                break;
            }
            if (info.valid_data) {
                ready_messages.push_back(PendingMessage{
                    std::move(st),
                    info,
                    listener_enter_steady,
                    listener_enter_system,
                    std::chrono::steady_clock::now(),
                    std::chrono::system_clock::now(),
                });
            }
        }

        if (ready_messages.empty()) {
            return;
        }

        bool should_schedule = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_requested_) {
                return;
            }
            for (auto &message : ready_messages) {
                EnqueuePendingMessageLocked(std::move(message));
            }
            if (!callback_scheduled_ && !pending_messages_.empty()) {
                callback_scheduled_ = true;
                should_schedule = true;
            }
        }

        if (should_schedule) {
            ScheduleExecution();
        }
    }

private:
    struct PendingMessage {
        MSG message;
        DdsSampleInfo sample_info;
        std::chrono::steady_clock::time_point listener_enter_steady_time;
        std::chrono::system_clock::time_point listener_enter_system_time;
        std::chrono::steady_clock::time_point enqueued_at;
        std::chrono::system_clock::time_point enqueued_system_time;
    };

    static bool IsValidRtpsSystemTime(const booster_eprosima::fastrtps::rtps::Time_t &time) {
        return time.seconds() >= 0;
    }

    static std::chrono::system_clock::time_point RtpsTimeToSystemClock(
        const booster_eprosima::fastrtps::rtps::Time_t &time) {
        return std::chrono::system_clock::time_point(
            std::chrono::nanoseconds(time.to_ns()));
    };

    void ApplyRealtimeListenerThreadConfigOnce() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (listener_thread_configured_ || !executor_options_.enable_realtime_listener) {
            return;
        }
        DdsDedicatedCallbackExecutor::TryConfigureThread(
            "DdsReaderListener",
            executor_options_.realtime_listener_priority,
            executor_options_.realtime_listener_cpu_affinity_mask);
        listener_thread_configured_ = true;
    }

    void EnqueuePendingMessageLocked(PendingMessage &&message) {
        if (executor_options_.enable_metrics) {
            ++metrics_.samples_received;
        }

        const auto capacity = executor_options_.queue_capacity;
        switch (executor_options_.overflow_policy) {
        case DdsExecutorOverflowPolicy::kDropNewest:
            if (capacity > 0 && pending_messages_.size() >= capacity) {
                RecordDropLocked(1);
                return;
            }
            pending_messages_.push_back(std::move(message));
            break;
        case DdsExecutorOverflowPolicy::kDropOldest:
            if (capacity > 0 && pending_messages_.size() >= capacity) {
                pending_messages_.pop_front();
                RecordDropLocked(1);
            }
            pending_messages_.push_back(std::move(message));
            break;
        case DdsExecutorOverflowPolicy::kLatestOnly:
            if (!pending_messages_.empty()) {
                RecordDropLocked(pending_messages_.size());
                pending_messages_.clear();
            }
            pending_messages_.push_back(std::move(message));
            break;
        }

        if (executor_options_.enable_metrics) {
            metrics_.queue_high_watermark = std::max(
                metrics_.queue_high_watermark,
                pending_messages_.size());
        }
    }

    void RecordDropLocked(size_t drop_count) {
        if (executor_options_.enable_metrics) {
            metrics_.samples_dropped += drop_count;
        }
    }

    void ScheduleExecution() {
        auto weak_self = this->weak_from_this();
        std::function<void()> task = [weak_self]() {
            auto self = weak_self.lock();
            if (self == nullptr) {
                return;
            }
            self->ExecutePendingCallbacks();
        };

        DdsExecutorDispatchMode dispatch_mode = DdsExecutorDispatchMode::kShared;
        DdsDedicatedCallbackExecutor *dedicated_executor = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            dispatch_mode = executor_options_.dispatch_mode;
            dedicated_executor = dedicated_executor_.get();
        }

        if (dispatch_mode == DdsExecutorDispatchMode::kDedicated && dedicated_executor != nullptr) {
            dedicated_executor->Submit(std::move(task));
            return;
        }

        DdsCallbackExecutor::Instance().Submit(std::move(task));
    }

    void ExecutePendingCallbacks() {
        while (true) {
            PendingMessage pending_message;
            DdsReaderCallbackPtr cb;
            bool metrics_enabled = false;
            size_t pending_queue_size_before_callback = 0;
            size_t pending_queue_size_after_pop = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (stop_requested_) {
                    callback_scheduled_ = false;
                    return;
                }
                if (pending_messages_.empty()) {
                    callback_scheduled_ = false;
                    return;
                }

                pending_queue_size_before_callback = pending_messages_.size();
                pending_message = std::move(pending_messages_.front());
                pending_messages_.pop_front();
                cb = cb_;
                metrics_enabled = executor_options_.enable_metrics;
                pending_queue_size_after_pop = pending_messages_.size();
            }

            const auto callback_start_steady = std::chrono::steady_clock::now();
            const auto callback_start_system = std::chrono::system_clock::now();
            if (cb != nullptr) {
                DdsMessageContext context;
                context.listener_enter_steady_time =
                    pending_message.listener_enter_steady_time;
                context.listener_enter_system_time =
                    pending_message.listener_enter_system_time;
                context.dds_rx_steady_time = pending_message.enqueued_at;
                context.dds_rx_system_time = pending_message.enqueued_system_time;
                context.callback_start_steady_time = callback_start_steady;
                context.callback_start_system_time = callback_start_system;
                context.listener_enter_to_dds_rx_delay =
                    pending_message.enqueued_at - pending_message.listener_enter_steady_time;
                context.dds_rx_to_callback_delay = callback_start_steady - pending_message.enqueued_at;
                context.queue_size_before_callback = pending_queue_size_before_callback;
                context.queue_size_after_pop = pending_queue_size_after_pop;
                context.has_source_timestamp = IsValidRtpsSystemTime(
                    pending_message.sample_info.source_timestamp);
                if (context.has_source_timestamp) {
                    context.source_timestamp_system_time = RtpsTimeToSystemClock(
                        pending_message.sample_info.source_timestamp);
                }
                context.has_reception_timestamp = IsValidRtpsSystemTime(
                    pending_message.sample_info.reception_timestamp);
                if (context.has_reception_timestamp) {
                    context.reception_timestamp_system_time = RtpsTimeToSystemClock(
                        pending_message.sample_info.reception_timestamp);
                }
                cb->OnDataAvailable(&pending_message.message, context);
            }

            if (metrics_enabled) {
                const auto callback_end = std::chrono::steady_clock::now();
                const auto queue_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    callback_start_steady - pending_message.enqueued_at).count();
                const auto callback_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    callback_end - callback_start_steady).count();

                std::lock_guard<std::mutex> lock(mutex_);
                ++metrics_.callbacks_executed;
                metrics_.current_queue_size = pending_queue_size_after_pop;
                metrics_.max_queue_latency_us = std::max<uint64_t>(
                    metrics_.max_queue_latency_us,
                    static_cast<uint64_t>(queue_latency_us));
                metrics_.max_callback_duration_us = std::max<uint64_t>(
                    metrics_.max_callback_duration_us,
                    static_cast<uint64_t>(callback_duration_us));
            }
        }
    }

private:
    mutable std::mutex mutex_;
    std::deque<PendingMessage> pending_messages_;
    bool stop_requested_{false};
    bool callback_scheduled_{false};
    bool listener_thread_configured_{false};
    DdsReaderExecutorOptions executor_options_;
    DdsReaderExecutorMetrics metrics_;
    DdsReaderCallbackPtr cb_;
    std::unique_ptr<DdsDedicatedCallbackExecutor> dedicated_executor_;
};

template <typename MSG>
using DdsReaderListenerPtr = std::shared_ptr<DdsReaderListener<MSG>>;
}
} // namespace booster::common

#endif
