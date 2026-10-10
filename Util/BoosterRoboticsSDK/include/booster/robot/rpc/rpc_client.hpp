#ifndef __BOOSTER_ROBOTICS_SDK_B1_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_CLIENT_HPP__

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <booster/idl/rpc/RpcReqMsg.h>
#include <booster/idl/rpc/RpcRespMsg.h>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/rpc/request.hpp>
#include <booster/robot/rpc/response.hpp>

namespace booster {
namespace robot {

class RpcClient {
public:
    static constexpr int64_t kDefaultWaitForServiceTimeoutMs = 5000;

    RpcClient() = default;
    ~RpcClient() = default;

    void Init(const std::string &channel_name);
    void Init(const std::string &channel_name, bool /*reliable*/) {
        Init(channel_name);
    }
    bool WaitForService(
        int64_t timeout_ms = kDefaultWaitForServiceTimeoutMs,
        bool require_response_path = true);
    Response SendApiRequest(const Request &req, int64_t timeout_ms = 1000);
    int32_t SendApiRequestFireAndForget(
        const Request &req,
        int64_t endpoint_match_timeout_ms = 1000);

    void Stop();

    std::string GenUuid();

private:
    static Response MakeErrorResponse(int64_t status, const std::string &body = "");
    bool WaitForEndpoints(bool require_response_path, int64_t timeout_ms) const;
    bool PublishRequest(const Request &req, const std::string &uuid, bool expect_response);
    void DdsSubMsgHandler(const void *msg);

    std::mutex mutex_;
    std::unordered_map<std::string, std::pair<Response, std::unique_ptr<std::condition_variable>>>
        resp_map_;

    std::shared_ptr<ChannelPublisher<booster_msgs::msg::RpcReqMsg>> channel_publisher_;
    std::shared_ptr<ChannelSubscriber<booster_msgs::msg::RpcRespMsg>> channel_subscriber_;
    std::string channel_name_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_B1_CLIENT_HPP__
