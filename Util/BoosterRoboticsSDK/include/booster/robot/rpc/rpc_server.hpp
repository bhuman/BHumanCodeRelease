#ifndef __BOOSTER_ROBOTICS_SDK_B1_SERVER_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_SERVER_HPP__

#include <memory>
#include <thread>

#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/rpc/RpcReqMsg.h>
#include <booster/idl/rpc/RpcRespMsg.h>

namespace booster {
namespace robot {

class RpcServer {
public:
    RpcServer() = default;
    ~RpcServer() = default;

    void Init(const std::string &channel_name);
    void Stop();

protected:
    virtual Response HandleRequest(const Request &req) = 0;

private:
    void DdsReqMsgHandler(const void *msg);
    int32_t SendResponse(const std::string &uuid, const Response &resp);

    std::shared_ptr<ChannelPublisher<booster_msgs::msg::RpcRespMsg>> channel_publisher_;
    std::shared_ptr<ChannelSubscriber<booster_msgs::msg::RpcReqMsg>> channel_subscriber_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_B1_SERVER_HPP__