#ifndef __BOOSTER_ROBOTICS_SDK_REQUEST_HPP__
#define __BOOSTER_ROBOTICS_SDK_REQUEST_HPP__

#include <booster/robot/rpc/request_header.hpp>

namespace booster {
namespace robot {

class Request {
public:
    Request() = default;
    Request(
        const RequestHeader &header,
        const std::string &body) :
        header_(header),
        body_(body) {
    }

    void SetHeader(const RequestHeader &header) {
        header_ = header;
    }

    RequestHeader GetHeader() const {
        return header_;
    }

    void SetBody(const std::string &body) {
        body_ = body;
    }

    std::string GetBody() const {
        return body_;
    }

private:
    RequestHeader header_;
    std::string body_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_REQUEST_HPP__