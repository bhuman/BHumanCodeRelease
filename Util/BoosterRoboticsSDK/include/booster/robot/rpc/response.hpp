#ifndef __BOOSTER_ROBOTICS_SDK_RESPONSE_HPP__
#define __BOOSTER_ROBOTICS_SDK_RESPONSE_HPP__

#include <booster/robot/rpc/response_header.hpp>
#include <booster/robot/rpc/error.hpp>

namespace booster {
namespace robot {

class Response {
public:
    Response() = default;
    Response(
        const ResponseHeader &header,
        const std::string &body) :
        header_(header),
        body_(body) {
    }

    static Response OkResponse() {
        return Response(ResponseHeader(kRpcStatusCodeSuccess), "");
    }

    void SetHeader(const ResponseHeader &header) {
        header_ = header;
    }

    const ResponseHeader &GetHeader() const {
        return header_;
    }

    void SetBody(const std::string &body) {
        body_ = body;
    }

    std::string GetBody() const {
        return body_;
    }

private:
    ResponseHeader header_;
    std::string body_;
};
}
}

#endif // __BOOSTER_ROBOTICS_SDK_RESPONSE_HPP__