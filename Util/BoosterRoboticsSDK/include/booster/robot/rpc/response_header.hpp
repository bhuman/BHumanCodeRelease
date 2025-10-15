#ifndef __BOOSTER_ROBOTICS_SDK_RESPONSE_HEADER_HPP__
#define __BOOSTER_ROBOTICS_SDK_RESPONSE_HEADER_HPP__

#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

class ResponseHeader {
public:
    ResponseHeader() = default;
    ResponseHeader(int64_t status) :
        status_(status) {
    }

    void SetStatus(int64_t status) {
        status_ = status;
    }

    int64_t GetStatus() const {
        return status_;
    }

public:
    void FromJson(nlohmann::json &json) {
        status_ = json["status"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["status"] = status_;
        return json;
    }

private:
    int64_t status_ = -1;

};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_RESPONSE_HEADER_HPP__