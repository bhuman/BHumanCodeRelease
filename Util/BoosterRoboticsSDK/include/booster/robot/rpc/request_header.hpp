#ifndef __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__
#define __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__

#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

class RequestHeader {
public:
    RequestHeader() = default;
    RequestHeader(int64_t api_id) :
        api_id_(api_id) {
    }

    void SetApiId(int64_t api_id) {
        api_id_ = api_id;
    }

    int64_t GetApiId() const {
        return api_id_;
    }

public:
    void FromJson(nlohmann::json &json) {
        api_id_ = json["api_id"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["api_id"] = api_id_;
        return json;
    }

private:
    int64_t api_id_ = 0;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__