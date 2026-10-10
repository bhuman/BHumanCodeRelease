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

    void SetExpectResponse(bool expect_response) {
        expect_response_ = expect_response;
    }

    bool GetExpectResponse() const {
        return expect_response_;
    }

public:
    void FromJson(nlohmann::json &json) {
        api_id_ = json["api_id"];
        if (json.contains("expect_response")) {
            expect_response_ = json["expect_response"];
        } else {
            expect_response_ = true;
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["api_id"] = api_id_;
        json["expect_response"] = expect_response_;
        return json;
    }

private:
    int64_t api_id_ = 0;
    bool expect_response_{true};
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__
