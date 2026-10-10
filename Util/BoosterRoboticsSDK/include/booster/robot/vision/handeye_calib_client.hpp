#pragma once

#include <memory>
#include <string>

#include <booster/robot/rpc/rpc_client.hpp>

#include "handeye_calib_api.hpp"

namespace booster {
namespace robot {
namespace vision {

class HandEyeCalibClient {
public:
    HandEyeCalibClient() = default;
    ~HandEyeCalibClient() = default;

    void Init();
    void Init(const std::string &robot_name);

    int32_t SendApiRequest(HandEyeCalibApiId api_id, const std::string &param);
    int32_t SendApiRequestWithResponse(
        HandEyeCalibApiId api_id,
        const std::string &param,
        Response &resp);

    int32_t StartCalibration(const StartHandEyeCalibParameter &param);
    int32_t StopCalibration();
    int32_t GetStatus(HandEyeCalibStatus &status);
    int32_t GetResult(HandEyeCalibResult &result);
    int32_t ApplyResult(HandEyeCalibApplyResult &result);

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::vision
