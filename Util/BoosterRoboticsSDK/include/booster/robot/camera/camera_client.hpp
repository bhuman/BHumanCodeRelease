#pragma once

#include <memory>
#include <string>

#include <booster/robot/camera/camera_api.hpp>
#include <booster/robot/common/device_info.hpp>
#include <booster/robot/rpc/rpc_client.hpp>
#include <booster/robot/rpc/response.hpp>

namespace booster {
namespace robot {
namespace camera {

class CameraClient {
public:
    CameraClient() = default;
    ~CameraClient() = default;

    void Init();
    void Init(const std::string &robot_name);

    int32_t SendApiRequest(CameraApiId api_id, const std::string &param);
    int32_t SendApiRequestWithResponse(CameraApiId api_id, const std::string &param,
                                       booster::robot::Response &resp);

    /**
     * @brief Query camera catalog. RPC JSON uses field "cameras".
     * Use booster::robot::CameraListFromDeviceInfo(list, info) (see device_info_parser.hpp).
     *
     * @param[out] info kind_ is kCamera; json_ holds RPC body.
     * @return 0 if success, otherwise error code
     */
    int32_t GetCameras(b1::DeviceInfo &info);

private:
    std::shared_ptr<booster::robot::RpcClient> rpc_client_;
};

} // namespace camera
} // namespace robot
} // namespace booster
