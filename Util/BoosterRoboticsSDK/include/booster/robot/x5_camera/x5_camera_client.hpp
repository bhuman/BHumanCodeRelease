#ifndef BOOSTER_ROBOTICS_SDK_X5_CAMERA_CLIENT_HPP
#define BOOSTER_ROBOTICS_SDK_X5_CAMERA_CLIENT_HPP

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>
#include <booster/robot/x5_camera/x5_camera_api.hpp>

namespace booster {
namespace robot {
namespace x5_camera {

class X5CameraClient {
public:
    X5CameraClient() = default;
    ~X5CameraClient() = default;

    void Init();

    /**
     * @brief Send API request to X5 Camera 
     *
     * @param api_id API_ID, you can find the API_ID in x5_camera_const.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(X5CameraApiId api_id, const std::string &param);

    /**
     * @brief Send API request to X5 Camera with response
     *
     * @param api_id API_ID, you can find the API_ID in x5_camera_api_const.hpp
     * @param param API parameter
     * @param resp [out] A reference to a Response object where the API's response will be stored.
     * This parameter is modified by the function to contain the result of the API call
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequestWithResponse(X5CameraApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief Change Camera mode
     *
     * @param mode Camera mode, options are:
     *   kCameraModeNormal,
     *   kCameraModeHighResolution,
     *   kCameraModeNormalEnable,
     *   kCameraModeHighResolutionEnable,
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t ChangeMode(CameraSetMode mode) {
        ChangeModeParameter change_mode(mode);
        std::string param = change_mode.ToJson().dump();
        return SendApiRequest(X5CameraApiId::kChangeMode, param);
    }

    /**
     * @brief get status
     *
     * @param get_status_response [out] A reference to a Response object where the API's response will be stored.
     *    kCameraStatusNormal = 0,
     *    kCameraStatusHighResolution = 1,
     *    kCameraStatusError = 2,
     *    kCameraStatusNull = 3,
     * @return 0 if success, otherwise return error code
     */
    int32_t GetStatus(GetStatusResponse &get_status_response) {
        std::string param{};
        Response resp;
        int32_t ret = SendApiRequestWithResponse(X5CameraApiId::kGetStatus,
                                                 param, resp);
        if (ret != 0) {
            return ret;
        }
        nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
        CameraSetMode status = static_cast<CameraSetMode>(body_json["status"]);
        get_status_response.FromJson(body_json);
        return ret;
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot

#endif
