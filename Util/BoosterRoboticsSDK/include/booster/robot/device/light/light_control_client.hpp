#pragma once

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>

#include "light_control_api.hpp"

namespace booster {
namespace robot {
namespace light {

class LightControlClient {
public:
    LightControlClient() = default;
    ~LightControlClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    /**
     * @brief Send API request to B1 robot
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(LightApiId api_id, const std::string &param);

    /**
     * @brief Send API request to B1 robot with response
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     * @param resp [out] A reference to a Response object where the API's response will be stored.
     * This parameter is modified by the function to contain the result of the API call
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequestWithResponse(LightApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief Set the color of the LED light
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SetLEDLightColor(uint8_t r, uint8_t g, uint8_t b) {
        SetLEDLightColorParameter set_color(r, g, b);
        std::string param = set_color.ToJson().dump();
        return SendApiRequest(LightApiId::kSetLEDLightColor, param);
    }

    int32_t SetLEDLightColors(const std::vector<SetLEDLightColorParameter> &colors) {
        SetLEDLightColorsParameter set_colors(colors);
        std::string param = set_colors.ToJson().dump();
        return SendApiRequest(LightApiId::kSetLEDLightColors, param);
    }

    /**
     * @brief Stop the LED light
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StopLEDLightControl() {
        return SendApiRequest(LightApiId::kStopLEDLightControl, "");
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::light
