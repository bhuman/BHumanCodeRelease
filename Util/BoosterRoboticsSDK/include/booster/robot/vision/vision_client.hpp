#pragma once

#include <memory>
#include <vector>
#include <string>

#include <booster/robot/rpc/rpc_client.hpp>
#include "vision_api.hpp"

namespace booster {
namespace robot {
namespace vision {

class VisionClient {
public:
    VisionClient() = default;
    ~VisionClient() = default;

    void Init();
    void Init(const std::string &robot_name);

    int32_t SendApiRequest(VisionApiId api_id, const std::string &param);
    int32_t SendApiRequestWithResponse(VisionApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief 1. Start Vision Service
     */
    int32_t StartVisionService(bool enable_position, bool enable_color, bool enable_face_detection) {
        StartVisionServiceParameter param_obj(enable_position, enable_color, enable_face_detection);
        std::string param = param_obj.ToJson().dump();
        return SendApiRequest(VisionApiId::kStartVisionService, param);
    }

    /**
     * @brief 2. Stop Vision Service
     */
    int32_t StopVisionService() {
        return SendApiRequest(VisionApiId::kStopVisionService, "{}");
    }

    /**
     * @brief 3. Get Detection Objects
     * * @param objects [Out] 输出的检测结果列表
     * @param ratio   [In]  聚焦比例 (默认 0.33f) -> 必须放在最后才能设默认值
     */
    int32_t GetDetectionObject(std::vector<DetectResults> &objects, float ratio = 0.33f) {
        // 使用传入的 ratio 构造参数 (如果是默认调用，ratio 就是 0.33)
        GetDetectionObjectParameter detect_param(ratio);
        std::string param = detect_param.ToJson().dump();

        Response resp;
        int32_t ret = SendApiRequestWithResponse(VisionApiId::kGetDetectionObject, param, resp);

        if (ret != 0) {
            return ret;
        }

        try {
            nlohmann::json body_json = nlohmann::json::parse(resp.GetBody());
            objects.clear();

            if (body_json.is_array()) {
                for (const auto &item_json : body_json) {
                    DetectResults item;
                    item.FromJson(item_json);
                    objects.push_back(item);
                }
            } else if (body_json.contains("objects") && body_json["objects"].is_array()) {
                for (const auto &item_json : body_json["objects"]) {
                    DetectResults item;
                    item.FromJson(item_json);
                    objects.push_back(item);
                }
            }
        } catch (const std::exception &e) {
            return -1;
        }

        return 0;
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::vision
