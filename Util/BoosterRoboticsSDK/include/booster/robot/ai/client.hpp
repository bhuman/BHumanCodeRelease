#pragma once

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>

#include "api.hpp"

namespace booster {
namespace robot {

class AiClient {
public:
    AiClient() = default;
    ~AiClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    /**
     * @brief Send API request to AI chat service
     *
     * @param api_id API_ID, you can find the API_ID in api.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(AiApiId api_id, const std::string &param);

    /**
     * @brief Send API request to AI chat service with response
     *
     * @param api_id API_ID, you can find the API_ID in api.hpp
     * @param param API parameter
     * @param resp [out] A reference to a Response object where the API's response will be stored.
     * This parameter is modified by the function to contain the result of the API call
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequestWithResponse(AiApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief Start AI chat session with the robot
     *
     * This function sends a request to initiate an AI chat session on the robot.
     *
     * @param param The parameters required to start the AI chat, serialized to JSON.
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t StartAiChat(const StartAiChatParameter &param) {
        std::string body = param.ToJson().dump();
        return SendApiRequest(AiApiId::kStartAiChat, body);
    }

    /**
     * @brief Stop ai chat
     *
     * @return 0 if success, otherwise return error code
     */

    int32_t StopAiChat() {
        return SendApiRequest(AiApiId::kStopAiChat, "");
    }

    /**
     * @brief Send a speech command to the robot
     *
     * @param param The parameter object containing speech details
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t Speak(const SpeakParameter &param) {
        std::string body = param.ToJson().dump();
        return SendApiRequest(AiApiId::kSpeak, body);
    }

    /**
     * @brief Start face tracking
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StartFaceTracking() {
        return SendApiRequest(AiApiId::kStartFaceTracking, "");
    }

    /**
     * @brief Start face tracking
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StopFaceTracking() {
        return SendApiRequest(AiApiId::kStopFaceTracking, "");
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

class LuiClient {
public:
    LuiClient() = default;
    ~LuiClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    /**
     * @brief Send API request to LUI service
     *
     * @param api_id API_ID, you can find the API_ID in api.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(LuiApiId api_id, const std::string &param);

    /**
     * @brief Send API request to LUI service with response
     *
     * @param api_id API_ID, you can find the API_ID in api.hpp
     * @param param API parameter
     * @param resp [out] A reference to a Response object where the API's response will be stored.
     * This parameter is modified by the function to contain the result of the API call
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequestWithResponse(LuiApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief Start ASR service
     *
     * This function sends a request to initiate an ASR service on the robot.
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t StartAsr() {
        return SendApiRequest(LuiApiId::kStartAsr, "");
    }

    /**
     * @brief Stop ASR service
     *
     * This function sends a request to stop the ASR service on the robot.
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t StopAsr() {
        return SendApiRequest(LuiApiId::kStopAsr, "");
    }

    /**
     * @brief Start TTS service
     *
     * This function sends a request to initiate a TTS service on the robot.
     *
     * @param config The configuration object containing TTS details
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t StartTts(const LuiTtsConfig &config) {
        std::string body = config.ToJson().dump();
        return SendApiRequest(LuiApiId::kStartTts, body);
    }

    /**
     * @brief Stop TTS service
     *
     * This function sends a request to stop the TTS service on the robot.
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t StopTts() {
        return SendApiRequest(LuiApiId::kStopTts, "");
    }

    /**
     * @brief Send TTS text to the robot
     *
     * This function sends a request to the robot to synthesize and play the specified text using TTS.
     *
     * @param param The parameter object containing the text to be synthesized.
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t SendTtsText(const LuiTtsParameter &param) {
        std::string body = param.ToJson().dump();
        return SendApiRequest(LuiApiId::kSendTtsText, body);
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
} // namespace booster::robot
