#pragma once

#include <string>

#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

enum class AiApiId {
    kStartAiChat = 2000,
    kStopAiChat = 2001,
    kSpeak = 2002,
    kStartFaceTracking = 2003,
    kStopFaceTracking = 2004,
};

class TtsConfig {
public:
    TtsConfig() = default;
    TtsConfig(const std::string &voice_type, const std::vector<int8_t> &ignore_bracket_text) :
        voice_type_(voice_type), ignore_bracket_text_(ignore_bracket_text) {
    }

public:
    void FromJson(nlohmann::json &json) {
        try {
            voice_type_ = json.at("voice_type").get<std::string>();

            auto int_array = json.at("ignore_bracket_text").get<std::vector<int>>();
            ignore_bracket_text_.assign(int_array.begin(), int_array.end());
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("TtsConfig JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"voice_type", voice_type_},
            {"ignore_bracket_text", std::vector<int>(ignore_bracket_text_.begin(), ignore_bracket_text_.end())}};
    }

public:
    //  The timbre (or tone color) can be adjusted, allowing for settings like male or female voices
    //  The following are examples of certain scenarios
    //  zh_male_wennuanahu_moon_bigtts : Supports Chinese and American English.
    //  zh_female_shuangkuaisisi_emo_v2_mars_bigtts : Supports Chinese and British English.
    std::string voice_type_;
    //  Filter out the text enclosed within specified punctuation marks from the large language model's response before performing speech synth//  You need to define in the LLM's prompt which content should be placed within these specified punctuation marks.
    //  Supported values and their meanings are as follows:
    //  1: Chinese parentheses ()
    //  2: English parentheses ()
    //  3: Chinese square brackets 【】
    //  4: English square brackets []
    //  5: English curly braces {}
    std::vector<int8_t> ignore_bracket_text_;
};

class LlmConfig {
public:
    LlmConfig() = default;
    LlmConfig(const std::string &system_prompt, const std::string &welcome_msg, const std::string &prompt_name = "") :
        system_prompt_(system_prompt), welcome_msg_(welcome_msg), prompt_name_(prompt_name) {
    }

    void FromJson(nlohmann::json &json) {
        system_prompt_ = json.at("system_prompt").get<std::string>();
        welcome_msg_ = json.at("welcome_msg").get<std::string>();
        prompt_name_ = json.at("prompt_name").get<std::string>();
    }

    nlohmann::json ToJson() const {
        return {
            {"system_prompt", system_prompt_},
            {"welcome_msg", welcome_msg_},
            {"prompt_name", prompt_name_},
        };
    }

public:
    std::string system_prompt_; // The system prompt defines the character's persona / personality
    std::string welcome_msg_;
    std::string prompt_name_;
};

class AsrConfig {
public:
    AsrConfig() = default;
    AsrConfig(int32_t interrupt_speech_duration, std::vector<std::string> interrupt_keywords) :
        interrupt_speech_duration_(interrupt_speech_duration), interrupt_keywords_(interrupt_keywords) {
    }

    void FromJson(nlohmann::json &json) {
        interrupt_speech_duration_ = json["interrupt_speech_duration"];
        interrupt_keywords_ = json["interrupt_keywords"];
    }

    nlohmann::json ToJson() const {
        return {{"interrupt_speech_duration", interrupt_speech_duration_},
                {"interrupt_keywords", interrupt_keywords_}};
    }

public:
    int32_t interrupt_speech_duration_;           // The threshold for triggering automatic interruption
    std::vector<std::string> interrupt_keywords_; // List of interruption-triggering keywords
};

class StartAiChatParameter {
public:
    StartAiChatParameter() = default;

public:
    void FromJson(nlohmann::json &json) {
        interrupt_mode_ = json["interrupt_mode"];
        enable_face_tracking_ = json["enable_face_tracking"];
        tts_config_.FromJson(json["tts_config"]);
        llm_config_.FromJson(json["llm_config"]);
        asr_config_.FromJson(json["asr_config"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["interrupt_mode"] = interrupt_mode_;
        json["asr_config"] = asr_config_.ToJson();
        json["llm_config"] = llm_config_.ToJson();
        json["tts_config"] = tts_config_.ToJson();
        json["enable_face_tracking"] = enable_face_tracking_;
        return json;
    }

public:
    bool interrupt_mode_ = false; // This mode allows a person to cut off the robot's speech
    TtsConfig tts_config_;
    LlmConfig llm_config_;
    AsrConfig asr_config_;
    bool enable_face_tracking_;
};

class SpeakParameter {
public:
    SpeakParameter() = default;
    SpeakParameter(const std::string &msg) :
        msg_(msg) {
    }

public:
    void FromJson(nlohmann::json &json) {
        msg_ = json["msg"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["msg"] = msg_;
        return json;
    }

public:
    std::string msg_;
};

enum class LuiApiId {
    kStartAsr = 1000,
    kStopAsr = 1001,
    kStartTts = 1050,
    kStopTts = 1051,
    kSendTtsText = 1052,
};

class LuiTtsConfig {
public:
    LuiTtsConfig() = default;
    LuiTtsConfig(const std::string &voice_type) :
        voice_type_(voice_type) {
    }

public:
    void FromJson(nlohmann::json &json) {
        try {
            voice_type_ = json.at("voice_type").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiTtsConfig JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"voice_type", voice_type_}};
    }

public:
    //  The timbre (or tone color) can be adjusted, allowing for settings like male or female voices
    //  The following are examples of certain scenarios
    //  zh_male_wennuanahu_moon_bigtts : Supports Chinese and American English.
    //  zh_female_shuangkuaisisi_emo_v2_mars_bigtts : Supports Chinese and British English.
    std::string voice_type_;
};

class LuiTtsParameter {
public:
    LuiTtsParameter() = default;
    LuiTtsParameter(const std::string &text) :
        text_(text) {
    }

public:
    void FromJson(nlohmann::json &json) {
        try {
            text_ = json.at("text").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiTtsParameter JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"text", text_}};
    }

public:
    std::string text_;
};

}
} // namespace booster::robot
