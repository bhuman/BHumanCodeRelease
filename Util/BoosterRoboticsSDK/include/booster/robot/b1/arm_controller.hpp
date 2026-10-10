#ifndef __BOOSTER_ROBOTICS_SDK_ARM_CONTROLLER_HPP__
#define __BOOSTER_ROBOTICS_SDK_ARM_CONTROLLER_HPP__

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/rpc/error.hpp>

namespace booster {
namespace robot {
namespace b1 {

enum class ArmJoint {
    kLeftPitch = 2,
    kLeftRoll = 3,
    kLeftYaw = 4,
    kLeftElbow = 5,
    kRightPitch = 6,
    kRightRoll = 7,
    kRightYaw = 8,
    kRightElbow = 9,
};

class ArmController {
public:
    explicit ArmController(const std::string &ip = "") :
        low_cmd_publisher_(kTopicJointCtrl) {
        try {
            ChannelFactory::Instance()->Init(0, ip);
            client_.Init();

            low_cmd_publisher_.InitChannel();
            tracker_.InitChannel();
            if (!WaitForTrackerReady(std::chrono::seconds(5))) {
                std::cerr << "[WARN] LowState timeout." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));

            if (!RetryApiCall([this]() { return client_.UpperBodyCustomControl(true); },
                              "Enable UpperBodyCustomControl",
                              5,
                              std::chrono::seconds(1))) {
                std::cerr << "[ERROR] Failed to enable UpperBodyCustomControl after retries."
                          << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } catch (...) {
            tracker_.CloseChannel();
            low_cmd_publisher_.CloseChannel();
            is_closed_ = true;
            throw;
        }
    }

    ~ArmController() {
        Close();
    }

    ArmController(const ArmController &) = delete;
    ArmController &operator=(const ArmController &) = delete;
    ArmController(ArmController &&) = delete;
    ArmController &operator=(ArmController &&) = delete;

    ArmController &ControlArm(ArmJoint joint_idx,
                              float target_rad,
                              float duration_ms) {
        return ControlArm(static_cast<int>(joint_idx), target_rad, duration_ms);
    }

    ArmController &ControlArm(int joint_idx,
                              float target_rad,
                              float duration_ms) {
        if (joint_idx < kArmJointStart || joint_idx > kArmJointEnd) {
            return *this;
        }

        duration_ms = std::max(50.0f, duration_ms);
        pending_actions_[joint_idx] = PendingAction{
            target_rad,
            duration_ms / 1000.0f,
        };
        return *this;
    }

    bool Finish() {
        if (pending_actions_.empty()) {
            return true;
        }

        std::unordered_map<int, PlannedTask> tasks;
        float max_duration = 0.0f;
        for (const auto &action : pending_actions_) {
            const float duration = action.second.duration_seconds;
            max_duration = std::max(max_duration, duration);
            tasks[action.first] = PlannedTask{
                tracker_.GetJointQ(action.first),
                action.second.target_rad,
                duration,
            };
        }

        bool success = true;
        const auto start_time = std::chrono::steady_clock::now();

        while (true) {
            const float elapsed = std::chrono::duration<float>(
                                      std::chrono::steady_clock::now() - start_time)
                                      .count();
            if (elapsed > max_duration + 0.02f) {
                break;
            }

            booster_interface::msg::LowCmd cmd_msg;
            std::vector<booster_interface::msg::MotorCmd> cmd_list(
                kLowLevelJointCount);
            const std::vector<float> current_real_q = tracker_.GetAllQ();

            for (std::size_t i = 0; i < kLowLevelJointCount; ++i) {
                booster_interface::msg::MotorCmd motor;
                motor.mode(0x0A);
                motor.dq(0.0f);
                motor.tau(0.0f);

                auto task_it = tasks.find(static_cast<int>(i));
                if (task_it != tasks.end()) {
                    const PlannedTask &task = task_it->second;
                    const float progress = std::min(1.0f, elapsed / task.duration_seconds);
                    const float target_q =
                        task.start_q + (task.end_q - task.start_q) * progress;
                    motor.q(target_q);
                    motor.kp(60.0f);
                    motor.kd(3.0f);
                } else if (i >= static_cast<std::size_t>(kArmJointStart) && i <= static_cast<std::size_t>(kArmJointEnd)) {
                    motor.q(current_real_q.at(i));
                    motor.kp(60.0f);
                    motor.kd(3.0f);
                } else {
                    motor.q(0.0f);
                    motor.kp(0.0f);
                    motor.kd(0.0f);
                }

                cmd_list[i] = std::move(motor);
            }

            cmd_msg.motor_cmd(std::move(cmd_list));
            success = low_cmd_publisher_.Write(&cmd_msg) && success;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        pending_actions_.clear();
        return success;
    }

    void Close() {
        if (is_closed_) {
            return;
        }

        if (!RetryApiCall([this]() { return client_.UpperBodyCustomControl(false); },
                          "Disable UpperBodyCustomControl",
                          3,
                          std::chrono::seconds(1))) {
            std::cerr << "[WARN] Failed to disable UpperBodyCustomControl."
                      << std::endl;
        }

        tracker_.CloseChannel();
        low_cmd_publisher_.CloseChannel();
        pending_actions_.clear();
        is_closed_ = true;
    }

private:
    class LowStateTracker {
    public:
        LowStateTracker() :
            joint_positions_(kLowLevelJointCount, 0.0f),
            channel_subscriber_(
                kTopicLowState,
                [this](const void *msg) { HandleMessage(msg); }) {
        }

        void InitChannel() {
            channel_subscriber_.InitChannel();
        }

        void CloseChannel() {
            channel_subscriber_.CloseChannel();
        }

        bool Ready() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return ready_;
        }

        float GetJointQ(int joint_idx) const {
            std::lock_guard<std::mutex> lock(mutex_);
            if (joint_idx < 0 || joint_idx >= static_cast<int>(joint_positions_.size())) {
                return 0.0f;
            }
            return joint_positions_[joint_idx];
        }

        std::vector<float> GetAllQ() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return joint_positions_;
        }

    private:
        void HandleMessage(const void *msg) {
            const auto *low_state =
                static_cast<const booster_interface::msg::LowState *>(msg);
            std::lock_guard<std::mutex> lock(mutex_);
            const auto &motor_states = low_state->motor_state_parallel();
            const std::size_t count = std::min(motor_states.size(), joint_positions_.size());
            for (std::size_t i = 0; i < count; ++i) {
                joint_positions_[i] = motor_states[i].q();
            }
            ready_ = true;
        }

    private:
        mutable std::mutex mutex_;
        bool ready_{false};
        std::vector<float> joint_positions_;
        ChannelSubscriber<booster_interface::msg::LowState> channel_subscriber_;
    };

    struct PendingAction {
        float target_rad;
        float duration_seconds;
    };

    struct PlannedTask {
        float start_q;
        float end_q;
        float duration_seconds;
    };

private:
    template <typename Func>
    bool RetryApiCall(Func &&func,
                      const std::string &description,
                      int max_retries,
                      std::chrono::milliseconds delay) {
        for (int attempt = 0; attempt < max_retries; ++attempt) {
            const int32_t ret = func();
            if (ret == kRpcStatusCodeSuccess) {
                return true;
            }
            if (ret == kRpcStatusCodeTimeout) {
                std::cerr << "[WARN] '" << description
                          << "' failed (RPC Timeout 100). Retrying "
                          << (attempt + 1) << "/" << max_retries << "..."
                          << std::endl;
                std::this_thread::sleep_for(delay);
                continue;
            }
            std::cerr << "[ERROR] '" << description
                      << "' failed, code = " << ret << std::endl;
            return false;
        }
        return false;
    }

    bool WaitForTrackerReady(std::chrono::milliseconds timeout) {
        const auto start = std::chrono::steady_clock::now();
        while (!tracker_.Ready()) {
            if (std::chrono::steady_clock::now() - start > timeout) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return true;
    }

private:
    static constexpr int kArmJointStart = 2;
    static constexpr int kArmJointEnd = 9;
    static constexpr std::size_t kLowLevelJointCount = kJointCnt;

    B1LocoClient client_;
    ChannelPublisher<booster_interface::msg::LowCmd> low_cmd_publisher_;
    LowStateTracker tracker_;
    std::unordered_map<int, PendingAction> pending_actions_;
    bool is_closed_{false};
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_ARM_CONTROLLER_HPP__
