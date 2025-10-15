#ifndef __BOOSTER_ROBOTICS_SDK_ERROR_HPP__
#define __BOOSTER_ROBOTICS_SDK_ERROR_HPP__

#include <cstdint>
namespace booster {
namespace robot {

const int64_t kRpcStatusCodeSuccess = 0;                 // success
const int64_t kRpcStatusCodeTimeout = 100;               // request timeout
const int64_t kRpcStatusCodeBadRequest = 400;            // bad request, usually when the request param is invalid
const int64_t kRpcStatusCodeInternalServerError = 500;   // internal server error
const int64_t kRpcStatusCodeServerRefused = 501;         // server refused the request
const int64_t kRpcStatusCodeStateTransitionFailed = 502; // robot state machine transition failed
const int64_t kRpcStatusCodeInvalid = -1;                // default value, usually when the request has not been sent yet

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_ERROR_HPP__