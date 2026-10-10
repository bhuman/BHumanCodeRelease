#ifndef __BOOSTER_ROBOTICS_SDK_ERROR_HPP__
#define __BOOSTER_ROBOTICS_SDK_ERROR_HPP__

#include <cstdint>
namespace booster {
namespace robot {

const int64_t kRpcStatusCodeInvalid = -1;                // default value, usually when the request has not been sent yet	
const int64_t kRpcStatusCodeSuccess = 0;                 // success
const int64_t kRpcStatusCodeTimeout = 100;               // request timeout
const int64_t kRpcStatusCodeBadRequest = 400;            // bad request, usually when the request param is invalid
const int64_t kRpcStatusCodeConflict = 409;              // conflict, usually indicates that the request could not be completed due to a conflict with the current state of the target resource
const int64_t kRpcStatusCodeRequestTooFrequent = 429;    // request too frequent
const int64_t kRpcStatusCodeInternalServerError = 500;   // internal server error
const int64_t kRpcStatusCodeServerRefused = 501;         // server refused the request
const int64_t kRpcStatusCodeStateTransitionFailed = 502; // robot state machine transition failed

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_ERROR_HPP__
