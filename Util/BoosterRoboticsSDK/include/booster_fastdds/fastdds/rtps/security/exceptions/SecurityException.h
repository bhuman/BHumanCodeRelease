// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file SecurityException.h
 */
#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_SECURITY_EXCEPTIONS_SECURITYEXCEPTION_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_SECURITY_EXCEPTIONS_SECURITYEXCEPTION_H_

#include <booster_fastdds/fastdds/rtps/exceptions/Exception.h>

namespace booster_eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

/**
 * @brief This class is thrown as an exception when there is an error in security module.
 * @ingroup EXCEPTIONMODULE
 */
class SecurityException : public Exception
{
    public:

        BOOSTER_RTPS_DllAPI SecurityException() {}

        /**
         * @brief Default constructor.
         * @param message An error message. This message is copied.
         */
        BOOSTER_RTPS_DllAPI SecurityException(const std::string& message) : Exception(message.c_str(), 1) {}

        /**
         * @brief Default copy constructor.
         * @param ex SecurityException that will be copied.
         */
        BOOSTER_RTPS_DllAPI SecurityException(const SecurityException &ex);

        /**
         * @brief Default move constructor.
         * @param ex SecurityException that will be moved.
         */
        BOOSTER_RTPS_DllAPI SecurityException(SecurityException&& ex);

        /**
         * @brief Assigment operation.
         * @param ex SecurityException that will be copied.
         */
        BOOSTER_RTPS_DllAPI SecurityException& operator=(const SecurityException &ex);

        /**
         * @brief Assigment operation.
         * @param ex SecurityException that will be moved.
         */
        BOOSTER_RTPS_DllAPI SecurityException& operator=(SecurityException&& ex);

        /// \brief Default constructor
        virtual BOOSTER_RTPS_DllAPI ~SecurityException() throw();

        /// \brief This function throws the object as an exception.
        virtual BOOSTER_RTPS_DllAPI void raise() const;
};
} // namespace security
} // namespace rtps
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_SECURITY_EXCEPTIONS_SECURITYEXCEPTION_H_
