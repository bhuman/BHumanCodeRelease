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

#ifndef BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_EXCEPTIONS_EXCEPTION_H_
#define BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_EXCEPTIONS_EXCEPTION_H_

#include <booster_fastdds/fastrtps/fastrtps_dll.h>
#include <exception>
#include <string>
#include <cstdint>

#undef minor

namespace booster_eprosima {
namespace fastrtps {
namespace rtps {

/**
 * @brief This abstract class is used to create exceptions.
 * @ingroup EXCEPTIONMODULE
 */
class Exception : public std::exception
{
    public:

        BOOSTER_RTPS_DllAPI Exception(){};

        /// @brief Default destructor.
        virtual BOOSTER_RTPS_DllAPI ~Exception() throw();

        /**
         * @brief This function returns the number associated with the system exception.
         * @return The number associated with the system exception.
         */
        BOOSTER_RTPS_DllAPI const int32_t& minor() const;

        /**
         * @brief This function sets the number that will be associated with the system exception.
         * @param minor The number that will be associated with the system exception.
         */
        BOOSTER_RTPS_DllAPI void minor(const int32_t &minor);

        /// @brief This function throws the object as exception.
        virtual BOOSTER_RTPS_DllAPI void raise() const = 0;

        /**
         * @brief This function returns the error message.
         * @return The error message.
         */
        virtual BOOSTER_RTPS_DllAPI const char* what() const throw();

    protected:

        /**
         * @brief Default constructor.
         */
        BOOSTER_RTPS_DllAPI explicit Exception(const char* const& message);

        /**
         * @brief Default copy constructor.
         * @param ex Exception that will be copied.
         */
        BOOSTER_RTPS_DllAPI Exception(const Exception &ex);

        /**
         * @brief Default move constructor.
         * @param ex Exception that will be moved.
         */
        BOOSTER_RTPS_DllAPI Exception(Exception&& ex);

        /**
         * @brief Constructor.
         * @param message An error message. This message is copied.
         * @param minor The number that will be associated with the system exception.
         */
        BOOSTER_RTPS_DllAPI explicit Exception(const char* const& message, const int32_t minor);

        /**
         * @brief Assigment operation.
         * @param ex Exception that will be copied.
         */
        BOOSTER_RTPS_DllAPI Exception& operator=(const Exception& ex);

        /**
         * @brief Assigment operation.
         * @param ex Exception that will be moved.
         */
        BOOSTER_RTPS_DllAPI Exception& operator=(Exception&& ex);

    private:

        std::string message_;

        int32_t minor_;
};

} // namespace rtps
} // namespace fastrtps
} // namespace booster_eprosima

#endif // BOOSTER_FASTDDS__BOOSTER_FASTDDS_RTPS_EXCEPTIONS_EXCEPTION_H_
