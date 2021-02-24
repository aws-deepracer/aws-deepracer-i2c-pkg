///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#ifndef I2C_HPP
#define I2C_HPP

#include <cinttypes>
#include "rclcpp/rclcpp.hpp"

namespace BoardChips {
    class I2C 
    {
    public:
        /// @param channel Bus channel where the ADC is located.
        /// @param address Slave address for which to read the ADC.
        I2C(int channel, int address, rclcpp::Logger logger_);
        ~I2C() = default;
        /// Method for reading the ADC from the bus.
        /// @param address For the ADC register.
        int readByte(uint8_t address) const;
        //! TODO implement a write byte if needed.

    private:
        /// Bus channel.
        const int i2cBus_;
        /// Bus address
        const int i2cAddr_;
        /// ROS Logger object to log messages.
        rclcpp::Logger logger_;
    };
}
#endif