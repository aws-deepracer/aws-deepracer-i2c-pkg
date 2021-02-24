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

#include "i2c_pkg/i2c.hpp"
#include "rclcpp/rclcpp.hpp"
#include "linux/i2c.h"
#include "linux/i2c-dev.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

namespace BoardChips {
    #define MAX_BUF 64
    #define BYTE_BUFF_SIZE 0x01


    I2C::I2C(int channel, int address, rclcpp::Logger logger_)
     : i2cBus_(channel),
       i2cAddr_(address),
       logger_(logger_)
    {
    }

    int I2C::readByte(uint8_t address) const {
        char i2cPath[MAX_BUF];
        snprintf(i2cPath, sizeof(i2cPath), "/dev/i2c-%d", i2cBus_);

        int fd = open(i2cPath, O_RDWR);
        if (fd < 0) {
            RCLCPP_ERROR(logger_, "Couldn't open I2C Bus %d", i2cBus_);
            return -1;
        }

        if (ioctl(fd, I2C_SLAVE, i2cAddr_) < 0) {
            RCLCPP_ERROR(logger_, "I2C slave %d failed", i2cAddr_);
            close(fd);
            return -1;
        }
        uint8_t buff[BYTE_BUFF_SIZE];
        buff[0] = address;

        if (write(fd, buff, BYTE_BUFF_SIZE) != BYTE_BUFF_SIZE) {
            RCLCPP_ERROR(logger_, "I2C slave 0x%x failed to go to register 0x%x", i2cAddr_, address);
            close(fd);
            return -1;
        }

        memset(buff, 0, sizeof(buff));

        if (read(fd, buff, BYTE_BUFF_SIZE) != BYTE_BUFF_SIZE) {
            RCLCPP_ERROR(logger_, "Could not read from I2C slave 0x%x, register 0x%x", i2cAddr_, address);
            close(fd);
            return -1;
        }
        close(fd);
        return buff[0];
    }
}