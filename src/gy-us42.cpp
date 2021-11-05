// Copyright (c) 2021 Xeni Robotics
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

/* **********************************************************************
 * Library to drive data from the GY-US42 Ultrasonic Rangefinder 
 * Reduced functionality, just enough to take a range measurement
 * ***********************************************************************/

#include <gy-us42/gy-us42.h>

// Constructor ////////////////////////////////////////////
Sonar::Sonar()
  : address(AddressDefault)
{ }

// Public Methods ////////////////////////////////////////
//Open the i2c port for reading and writing
bool Sonar::init() {
  // Open it i2c port
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
      return false;
  }

  // Setup flow control
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    return false;
  }
  
  return true;
}

/*------------------------------------------------------------------------------
  Read Distance
  Initiate a distance measurement by writing 0x51 to register 0x70.
  Read and return result of distance measurement.
------------------------------------------------------------------------------*/
uint16_t Sonar::readDistance()
{
  using namespace std::this_thread;     // sleep_for, sleep_until
  using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

  uint8_t  writeBuffer[1];
  writeBuffer[0] = RANGE_CMD;

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < (long int)sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "ERROR on write to i2c" << std::endl;
    return 0x0000;
  }

  sleep_for(100ms);

  uint8_t readBuffer[2];
  if( read(fd, readBuffer, sizeof(readBuffer)) < (long int)sizeof(readBuffer)) {
    std::cout << "Error on read from i2c" << std::endl;
    return 0x0000;
  }

  // The high byte is by default 128.  The moment the range is over 255 cm (more than one
  // byte, the high byte increases to 129, 130, etc.  I applied this logic in here.
  // I do not have a data sheet to explain this anomaly.
  uint16_t value = ((uint16_t)readBuffer[0]-128) * 100 + (uint16_t)readBuffer[1];

  return value;
}


