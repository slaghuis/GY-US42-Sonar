#ifndef GYUS42_H
#define GYUS42_H

#include <stdio.h>
#include <iostream>        // for output to std::cout
#include <linux/i2c-dev.h> // for the ioctl() function
#include <unistd.h>        // for the read() and write() function
#include <fcntl.h>         // for the open() function include <stdio.h>
#include <string.h>        // for the strlen() function
#include <stdlib.h>        // for exit
#include <sys/ioctl.h>
#include <errno.h>
#include <chrono>
#include <thread>

// Sonar default I2C device address
#define AddressDefault 0x70

// Sonar internal register addresses
#define RANGE_ADDR      0x70
#define RANGE_CMD       81      // 0x51

class Sonar {
  public:
    Sonar();
      bool init();
    uint16_t readDistance();

  private:
    int fd;  //File Descriptor
    uint8_t address;  //I2C address

};

#endif  // GYUS42_H

