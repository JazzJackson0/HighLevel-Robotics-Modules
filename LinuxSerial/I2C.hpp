#pragma once

#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <string>
#include <thread>
#include <iostream>
#include <stdio.h>
#include <string.h>
using namespace std;

#define BUFFER_SIZE 100