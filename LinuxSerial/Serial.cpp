#include "Serial.hpp"

// Allocate memory for static vars
std::unordered_map<int, struct termios> Serial::default_terminal_settings;
std::unordered_map<int, int> Serial::uart_buses;

Serial::Serial() {
    buffer_size = 255;
}


void Serial::Set_BufferSize(int buff_size) {
    buffer_size = buff_size;
}


int8_t Serial::PinInit(uint8_t gpioNum, int pinDirection) {

    FILE *file;

    // Need to EXPORT the pin for the device and it's files to become available
    // FILE *export_file;
    // std::string pin_export = "/sys/class/gpio/export";
    // export_file = fopen(pin_export.c_str(), "w");
    // if (export_file == ((void*)0)) {
    //     std::cerr << "Unable to open 'export' file for pin " << std::to_string(gpioNum) << std::endl;
    //     return -1;
    // }
    // fprintf(export_file, std::to_string(gpioNum).c_str());
    // fclose(export_file);

    // char path[buffer_size];
    // snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/direction", gpioNum);

    // std::string path = "/sys/class/gpio/gpio" + std::to_string(gpioNum) + "/direction"; // Beaglebone
    std::string path = "/sys/class/gpio/gpiochip" + std::to_string(gpioNum) + "/direction"; // Raspberry Pi
    file = fopen(path.c_str(), "w");

    if (file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open file. Cannot initialize pin " << std::to_string(gpioNum) << std::endl;
        return -1;
    }

    if (pinDirection) { fprintf(file, "out"); }
    else { fprintf(file, "in"); }

    fclose(file);
    return 1;
}


int8_t Serial::UARTInit(uint8_t uartNum) {
  
    struct termios settings; // termios: A general API for configuring the I/O characteristics of character devices, including both terminals and UARTs

    std::string path = "/dev/serial" + std::to_string(uartNum);
    if((uart_buses[uartNum] = open(path.c_str(), O_RDWR | O_NDELAY, O_NOCTTY)) < 0) {
        std::cerr << "ERROR: Unable to open file. Cannot initialize serial/uart-" << std::to_string(uartNum) << std::endl;
        return -1;
    }

    // Retrieve & Save current tty terminal settings for uart device
    if (tcgetattr(uart_buses[uartNum], &default_terminal_settings[uartNum]) != 0) {
        std::cerr << "ERROR: Failed to get UART attributes." << std::endl;
        close(uart_buses[uartNum]);
        return -1;
    }

    // Register function to restore terminal settings at exit
    atexit(RestoreTerminal);

    cfsetspeed(&settings, B9600); // Baudrate
    settings.c_cflag &= ~PARENB;  // No parity bit
    settings.c_cflag &= ~CSTOPB;  // 1 stop bit
    settings.c_cflag &= ~CSIZE;   // Clear Data Size bits
    settings.c_cflag |= CS8;      // Set Data Size: 8 data bits
    settings.c_cflag |= (CLOCAL | CREAD); // Ignore Modem Status | Dont ignore Received data
    settings.c_iflag = IGNPAR; // Input Modes: Ignore Parity Errors
    settings.c_oflag &= ~OPOST; // Set raw output mode (disable pre-processing)
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Local Modes: Disable canonical mode & echo
        // Canonical Mode - (line buffering and special character processing)
            // Non-Canonical Mode: Raw data without line editing or special character processing
        // Echo - Echoes typed characters back to the terminal
        // Echoe - Echoes the erase character (like backspace) as a space+backspace
        // Isig - Enables signal characters (Ctrl+C, Ctrl+Z, etc.)


    // Apply Serial Port Settings
    tcflush(uart_buses[uartNum], TCIFLUSH); // TCIFLUSH: Flush uart's input-data buffer
    if (tcsetattr(uart_buses[uartNum], TCSANOW, &settings) != 0) { // TCSANOW: Apply settings immediately
        std::cerr << "ERROR: Failed to set UART attributes." << std::endl;
        close(uart_buses[uartNum]);
        return -1;
    }

    return 1;
}


void Serial::RestoreTerminal() {

    for (const auto [uart_num, fd] : uart_buses) {
        tcsetattr(fd, TCSANOW, &default_terminal_settings[uart_num]);
    }   
}

void Serial::UARTDeInit(uint8_t uartNum) {

    close(uart_buses[uartNum]);
}


int8_t Serial::I2CInit(uint8_t i2cNum, uint8_t slaveAddress) {

    // char path[20];
    // snprintf(path, sizeof(path), "/dev/i2c-%d", i2cNum);
    std::string path = "/dev/i2c-" + std::to_string(i2cNum);
    i2c_buses[i2cNum] = open(path.c_str(), O_RDWR);

    if(i2c_buses[i2cNum] < 0) {
        std::cerr << "ERROR: Unable to open I2C bus. Cannot initialize i2c-" << std::to_string(i2cNum) << "(" <<strerror(errno) << ")" << std::endl;
        return -1;
    }
	
	// Setup I2C Bus
	if (ioctl(i2c_buses[i2cNum], I2C_SLAVE, slaveAddress) < 0) {
		std::cerr << "ERROR: Failed to connect to I2C device at slave address 0x" << std::hex << (int)slaveAddress 
            << "(" <<strerror(errno) << ")" << std::endl;
        close(i2c_buses[i2cNum]);
        return -1;
	}

    return 1;
}

void Serial::I2CDeInit(uint8_t i2cNum) {

    close(i2c_buses[i2cNum]);
}

int8_t Serial::SPIInit(uint8_t spiNum, uint8_t spi_mode, uint32_t speed_hz, uint8_t spi_bits) {

    mode = spi_mode;
    speed = speed_hz;
    bits = spi_bits;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), "/dev/spidev0.%d", spiNum);
    std::string path = "/dev/spidev0." + std::to_string(spiNum);
    
    if(spi_buses[spiNum] = open(path.c_str(), O_RDWR) < 0) {
        std::cerr << "ERROR: Unable to open file. Cannot initialize spi-" << std::to_string(spiNum) << std::endl;
        return -1;
    }
	
    // Setup the SPI Bus----------------------------------------------
	if (ioctl(spi_buses[spiNum], SPI_IOC_WR_MODE, &mode) < 0) {
		std::cerr << "ERROR: Failed to set SPI Mode" << std::endl;
        close(spi_buses[spiNum]);
        return -1;
	}

    if (ioctl(spi_buses[spiNum], SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		std::cerr << "ERROR: Failed to set SPI Mode" << std::endl;
        close(spi_buses[spiNum]);
        return -1;
	}

    if (ioctl(spi_buses[spiNum], SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		std::cerr << "ERROR: Failed to set SPI Mode" << std::endl;
        close(spi_buses[spiNum]);
        return -1;
	}

    return 1;
}


void Serial::SPIDeInit(uint8_t spiNum) {

    close(spi_buses[spiNum]);
}


int8_t Serial::PinWrite(uint8_t gpioNum, uint8_t pinState) {

    FILE *file;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);

    std::string path = "/sys/class/gpio/gpio" + std::to_string(gpioNum) + "/value";
    file = fopen(path.c_str(), "w");

    if (file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open file. Cannot set state of pin " << std::to_string(gpioNum) << std::endl;
        return -1;
    }

    if (pinState != 1 && pinState != 0) { std::cerr << "ERROR: Invalid Pin State. HIGH = 1, LOW = 0" << std::endl; }
    else { fprintf(file, "%d", pinState); }

    fclose(file);
    return 1;

}


int8_t Serial::PinRead(uint8_t gpioNum) {
    
    FILE *file;
    char* pinState;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);
    std::string path = "/sys/class/gpio/gpio" + std::to_string(gpioNum) + "/value";
    file = fopen(path.c_str(), "r");

    if (file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open file. Cannot determine state of pin " << std::to_string(gpioNum) << std::endl;
        return -1;
    }

    fgets(pinState, 1, file);

    fclose(file);
    return atoi(pinState);
}



int8_t Serial::UARTWrite(uint8_t uartNum, char* data, int datalen) {

    if (write(uart_buses[uartNum], data, datalen) != datalen) {
        std::cerr << "ERROR: Failed to write" << std::endl;
        return 0;
    }
    return 1;
}


int8_t Serial::UARTRead(uint8_t uartNum, char* data, int datalen) {

    if (write(uart_buses[uartNum], data, datalen) != datalen) {
        std::cerr << "ERROR: Failed to write" << std::endl;
        return 0;
    }
    return 1;
}


int8_t Serial::I2CWrite(uint8_t i2cNum, uint8_t *dataBytes, int byteNum) {
    
    int bytesWritten = write(i2c_buses[i2cNum], dataBytes, byteNum);
    
    if (bytesWritten < 0) {
        std::cerr << "ERROR: Failed to write to I2C bus: " << "(" <<strerror(errno) << ")" << std::endl;
        return 0;
    } 
    else if (bytesWritten != byteNum) {
        std::cerr << "WARNING: Expected to write " << byteNum << " bytes. Only wrote " << bytesWritten << std::endl;
    }
        
    return 1;
}


int8_t Serial::I2CRead(uint8_t i2cNum, uint8_t *dataBytes, int byteNum) {

    int bytesRead = read(i2c_buses[i2cNum], dataBytes, byteNum);
    
    if (bytesRead < 0) {
        std::cerr << "ERROR: Failed to read from I2C bus: " << "(" <<strerror(errno) << ")" << std::endl;
        return 0;
    } 
    else if (bytesRead != byteNum) {
        std::cerr << "WARNING: Expected " << byteNum << " bytes but received " << bytesRead << std::endl;
    }
        
    return 1;

    // Interesting Alternate Version [using ioctrl] (Also Works)-------------------------------------------------
    // struct i2c_rdwr_ioctl_data packets;
    // struct i2c_msg messages[1];

    // // i2c Transaction Settings
    // messages[0].addr = 0x55;
    // messages[0].flags = I2C_M_RD;  // Read operation
    // messages[0].len = BUFFER_SIZE;
    // messages[0].buf = dataBytes;

    // packets.msgs = messages;
    // packets.nmsgs = 1;

    // if (ioctl(i2c_buses[i2cNum], I2C_RDWR, &packets) < 0) {
    //     std::cerr << "I2C Bus Read failed: " << "(" <<strerror(errno) << ")" << std::endl;
    //     close(i2c_buses[i2cNum]);
    //     return -1;
    // }

    // return 1;
}



int Serial::SPITransfer(uint8_t spiNum, int8_t *dataBytes, int len) {

    struct spi_ioc_transfer spi_info[len];

    // Set up the transfer struct
    for (int i = 0; i < len; i++) {
        
        // Initialize w/ all 0 values
        memset(&spi_info[i], 0, sizeof(struct spi_ioc_transfer));

        spi_info[i].tx_buf = (unsigned long) (dataBytes + i);
        spi_info[i].rx_buf = (unsigned long) (dataBytes + i);
        spi_info[i].len = 1;
        spi_info[i].speed_hz = speed;
        spi_info[i].bits_per_word = bits;
    }

    // Transfer Data
    if (ioctl(spi_buses[spiNum], SPI_IOC_MESSAGE(len), spi_info) < 0) {
        std::cerr << "ERROR: Failed transfer data over SPI" << std::endl;
        close(spi_buses[spiNum]);
        return -1;
    }

    return 0;
}


int Serial::PWMInit(uint8_t chipNum, uint8_t pwmNum, uint64_t period, uint64_t duty_cycle) {

    FILE *enable_file;
    FILE *period_file;
    FILE *dutycycle_file;

    std::string pwm_export = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/export";
    std::string pwm_enable = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/pwm" + std::to_string(pwmNum) + "/enable";
    std::string pwm_period = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/pwm" + std::to_string(pwmNum) + "/period";
    std::string pwm_dutycycle = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/pwm"  + std::to_string(pwmNum) + "/duty_cycle";

    // Need to EXPORT the pwm device for its files (period, dutycycle) to become available
    FILE *export_file;
    export_file = fopen(pwm_export.c_str(), "w");
    if (export_file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open 'export' file for chip " << std::to_string(chipNum) << std::endl;
        return -1;
    }
    fprintf(export_file, std::to_string(pwmNum).c_str());
    fclose(export_file);

    enable_file = fopen(pwm_enable.c_str(), "w");
    period_file = fopen(pwm_period.c_str(), "w");
    dutycycle_file = fopen(pwm_dutycycle.c_str(), "w");


    if (enable_file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open 'enable' file for chip " << std::to_string(chipNum) << std::endl;
        return -1;
    }

    if (period_file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open 'period' file for chip " << std::to_string(chipNum) << "pwm " << std::to_string(pwmNum) << std::endl;
        return -1;
    }

    if (dutycycle_file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open 'duty_cycle' file for chip " << std::to_string(chipNum) << "pwm " << std::to_string(pwmNum) << std::endl;
        return -1;
    }

    fprintf(enable_file, "1");
    fprintf(period_file, std::to_string(period).c_str());
    fprintf(dutycycle_file, std::to_string(duty_cycle).c_str());
 
    fclose(enable_file);
    fclose(period_file);
    fclose(dutycycle_file);
    return 1;

}

int Serial::PWMDeInit(uint8_t chipNum, uint8_t pwmNum) {

    // FILE *unexport_file;

    // std::string pwm_unexport = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/unexport";

    // unexport_file = fopen(pwm_unexport.c_str(), "w");

    // if (unexport_file == ((void*)0)) {
    //     std::cerr << "Unable to open 'export' file for chip " << std::to_string(chipNum) << std::endl;
    //     return -1;
    // }

    // fprintf(unexport_file, "0");

    // fclose(unexport_file);
    return 1;

}

int Serial::PWMUpdate(uint8_t chipNum, uint8_t pwmNum, uint64_t duty_cycle) {
    
    FILE *dutycycle_file;
    std::string pwm_dutycycle = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/pwm"  + std::to_string(pwmNum) + "/duty_cycle";
    dutycycle_file = fopen(pwm_dutycycle.c_str(), "w");

    if (dutycycle_file == ((void*)0)) {
        std::cerr << "ERROR: Unable to open 'duty_cycle' file for chip " << std::to_string(chipNum) << " pwm " << std::to_string(pwmNum) << std::endl;
        return -1;
    }

    fprintf(dutycycle_file, std::to_string(duty_cycle).c_str());

    fclose(dutycycle_file);
    return 1;
}




