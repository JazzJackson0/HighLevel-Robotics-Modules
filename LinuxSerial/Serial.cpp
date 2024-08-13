#include "Serial.hpp"

Serial::Serial() {
    buffer_size = 255;
}


void Serial::Set_BufferSize(int buff_size) {
    buffer_size = buff_size;
}


int8_t Serial::PinInit(uint8_t gpioNum, int pinDirection) {

    FILE *file;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/direction", gpioNum);

    std::string path = "/gpio" + std::to_string(gpioNum) + "/direction";
    const char *raw_path = path.c_str();

    file = fopen(raw_path, "w");

    if (file == ((void*)0)) {
        std::cerr << "Unable to open file. Cannot initialize pin " << std::to_string(gpioNum) << std::endl;
        return -1;
    }

    if (pinDirection) { fprintf(file, "out"); }
    else { fprintf(file, "in"); }

    fclose(file);
    return 1;
}


int8_t Serial::UARTInit(uint8_t uartNum) {
  
    struct termios settings; // Serial settings
    int uart;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), "/dev/serial.%d", uartNum);

    std::string path = "/dev/serial." + std::to_string(uartNum);
    const char *raw_path = path.c_str();
    
    if(uart = open(raw_path, O_RDWR | O_NDELAY, O_NOCTTY) < 0) {
        std::cerr << "Unable to open file. Cannot initialize serial/uart-" << std::to_string(uartNum) << std::endl;
        return -1;
    }
	
    // Get Serial Port Settings
	// tcgetattr(uart, &settings);

    // Set Serial Port Settings
    //cfsetspeed(&settings, B9600);
    settings.c_cflag = B9600 | CS8 | CLOCAL | CREAD; // Baudrate | Size(Bits) | Ignore Modem Status | Set Receiver
    settings.c_iflag = IGNPAR; // Input Flags -> Ignore Parity Errors
    settings.c_oflag = 0; // Ouptut flags
    settings.c_lflag = 0; // Local flags

    // Apply Serial Port Settings
    tcflush(uart, TCIFLUSH); // TCIFLUSH: Flush the INPUT buffer
    tcsetattr(uart, TCSANOW, &settings); // TCSANOW: Apply settings immediately

    return uart;
}

void UARTDeInit(uint8_t uartNum) {

    close(uartNum);
}


int8_t Serial::I2CInit(uint8_t i2cNum, uint8_t slaveAddress) {

    // char path[20];
    // snprintf(path, sizeof(path), "/dev/i2c-%d", i2cNum);
    std::string path = "/dev/i2c-" + std::to_string(i2cNum);
    const char *raw_path = path.c_str();
    int8_t i2c_bus = open(raw_path, O_RDWR);

    if(i2c_bus < 0) {
        std::cerr << "Unable to open file. Cannot initialize i2c-" << std::to_string(i2cNum) << std::endl;
        return -1;
    }
	
	// Setup I2C Bus
	if (ioctl(i2c_bus, I2C_SLAVE, slaveAddress) < 0) {
		std::cerr << "Failed to connect to I2C Bus" << std::endl;
        close(i2c_bus);
        return -1;
	}

    return i2c_bus;
}

void I2CDeInit(uint8_t i2cNum) {

    close(i2cNum);
}

int8_t Serial::SPIInit(uint8_t spiNum, uint8_t spi_mode, uint32_t speed_hz, uint8_t spi_bits) {

    mode = spi_mode;
    speed = speed_hz;
    bits = spi_bits;
    int spi_bus;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), "/dev/spidev0.%d", spiNum);

    std::string path = "/dev/spidev0." + std::to_string(spiNum);
    const char *raw_path = path.c_str();
    
    if(spi_bus = open(raw_path, O_RDWR) < 0) {
        std::cerr << "Unable to open file. Cannot initialize spi-" << std::to_string(spiNum) << std::endl;
        return -1;
    }
	
    // Setup the SPI Bus----------------------------------------------
	if (ioctl(spi_bus, SPI_IOC_WR_MODE, &mode) < 0) {
		std::cerr << "Failed to set SPI Mode" << std::endl;
        close(spi_bus);
        return -1;
	}

    if (ioctl(spi_bus, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		std::cerr << "Failed to set SPI Mode" << std::endl;
        close(spi_bus);
        return -1;
	}

    if (ioctl(spi_bus, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		std::cerr << "Failed to set SPI Mode" << std::endl;
        close(spi_bus);
        return -1;
	}

    return spi_bus;
}


void SPIDeInit(uint8_t spiNum) {

    close(spiNum);
}


int8_t Serial::PinWrite(uint8_t gpioNum, uint8_t pinState) {

    FILE *file;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);

    std::string path = "/gpio" + std::to_string(gpioNum) + "/value";
    const char *raw_path = path.c_str();
    file = fopen(raw_path, "w");

    if (file == ((void*)0)) {
        std::cerr << "Unable to open file. Cannot set state of pin " << std::to_string(gpioNum) << std::endl;
        return -1;
    }

    if (pinState != 1 && pinState != 0) { std::cerr << "Invalid Pin State. HIGH = 1, LOW = 0" << std::endl; }
    else { fprintf(file, "%d", pinState); }

    fclose(file);
    return 1;

}


int8_t Serial::PinRead(uint8_t gpioNum) {
    
    FILE *file;
    char* pinState;

    // char path[buffer_size];
    // snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);
    std::string path = "/gpio" + std::to_string(gpioNum) + "/value";
    const char *raw_path = path.c_str();
    file = fopen(raw_path, "r");

    if (file == ((void*)0)) {
        std::cerr << "Unable to open file. Cannot determine state of pin " << std::to_string(gpioNum) << std::endl;
        return -1;
    }

    fgets(pinState, 1, file);

    fclose(file);
    return atoi(pinState);
}



int8_t Serial::UARTWrite(int uart, char* data, int datalen) {

    if(write(uart, data, datalen) != datalen){
        std::cerr << "Failed to write" << std::endl;
        return 0;
    }
    return 1;
}


int8_t Serial::UARTRead(int uart, char* data, int datalen) {

    if(write(uart, data, datalen) != datalen){
        std::cerr << "Failed to write" << std::endl;
        return 0;
    }
    return 1;
}


int8_t Serial::I2CWrite(int8_t &i2c_bus, int8_t *dataBytes, int byteNum) {
    
    if(write(i2c_bus, dataBytes, byteNum) != byteNum){
        std::cerr << "Failed to write" << std::endl;
        return 0;
    }
    return 1;
}


int8_t Serial::I2CRead(int8_t &i2c_bus, int8_t *dataBytes, int byteNum) {

    if (read(i2c_bus, dataBytes, byteNum) != byteNum) {
        std::cerr << "Failed to read" << std::endl;
        return 0;
    }
    return 1;
}



int Serial::SPITransfer(int8_t spi_bus, int8_t *dataBytes, int len) {

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
    if (ioctl(spi_bus, SPI_IOC_MESSAGE(len), spi_info) < 0) {
        std::cerr << "Failed transfer data over SPI" << std::endl;
        close(spi_bus);
        return -1;
    }

    return 0;
}




