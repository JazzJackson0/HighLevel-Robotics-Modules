#include "Serial.hpp"

// TODO: Need to test


Serial::Serial() {
    buffer_size = 100;
}


void Serial::Set_BufferSize(int buff_size) {
    buffer_size = buff_size;
}


int8_t Serial::PinInit(uint8_t gpioNum, int pinDirection) {

    char path[buffer_size];
    FILE *file;

    snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/direction", gpioNum);
    file = fopen(path, "w");

    if (file == ((void*)0)) {
        std::cerr << "Unable to open file. Cannot initialize pin " << gpioNum << std::endl;
        return -1;
    }

    if (pinDirection) { fprintf(file, "out"); }
    else { fprintf(file, "in"); }

    fclose(file);
    return 1;
}


int8_t Serial::UARTInit(uint8_t uartNum) {

    char path[buffer_size];
    struct termios settings; // Serial settings
    int uart;

    snprintf(path, sizeof(path), "/dev/serial.%d", uartNum);
    
    if(uart = open(path, O_RDWR | O_NDELAY, O_NOCTTY) < 0) {
        std::cerr << "Unable to open file. Cannot initialize serial/uart " << uartNum << std::endl;
        close(uart);
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


int8_t Serial::I2CInit(uint8_t i2cNum, uint8_t slaveAddress) {

    char path[buffer_size];
    int8_t i2c_bus;

    snprintf(path, sizeof(path), "/dev/i2c-%d", i2cNum);
    
    if(i2c_bus = open(path, O_RDWR) < 0) {
        std::cerr << "Unable to open file. Cannot initialize i2c " << i2cNum << std::endl;
        close(i2c_bus);
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



int8_t Serial::SPIInit(uint8_t spiNum, uint8_t mode, uint32_t speed_hz, uint8_t bits) {

    char path[buffer_size];
    int spi_bus;

    snprintf(path, sizeof(path), "/dev/spidev0.%d", spiNum);
    
    if(spi_bus = open(path, O_RDWR) < 0) {
        std::cerr << "Unable to open file. Cannot initialize spi " <<spiNum << std::endl;
        close(spi_bus);
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


int8_t Serial::PinWrite(uint8_t gpioNum, uint8_t pinState) {

    char path[buffer_size];
    FILE *file;

    snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);
    file = fopen(path, "w");

    if (file == ((void*)0)) {
        std::cerr << "Unable to open file. Cannot set state of pin " << gpioNum << std::endl;
        return -1;
    }

    if (pinState != 1 && pinState != 0) { std::cerr << "Invalid Pin State. HIGH = 1, LOW = 0" << std::endl; }
    else { fprintf(file, "%d", pinState); }

    fclose(file);
    return 1;

}


int8_t Serial::PinRead(uint8_t gpioNum) {

    char path[buffer_size];
    FILE *file;
    char* pinState;

    snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);
    file = fopen(path, "r");

    if (file == ((void*)0)) {
        std::cerr << "Unable to open file. Cannot determine state of pin " << gpioNum << std::endl;
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


int8_t Serial::I2CWrite(int &i2c_bus, int8_t *dataBytes, int byteNum) {
    
    if(write(i2c_bus, dataBytes, byteNum) != byteNum){
        std::cerr << "Failed to write" << std::endl;
        return 0;
    }
    return 1;
}


int8_t Serial::I2CRead(int &i2c_bus, int8_t *dataBytes, int byteNum) {

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




