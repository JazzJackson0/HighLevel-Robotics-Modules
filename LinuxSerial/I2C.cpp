#include "I2C.hpp"
// i2cget -y [i2cnum, e.g. 3] CHIP-ADDDRESS DATA-ADDRESS
// i2cget -y 3 0x40 05 (0x40 + 5 = 0x45)

//i2cset -y 3 0x40 255 ...

// Linux handles all of the single bit operations for you, Start/Stop, R/W, Ack/Nack

int8_t I2CInit(uint8_t i2cNum, uint8_t slaveAddress) {

    char path[BUFFER_SIZE];
    int8_t i2c_bus;

    snprintf(path, sizeof(path), "/dev/i2c-%d", i2cNum);
    
    if(i2c_bus = open(path, O_RDWR) < 0) {
        perror("Unable to open file. Cannot initialize i2c %d", i2cNum);
        close(i2c_bus);
        return -1;
    }
	
	// Setup I2C Bus
	if (ioctl(i2c_bus, I2C_SLAVE, slaveAddress) < 0) {
		perror("Failed to connect to I2C Bus\n");
        close(i2c_bus);
        return -1;
	}

    return i2c_bus;
}

/**
 * @brief Read data from I2C bus
 * 
 * @param i2c_bus I2C Bus device file
 * @param dataBytes Buffer to write data to
 * @param byteNum Number of bytes to read
 * @return int 
 */
int8_t readI2C(int &i2c_bus, int8_t *dataBytes, int byteNum) {

    if (read(i2c_bus, dataBytes, byteNum) != byteNum) {
        perror("Failed to read\n");
        return 0;
    }
    return 1;
}

/**
 * @brief Write data to I2C bus
 * 
 * @param i2c_bus I2C Bus device file
 * @param dataBytes Data to write
 * @param byteNum Number of bytes to write
 * @return int 
 */
int8_t writeI2C(int &i2c_bus, int8_t *dataBytes, int byteNum) {
    
    if(write(i2c_bus, dataBytes, byteNum) != byteNum){
        perror("Failed to write\n");
        return 0;
    }
    return 1;
}

// Example Main (Using BNO055 IMU)-------------------------------------------------
#include "BNO055-I2CExample.hpp"
#define BNO055_ADDRESS_A 0x28
#define REG_INCREMENT_NUM 6 
int regs[] = {BNO055_ACCEL_DATA_X_LSB_ADDR, BNO055_GYRO_DATA_X_LSB_ADDR}; // Start of Accel Arrdesses & Start of Gyro Addresses
#define REG_NUM 2

int main() {  
    int8_t *buff;

    int i2c_bus = I2CInit(2, BNO055_ADDRESS_A);
    if (i2c_bus < 0) { return 1; }

    // Read from every needed register
    for (int i = 0; i < REG_NUM; i++) {

        // Select Register to read from
        writeI2C(i2c_bus, (int8_t*) &regs[i], 1); 

        // Many sensors will auto-increment to next register address
        for (int j = 0; j < REG_INCREMENT_NUM; j++) { 
            
            // Read 1 Byte per Register 
            readI2C(i2c_bus, buff, 1); 
        }
    }
    close(i2c_bus);
    return 0;
}
