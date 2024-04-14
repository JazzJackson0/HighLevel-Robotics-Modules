#ifndef GPIO_H
#define GPIO_H
#include <stdio.h>
#include <stdint.h>


#define GPIO_PATH "/sys/class/gpio"
#define BUFFER_SIZE 100

typedef enum dir E_Direction;
typedef enum state E_State;
/**
 * @brief Initialize the given pin and set its direction.
 * 
 * @param gpioNum 
 * @param pinDirection Direction of Pin (IN or OUT)
 * @return ** int8_t - Returns 1 is successful, -1 if error occured.
 */
int8_t PinInit(uint8_t gpioNum, E_Direction pinDirection);

/**
 * @brief Set the state of a given pin.
 * 
 * @param gpioNum 
 * @param pinState State of Pin (HIGH or LOW)
 * @return ** int8_t - Returns 1 is successful, -1 if error occured.
 */
int8_t PinOut(uint8_t gpioNum, E_State pinState);

/**
 * @brief Get the state of a given pin.
 * 
 * @param gpioNum 
 * @return ** int8_t - Returns State of Pin, or -1 if error occured.
 */
int8_t Get_Input(uint8_t gpioNum);


enum dir { IN, OUT };
enum state { LOW, HIGH };

#endif



