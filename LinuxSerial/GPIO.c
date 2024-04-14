#include "GPIO.h"


/**
 * @brief 
 * 
 * @param gpioNum 
 * @param pinDirection 
 * @return int8_t 
 */
int8_t PinInit(uint8_t gpioNum, E_Direction pinDirection) {

    char path[BUFFER_SIZE];
    FILE *file;

    snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/direction", gpioNum);
    file = fopen(path, "w");

    if (file == ((void*)0)) {
        perror("Unable to open file. Cannot initialize pin %d", gpioNum);
        return -1;
    }

    if (pinDirection) { fprintf(file, "out"); }
    else { fprintf(file, "in"); }

    fclose(file);
    return 1;
}


/**
 * @brief 
 * 
 * @param gpioNum 
 * @param pinState 
 * @return int8_t 
 */
int8_t PinOut(uint8_t gpioNum, E_State pinState) {

    char path[BUFFER_SIZE];
    FILE *file;

    snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);
    file = fopen(path, "w");

    if (file == ((void*)0)) {
        perror("Unable to open file. Cannot set state of pin %d", gpioNum);
        return -1;
    }

    if (pinState != 1 && pinState != 0) { perror("Invalid Pin State. HIGH = 1, LOW = 0"); }
    else { fprintf(file, "%d", pinState); }

    fclose(file);
    return 1;

}

/**
 * @brief 
 * 
 * @param gpioNum 
 * @return int8_t 
 */
int8_t Get_Input(uint8_t gpioNum) {

    char path[BUFFER_SIZE];
    FILE *file;
    int8_t pinState;

    snprintf(path, sizeof(path), GPIO_PATH"/gpio%d/value", gpioNum);
    file = fopen(path, "r");

    if (file == ((void*)0)) {
        perror("Unable to open file. Cannot determine state of pin %d", gpioNum);
        return -1;
    }

    fgets(&pinState, 1, file);

    fclose(file);
    return pinState;
}





