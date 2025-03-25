/*!
 *  @file ADXL345.cpp
 *
 *  @mainpage ADXL345 is a low cost accelerometer sensor.
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for Boardoza ADXL345 low cost accelerometer sensors.
 *
 *  @section author Author
 *
 *  Written by Boardoza.
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */
#include "ADXL345.h"

uint8_t ssPin = 5;
uint16_t devAddress = ADXL345_ADDRESS;
uint16_t communicationMode = I2C_MODE;
uint16_t range = RANGE_2G;
uint16_t dataRate = RATE_100_HZ;
uint16_t resolution = RESOLUTION_10_bit;

/**
 * @brief Constructs a new ADXL345 object
 * 
 * This constructor initializes the ADXL345 object with the default device address (ADXL345_ADDRESS) and
 * communication mode (I2C_MODE).
 */
ADXL345::ADXL345() {}

/**
 * @brief Enables the data ready interrupt of the ADXL345 sensor.
 *
 * This function enables the interrupt that is triggered when new data is available
 * from the ADXL345 sensor. It configures the necessary registers to enable the data ready interrupt.
 */
void ADXL345::enableDataReadyInterrupt() {
  if(communicationMode == I2C_MODE) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(INT_ENABLE); // INT_ENABLE_REGISTER
    Wire.write(DATA_READY_BIT); // 0X80
    Wire.endTransmission();
  } else if(communicationMode == SPI_MODE) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // SPI communication is initiated
    
    digitalWrite(ssPin, LOW); // Chip Select pin is pulled low
    
    SPI.transfer(INT_ENABLE); // INT_ENABLE_REGISTER
    SPI.transfer(DATA_READY_BIT); // 0x80

    digitalWrite(ssPin, HIGH); // Chip Select pin is pulled high

    SPI.endTransaction(); // SPI communication is terminated
  }
}

/**
 * @brief Enables the single tap interrupt of the ADXL345 sensor.
 * 
 * This function enables the interrupt that is triggered when a single tap is detected by the ADXL345 sensor.
 * It configures the necessary registers to enable the single tap interrupt and sets the tap threshold and duration.
 *
 * @param tapThreshold  The threshold value for a tap event. The higher the value, the more force is required to trigger a tap. Valid range: 0-255.
 * @param tapDuration The duration of a tap event. This determines how long the acceleration value must stay above the threshold to be considered a tap. Valid range: 0-255.
*/
void ADXL345::enableSingleTapInterrupt(uint16_t tapThreshold, uint16_t tapDuration, uint16_t tapLatent, uint16_t tapWindow) {
    if(communicationMode == I2C_MODE) {

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(TAP_AXES); // TAP_AXES
        Wire.write(ADXL345_X_AXIS | ADXL345_Y_AXIS | ADXL345_Z_AXIS);
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(INT_MAP); // INT_MAP
        Wire.write(SINGLE_TAP_BIT);
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(THRESH_TAP); // THRESH_TAP
        Wire.write(tapThreshold); // Single Tap edge
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(DUR); // DUR
        Wire.write(tapDuration); // Tap time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(LATENT); // LATENT
        Wire.write(tapLatent); // Latent time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(WINDOW); // WINDOW
        Wire.write(tapWindow); // Tap window time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS); 
        Wire.write(INT_ENABLE); // INT_ENABLE
        Wire.write(SINGLE_TAP_BIT);
        Wire.endTransmission();

    } else if(communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(ssPin, LOW);

        SPI.transfer(TAP_AXES); // TAP_AXES
        SPI.transfer(ADXL345_X_AXIS | ADXL345_Y_AXIS | ADXL345_Z_AXIS);

        SPI.transfer(INT_MAP); // INT_MAP
        SPI.transfer(SINGLE_TAP_BIT);

        SPI.transfer(THRESH_TAP); // THRESH_TAP
        SPI.transfer(tapThreshold); // Single Tap edge

        SPI.transfer(DUR); // DUR
        SPI.transfer(tapDuration); // Tap time

        SPI.transfer(LATENT); // LATENT
        SPI.transfer(tapLatent); // Latent time

        SPI.transfer(WINDOW); // WINDOW
        SPI.transfer(tapWindow); // Tap window time

        SPI.transfer(INT_ENABLE); // INT_ENABLE
        SPI.transfer(SINGLE_TAP_BIT);

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Enables the double tap interrupt of the ADXL345 sensor.
 *
 * This function enables the interrupt that is triggered when a double tap is detected by the ADXL345 sensor.
 * It configures the necessary registers to enable the double tap interrupt and sets the tap threshold, duration, latent time, and window time.
 *
 * @param tapThreshold The threshold value for a tap event. The higher the value, the more force is required to trigger a tap. Valid range: 0-255.
 * @param tapDuration The duration of a tap event. This determines how long the acceleration value must stay above the threshold to be considered a tap. Valid range: 0-65535.
 * @param tapLatent The time between the end of the first tap and the start of the second tap. Valid range: 0-255.
 * @param tapWindow The maximum time between the start of the first tap and the end of the second tap. Valid range: 0-65535.
 */
void ADXL345::enableDoubleTapInterrupt(uint16_t tapThreshold, uint16_t tapDuration, uint16_t tapLatent, uint16_t tapWindow) {
    if(communicationMode == I2C_MODE) {

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(TAP_AXES); // TAP_AXES
        Wire.write(ADXL345_X_AXIS | ADXL345_Y_AXIS | ADXL345_Z_AXIS);
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(INT_MAP); // INT_MAP
        Wire.write(DOUBLE_TAP_BIT);
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(THRESH_TAP); // THRESH_TAP
        Wire.write(tapThreshold); // Double Tap edge
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(DUR); // DUR
        Wire.write(tapDuration); // Tap time 
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(LATENT); // LATENT
        Wire.write(tapLatent); // Latent time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(WINDOW); // WINDOW
        Wire.write(tapWindow); // Tap window time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS); 
        Wire.write(INT_ENABLE); // INT_ENABLE
        Wire.write(DOUBLE_TAP_BIT);
        Wire.endTransmission();

    } else if(communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(ssPin, LOW);

        SPI.transfer(TAP_AXES); // TAP_AXES
        SPI.transfer(ADXL345_X_AXIS | ADXL345_Y_AXIS | ADXL345_Z_AXIS);

        SPI.transfer(INT_MAP); // INT_MAP
        SPI.transfer(DOUBLE_TAP_BIT);

        SPI.transfer(THRESH_TAP); // THRESH_TAP
        SPI.transfer(tapThreshold); // Double Tap edge

        SPI.transfer(DUR); // DUR
        SPI.transfer(tapDuration); // Tap time

        SPI.transfer(LATENT); // LATENT
        SPI.transfer(tapLatent); // Latent time

        SPI.transfer(WINDOW); // WINDOW
        SPI.transfer(tapWindow); // Tap window time
        
        SPI.transfer(INT_ENABLE); // INT_ENABLE
        SPI.transfer(DOUBLE_TAP_BIT);

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Enables the activity interrupt of the ADXL345 sensor.
 *
 * This function enables the interrupt that is triggered when activity is detected by the ADXL345 sensor.
 * It configures the necessary registers to enable the activity interrupt and sets the activity threshold and activity time.
 *
 * @param threshold The threshold value for an activity event. The higher the value, the more acceleration is required to trigger an activity. Valid range: 0-255.
 * @param time The time period in which the acceleration value must stay above the threshold to be considered an activity. Valid range: 0-255.
 */
void ADXL345::enableActivityInterrupt(uint8_t threshold, uint8_t time) {
    if (communicationMode == I2C_MODE) {
        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(THRESH_ACT); // THRESH_ACT
        Wire.write(threshold); // Activity threshold
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(ACT_INACT_CTL); // ACT_INACT_CTL
        Wire.write(0x70); // Activity bits
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(INT_ENABLE); // INT_ENABLE
        Wire.write(ACTIVITY_BIT);
        Wire.endTransmission();
    } else if (communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

        digitalWrite(ssPin, LOW);

        SPI.transfer(THRESH_ACT); // THRESH_ACT
        SPI.transfer(threshold); // Activity threshold

        SPI.transfer(ACT_INACT_CTL); // ACT_INACT_CTL
        SPI.transfer(time); // Activity time

        SPI.transfer(INT_ENABLE); // INT_ENABLE
        SPI.transfer(ACTIVITY_BIT);

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Enable the inactivity interrupt for the ADXL345 sensor.
 * 
 * @param threshold The inactivity threshold value.
 * @param time The inactivity time value.
 * 
 * @details This function enables the inactivity interrupt for the ADXL345 sensor. The inactivity threshold and inactivity time
 * parameters determine when the interrupt will be triggered.
 * 
 * @code
 * // Example usage:
 * accelerometer.enableInactivityInterrupt(10, 5);
 * @endcode
 */
void ADXL345::enableInactivityInterrupt(uint8_t threshold, uint8_t time) { // time değerleri denenecek 10,20,30,40 vs.
    if (communicationMode == I2C_MODE) {
        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(THRESH_INACT); // THRESH_INACT
        Wire.write(threshold); // Inactivity threshold
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(TIME_INACT); // THRESH_INACT
        Wire.write(time); // Inactivity time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(ACT_INACT_CTL); // ACT_INACT_CTL
        Wire.write(0x07); // Inactivity bits
        Wire.endTransmission();
        
        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(INT_ENABLE); // INT_ENABLE
        Wire.write(INACTIVITY_BIT);
        Wire.endTransmission();
    } else if (communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(ssPin, LOW);

        SPI.transfer(THRESH_INACT); // THRESH_INACT
        SPI.transfer(threshold); // Inactivity threshold

        SPI.transfer(ACT_INACT_CTL); // ACT_INACT_CTL
        SPI.transfer(0x07); // Inactivity bits

        SPI.transfer(INT_ENABLE); // INT_ENABLE
        SPI.transfer(INACTIVITY_BIT);

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Enable the freefall interrupt for the ADXL345 sensor.
 * 
 * @param threshold The freefall threshold value. Threshold recommended value between 300mg and 600mg (0x05 to 0x09).
 * @param time The freefall time value. Time recommended value between 100ms and 350ms (0x14 to 0x46).
 * 
 * @details This function enables the freefall interrupt for the ADXL345 sensor. The freefall threshold and freefall time
 * parameters determine when the interrupt will be triggered.
 * 
 * @code
 * // Example usage:
 * accelerometer.enableFreefallInterrupt(0x05, 0x14);
 */
void ADXL345::enableFreefallInterrupt(uint8_t threshold, uint8_t time) {
    if (communicationMode == I2C_MODE) {
        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(FREEFALL_THRESHOLD); // FREE_FALL
        Wire.write(threshold); // Freefall threshold
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(FREEFALL_TIME); // FREE_FALL_TIME
        Wire.write(time); // Freefall time
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(INT_ENABLE); // INT_ENABLE
        Wire.write(FREE_FALL_BIT);
        Wire.endTransmission();
    } else if (communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(ssPin, LOW);

        SPI.transfer(FREEFALL_THRESHOLD); // FREE_FALL
        SPI.transfer(threshold); // Freefall threshold

        SPI.transfer(FREEFALL_TIME); // FREE_FALL_TIME
        SPI.transfer(time); // Freefall time

        SPI.transfer(INT_ENABLE); // INT_ENABLE
        SPI.transfer(FREE_FALL_BIT);

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Enable the watermark interrupt for the ADXL345 sensor.
 * 
 * @param threshold The watermark threshold value.
 * 
 * @details This function enables the watermark interrupt for the ADXL345 sensor. The watermark threshold parameter determines
 * when the interrupt will be triggered.
 * 
 * @code
 * // Example usage:
 * accelerometer.enableWatermarkInterrupt(10);
 */
void ADXL345::enableWatermarkInterrupt(uint8_t threshold) {
    if (communicationMode == I2C_MODE) {
        Wire.beginTransmission(ADXL345_ADDRESS); // Set ADXL345 sensor address
        Wire.write(0x38); // FIFO_CTL
        Wire.write(0x80 | threshold); // Enable watermark interrupt and set threshold
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(0x2E); // INT_ENABLE
        Wire.write(WATERMARK_BIT);
        Wire.endTransmission();
    } else if (communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(ssPin, LOW);

        SPI.transfer(0x38); // FIFO_CTL
        SPI.transfer(0x80 | threshold); // Enable watermark interrupt and set threshold

        SPI.transfer(0x2E); // INT_ENABLE
        SPI.transfer(WATERMARK_BIT);

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Enable the overrun interrupt for the ADXL345 sensor.
 * 
 * @details This function enables the overrun interrupt for the ADXL345 sensor. The overrun interrupt is triggered when the
 * FIFO buffer of the sensor is full and new data is being written to it, causing old data to be overwritten.
 * 
 * @code
 * // Example usage:
 * accelerometer.enableOverrunInterrupt();
 */
void ADXL345::enableOverrunInterrupt() {
    if (communicationMode == I2C_MODE) {
        Wire.beginTransmission(ADXL345_ADDRESS); // Set ADXL345 sensor address
        Wire.write(INT_MAP); // INT_MAP
        Wire.write(0x01); // Set Overrun bit to INT2 pin
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(0x38); // FIFO_CTL
        Wire.write(0x00); // Set FIFO mode to bypass mode
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDRESS);
        Wire.write(INT_ENABLE); // INT_ENABLE
        Wire.write(OVERRUN_BIT); // Enable Overrun interrupt
        Wire.endTransmission();
    } else if (communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(ssPin, LOW);

        SPI.transfer(INT_MAP); // INT_MAP
        SPI.transfer(0x01); // Set Overrun bit to INT2 pin

        SPI.transfer(0x38); // FIFO_CTL
        SPI.transfer(0x00); // Set FIFO mode to bypass mode

        SPI.transfer(INT_ENABLE); // INT_ENABLE
        SPI.transfer(OVERRUN_BIT); // Enable Overrun interrupt

        digitalWrite(ssPin, HIGH);

        SPI.endTransaction();
    }
}

/**
 * @brief Set the FIFO mode for the ADXL345 sensor.
 * 
 * @param fifoMode The FIFO mode to set.
 * 
 * @details This function sets the FIFO mode for the ADXL345 sensor. The available FIFO modes are:
 * - FIFO_MODE_BYPASS: Bypass mode, where the FIFO buffer is not used.
 * - FIFO_MODE_FIFO: FIFO mode, where the FIFO buffer is used to store data until it is read.
 * - FIFO_MODE_STREAM: Stream mode, where the FIFO buffer is used to store data and older data is discarded if the buffer is full.
 * - FIFO_MODE_TRIGGER: Trigger mode, where the FIFO buffer is used to store data until a trigger event occurs.
 * 
 * @code
 * // Example usage:
 * accelerometer.setFifoMode(FIFO_MODE_FIFO);
 * 
 * @return void
 * - If a valid FIFO mode is selected, the FIFO mode will be set accordingly.
 * - If an invalid FIFO mode is selected, the function will return without changing the FIFO mode.
 */
void ADXL345::setFifoMode(uint8_t fifoMode) {
  // Ensure valid FIFO mode is selected
  if (fifoMode < FIFO_MODE_BYPASS || fifoMode > FIFO_MODE_TRIGGER) {
    return; // Invalid FIFO mode
  }

  uint8_t regValue = readRegister(0x38); // FIFO_CTL
  regValue &= 0xFC; // Clear FIFO_MODE bits
  regValue |= fifoMode; // Set FIFO_MODE bits

  writeRegister(0x38, regValue); // FIFO_CTL
}

/**
 * @brief Set the threshold value for the ADXL345 sensor.
 * 
 * @param threshold The threshold value to set.
 * 
 * @details This function sets the threshold value for the ADXL345 sensor. The threshold value determines the minimum acceleration value that will trigger an interrupt or event.
 * 
 * @param threshold The threshold value to set. Valid values are between 0 and 255.
 * 
 * @return void
 * - If a valid threshold value is selected, the threshold will be set accordingly.
 * - If an invalid threshold value is selected, the function will return without changing the threshold value.
 */
void ADXL345::setThreshold(int threshold) {
  // Ensure valid threshold value is selected
  if (threshold < 0 || threshold > 255) {
    return; // Invalid threshold value
  }
  uint8_t thresholdL = threshold & 0xFF; // Lower byte of threshold

  writeRegister(0x1D, thresholdL); 
}

/**
 * @brief Perform offset calibration for the ADXL345 sensor.
 * 
 * @note This function should be called after initializing the ADXL345 sensor.
 * 
 * @details This function performs offset calibration for the ADXL345 sensor.
 *  It reads the acceleration values when the sensor is placed on a flat surface and held at rest, and calculates the offset values based on these readings.
 *  The offset values are then set in the corresponding registers of the ADXL345 sensor.
 */
void ADXL345::offsetCalibration() {
  // Readings from an accelerometer placed on a flat surface and held at rest
float offsetX, offsetY, offsetZ;
readAcceleration(offsetX, offsetY, offsetZ);

// You can make offset calculations
int xOffset = -offsetX;
int yOffset = -offsetY;
int zOffset = (256 - offsetZ);

  // Set the offset values in the IV_OFFS registers
  writeRegister(0x1E, xOffset); // 0x1E = OFSX
  writeRegister(0x1F, yOffset); // 0x1F = OFSY
  writeRegister(0x20, zOffset); // 0x20 = OFSZ
}

/**
 * @brief Set the power mode for the ADXL345 sensor.
 * 
 * @param powerMode The power mode to set.
 * 
 * @details This function sets the power mode for the ADXL345 sensor. The available power modes are:
 * - POWER_NORMAL: Normal power mode, where the sensor is actively measuring and consuming higher power.
 * - POWER_LOW: Low power mode, where the sensor is in sleep mode to conserve power.
 * - POWER_STANDBY: Standby mode, where the sensor is in a state between normal and sleep mode.
 * 
 * @return void
 * - If a valid power mode is selected, the power mode will be set accordingly.
 * - If an invalid power mode is selected, the function will return without changing the power mode.
 */
void ADXL345::setPowerMode(PowerMode powerMode) {
  uint8_t powerCtlReg = readRegister(ADXL345_REG_POWER_CTL);
  powerCtlReg &= ~(0x0F); // Clear current power mode

  if (powerMode == POWER_NORMAL) {
    powerCtlReg |= POWER_CTL_MEASURE_BIT; // Setup measurement mode
  } else if (powerMode == POWER_LOW) {
    powerCtlReg |= POWER_CTL_LOW_POWER_BIT; // Setup sleep mode
  } else if (powerMode == POWER_STANDBY) {
    // No need to assign any register for standby mode
  }

  writeRegister(ADXL345_REG_POWER_CTL, powerCtlReg);
}

/**
 * @brief Get the current power mode of the ADXL345 sensor.
 * 
 * @details This function reads the power control register of the ADXL345 sensor to determine the current power mode.
 * The power mode determines the operating mode of the sensor, such as normal power mode, low power mode, or standby mode.
 * The function returns the current power mode as a value of the ADXL345::PowerMode enum.
 * 
 * @return The current power mode of the ADXL345 sensor.
 * 
 * @see ADXL345::PowerMode
 */
ADXL345::PowerMode ADXL345::getPowerMode() {
  uint8_t powerModeReg = readRegister(ADXL345_REG_POWER_CTL);
  powerModeReg &= 0x03; // Mask the power mode bits
  switch(powerModeReg) {
    case POWER_NORMAL:
      return POWER_NORMAL;
    case POWER_LOW:
      return POWER_LOW;
    case POWER_STANDBY:
      return POWER_STANDBY;
    default:
      return POWER_NORMAL;
  }
}

/**
 * @brief Initialize the ADXL345 sensor.
 * 
 * @details This function initializes the ADXL345 sensor.
 *  It sets up the communication mode and initializes the necessary peripherals (I2C or SPI). It also sets the default range of the sensor to 2G.
 * 
 * @param communicationMode The communication mode to use. Valid values are:
 * - I2C_MODE: I2C communication mode.
 * - SPI_MODE: SPI communication mode.
 */
void ADXL345::begin(int communicationMode) {
    
    if(communicationMode == I2C_MODE) {
        Wire.begin();
    } else if(communicationMode == SPI_MODE) {
        SPI.begin();
        SPI.setDataMode(SPI_MODE3);
        pinMode(ssPin, OUTPUT);
        digitalWrite(ssPin, HIGH);
    }

}

/**
 * @brief Set the range for the ADXL345 sensor.
 * 
 * @param range The range to set.
 * 
 * @details This function sets the range for the ADXL345 sensor.
 *  The range determines the maximum acceleration value that the sensor can measure. The available ranges are:
 * - RANGE_2G: ±2g range
 * - RANGE_4G: ±4g range
 * - RANGE_8G: ±8g range
 * - RANGE_16G: ±16g range
 */
void ADXL345::setRange(int temp_range) {
    uint8_t regValue = readRegister(ADXL345_REG_DATA_FORMAT);
    regValue &= 0xFC;
    regValue |= temp_range;
    range=temp_range;
    writeRegister(ADXL345_REG_DATA_FORMAT, regValue);

}

/**
 * @brief Set the resolution for the ADXL345 sensor.
 * 
 * @param resolution The resolution to set.
 * 
 * @details This function sets the resolution for the ADXL345 sensor.
 * The resolution determines the number of bits used to represent the acceleration data.
 * The available resolutions are:
 * - RESOLUTION_10_bit: 10-bit resolution
 * - RESOLUTION_11_bit: 11-bit resolution
 * - RESOLUTION_12_bit: 12-bit resolution
 * - RESOLUTION_13_bit: 13-bit resolution
 */
void ADXL345::setResolution(int temp_resolution) {
    uint8_t regValue = readRegister(ADXL345_REG_DATA_FORMAT);
    regValue &= 0xF7;
    regValue |= temp_resolution;
    resolution = temp_resolution;
    writeRegister(ADXL345_REG_DATA_FORMAT, regValue);
}

/**
 * @brief Set the justify for the ADXL345 sensor.
 * 
 * @param justify The justify to set.
 * 
 * @details This function sets the justify for the ADXL345 sensor.
 * The justify determines the data justification in the data format register.
 * The available justify values are:
 * - RIGHT_JUSTIFY: Right justify the data
 * - LEFT_JUSTIFY: Left justify the data
 */
void ADXL345::setJustify(int justify) {
    uint8_t regValue = readRegister(ADXL345_REG_DATA_FORMAT);
    regValue &= 0xFB;
    regValue |= justify << 2;
    writeRegister(ADXL345_REG_DATA_FORMAT, regValue);
}

/**
 * @brief Set the self-test for the ADXL345 sensor.
 * 
 * @param test The self-test value to set.
 * 
 * @details This function sets the self-test for the ADXL345 sensor.
 * The self-test applies a self-test force to the sensor, causing a shift in the output data.
 * The available self-test values are:
 * - 0: Disable self-test force
 * - 1: Enable self-test force
 */
void ADXL345::setSelf_test(int test) {
    uint8_t regValue = readRegister(ADXL345_REG_DATA_FORMAT);
    regValue &= 0x7F;
    regValue |= test;
    writeRegister(ADXL345_REG_DATA_FORMAT, regValue);
}

/**
 * @brief Get the current range setting of the ADXL345 sensor.
 * 
 * @details This function reads the range register of the ADXL345 sensor to determine the current range setting.
 * The range setting determines the full scale range for the acceleration measurements.
 * The function returns the range value in g units.
 * 
 * @return The current range setting in g units.
 */
float ADXL345::getRange() {
  uint8_t rangeReg = readRegister(ADXL345_REG_DATA_FORMAT);
  rangeReg &= 0x03; // Mask the range bits
  switch(rangeReg) {
    case RANGE_2G:
      return 2.0;
    case RANGE_4G:
      return 4.0;
    case RANGE_8G:
      return 8.0;
    case RANGE_16G:
      return 16.0;
    default:
      return 0.0;
  }
}

/**
 * @brief Set the data rate for the ADXL345 sensor.
 * 
 * @param dataRate The data rate to set.
 * 
 * @details This function sets the data rate for the ADXL345 sensor.
 *  The data rate determines the rate at which the sensor samples and outputs acceleration data.
 *  The available data rates are:
 * - DATA_RATE_0_10_HZ: 0.10 Hz
 * - DATA_RATE_0_20_HZ: 0.20 Hz
 * - DATA_RATE_0_39_HZ: 0.39 Hz
 * - DATA_RATE_0_78_HZ: 0.78 Hz
 * - DATA_RATE_1_56_HZ: 1.56 Hz
 * - DATA_RATE_3_13_HZ: 3.13 Hz
 * - DATA_RATE_6_25_HZ: 6.25 Hz
 * - DATA_RATE_12_5_HZ: 12.5 Hz
 * - DATA_RATE_25_HZ: 25 Hz
 * - DATA_RATE_50_HZ: 50 Hz
 * - DATA_RATE_100_HZ: 100 Hz
 * - DATA_RATE_200_HZ: 200 Hz
 * - DATA_RATE_400_HZ: 400 Hz
 * - DATA_RATE_800_HZ: 800 Hz
 * - DATA_RATE_1600_HZ: 1600 Hz
 * - DATA_RATE_3200_HZ: 3200 Hz
 */
void ADXL345::setDataRate(int dataRate) {
    uint8_t regValue = readRegister(ADXL345_REG_BW_RATE);
    regValue &= 0xF0;
    regValue |= dataRate;
    writeRegister(ADXL345_REG_BW_RATE, regValue);

}

/**
 * @brief Get the current data rate setting of the ADXL345 sensor.
 * 
 * @details This function reads the bandwidth rate register of the ADXL345 sensor to determine the current data rate setting.
 * The data rate setting determines the output data rate of the sensor, which affects the frequency at which new acceleration data is available.
 * The function returns the current data rate value in Hz (Hertz).
 * 
 * @return The current data rate setting in Hz.
 */
float ADXL345::getDataRate() {
  uint8_t dataRateReg = readRegister(ADXL345_REG_BW_RATE);
  dataRateReg &= 0x0F; // Mask the data rate bits
  switch(dataRateReg) {
    case RATE_3200_HZ:
      return 3200.0;
    case RATE_1600_HZ:
      return 1600.0;
    case RATE_800_HZ:
      return 800.0;
    case RATE_400_HZ:
      return 400.0;
    case RATE_200_HZ:
      return 200.0;
    case RATE_100_HZ:
      return 100.0;
    case RATE_50_HZ:
      return 50.0;
    case RATE_25_HZ:
      return 25.0;
    case RATE_12_5_HZ:
      return 12.5;
    case RATE_6_25_HZ:
      return 6.25;
    case RATE_3_13_HZ:
      return 3.13;
    case RATE_1_56_HZ:
      return 1.56;
    case RATE_0_78_HZ:
      return 0.78;
    case RATE_0_39_HZ:
      return 0.39;
    case RATE_0_20_HZ:
      return 0.20;
    case RATE_0_10_HZ:
      return 0.10;
    default:
      return 0.0;
  }
}

/**
 * @brief Enable full resolution for the ADXL345 sensor.
 * 
 * @details This function enables full resolution for the ADXL345 sensor.
 * Full resolution is required for certain ranges: 11-bit for 4G, 12-bit for 8G, and 13-bit for 16G.
 * Make sure to call this function when using the corresponding range.
 */
void ADXL345::enableFullRes() {
    uint8_t regValue = readRegister(ADXL345_REG_DATA_FORMAT);
    regValue |= 0x08; // 0x08 is a bit mask used to enable the measurement.
    writeRegister(ADXL345_REG_DATA_FORMAT, regValue);
}

/**
 * @brief Disable full resolution for the ADXL345 sensor.
 * 
 * @details This function disables full resolution for the ADXL345 sensor.
 */
void ADXL345::disableFullRes() {
    uint8_t regValue = readRegister(ADXL345_REG_DATA_FORMAT);
    regValue &= ~0x08; // ~0x08 is a bit mask used to disable the measurement.
    writeRegister(ADXL345_REG_DATA_FORMAT, regValue);
}

/**
 * @brief Enable measurement for the ADXL345 sensor.
 * 
 * @details This function enables the measurement mode for the ADXL345 sensor. In measurement mode, the sensor actively measures acceleration data.
 * 
 * The value 0x08 is a bit mask used to enable measurement in the ADXL345_REG_POWER_CTL (power control) register.
 *  This value sets the corresponding bit to 1 and enables the measurement mode.
 */
void ADXL345::enableMeasurement() {
    uint8_t regValue = readRegister(ADXL345_REG_POWER_CTL);
    regValue |= 0x08; // 0x08 is a bit mask used to enable the measurement.
    writeRegister(ADXL345_REG_POWER_CTL, regValue);
}

/**
 * @brief Disable measurement for the ADXL345 sensor.
 * 
 * @details This function disables the measurement mode for the ADXL345 sensor.
 *  In measurement mode, the sensor actively measures acceleration data.
 *  By disabling the measurement mode, the sensor stops measuring and conserves power.
 */
void ADXL345::disableMeasurement() {
    uint8_t regValue = readRegister(ADXL345_REG_POWER_CTL);
    regValue &= ~0x08; // ~0x08 is a bit mask used to disable the measurement.
    writeRegister(ADXL345_REG_POWER_CTL, regValue);
}

/**
 * @brief Reads the acceleration data from the ADXL345 sensor.
 * 
 * @param x Reference to store the acceleration value in the x-axis.
 * @param y Reference to store the acceleration value in the y-axis.
 * @param z Reference to store the acceleration value in the z-axis.
 * 
 * @details This function reads the raw acceleration data from the ADXL345 sensor and converts it to G units.
 * The sensor provides 3-axis acceleration data, which is stored in two's complement format.
 * The raw acceleration data is converted to floating-point values in G units using the `convertToG` function.
 * 
 * @note The acceleration values can be negative if there is a negative acceleration in any axis.
 * 
 * @see 
 */
bool ADXL345::readAcceleration(float& x, float& y, float& z) {
    uint8_t rawData[6];
    readRegisters(ADXL345_REG_DATAX0, 6, rawData);
    int16_t rawX = (uint16_t)(rawData[0]) | (uint16_t)(rawData[1] << 8);
    int16_t rawY = (uint16_t)(rawData[2]) | (uint16_t)(rawData[3] << 8);
    int16_t rawZ = (uint16_t)(rawData[4]) | (uint16_t)(rawData[5] << 8); 

    switch (range)
    {
    case RANGE_2G:
      x = (float)rawX / 256.0;
      y = (float)rawY / 256.0;
      z = (float)rawZ / 256.0;
      break;
    case RANGE_4G:
    if(resolution == RESOLUTION_10_bit) {
      x = (float)rawX / 128.0;
      y = (float)rawY / 128.0;
      z = (float)rawZ / 128.0;
    } else if (resolution == RESOLUTION_11_bit) {
      x = (float)rawX / 256.0;
      y = (float)rawY / 256.0;
      z = (float)rawZ / 256.0;
    }
    break;
    case RANGE_8G:
      if(resolution == RESOLUTION_10_bit) {
      x = (float)rawX / 64.0;
      y = (float)rawY / 64.0;
      z = (float)rawZ / 64.0;
    } else if (resolution == RESOLUTION_12_bit) {
      x = (float)rawX / 256.0;
      y = (float)rawY / 256.0;
      z = (float)rawZ / 256.0;
    }
    break;
    case RANGE_16G:
      if(resolution == RESOLUTION_10_bit) {
      x = (float)rawX / 32.0;
      y = (float)rawY / 32.0;
      z = (float)rawZ / 32.0;
    } else if (resolution == RESOLUTION_13_bit) {
      x = (float)rawX / 256.0;
      y = (float)rawY / 256.0;
      z = (float)rawZ / 256.0;
    }
      break;
    }

    return true;
}

int16_t ADXL345::getX(void) {
    return readRegister(ADXL345_REG_DATAX0);
}

int16_t ADXL345::getY(void) {
    return readRegister(ADXL345_REG_DATAY0);
}

int16_t ADXL345::getZ(void) {
    return readRegister(ADXL345_REG_DATAZ0);
}

/**
 * @brief Write a value to a register in the ADXL345 sensor.
 * 
 * @param reg The register address to write to.
 * @param value The value to write to the register.
 * 
 * @details This function writes a value to a register in the ADXL345 sensor.
 *  The function supports both I2C and SPI communication modes.
 *  The communication mode is set to I2C or SPI, the function uses the Wire or SPI library to send the register address and value to the sensor.
 */
void ADXL345::writeRegister(uint8_t reg, uint8_t value) {
    if(communicationMode == I2C_MODE) {
        Wire.beginTransmission(devAddress);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission(true);
    } else if(communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        digitalWrite(ssPin, LOW);
        SPI.transfer(reg);
        SPI.transfer(value);
        digitalWrite(ssPin, HIGH);
        SPI.endTransaction();
    }
}

/**
 * @brief Read the value from a register in the ADXL345 sensor.
 * 
 * @param reg The register address to read from.
 * 
 * @details This function reads the value from a register in the ADXL345 sensor.
 *  The function supports both I2C and SPI communication modes.
 *  The communication mode is set to I2C or SPI, the function uses the Wire or SPI library to send the register address to the sensor.
 *  Request the value from the sensor, read the value.
 * 
 * @return The value read from the register.
 */
uint8_t ADXL345::readRegister(uint8_t reg) {
    uint8_t value = 0;
    if(communicationMode == I2C_MODE) {
        Wire.beginTransmission(devAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(devAddress, 1u);
        value = Wire.read();
        return value;
    } else if(communicationMode == SPI_MODE) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        digitalWrite(ssPin, LOW);
        SPI.transfer(reg | 0x80);
        value = SPI.transfer(0x00);
        digitalWrite(ssPin, HIGH);
        SPI.endTransaction();
    }
    return value;
}

/**
 * @brief Read multiple registers from the ADXL345 sensor.
 * 
 * @param reg The starting register address to read from.
 * @param count The number of registers to read.
 * @param data Pointer to an array to store the read register values.
 */
void ADXL345::readRegisters(uint8_t reg, uint8_t count, uint8_t* data) {
    if(communicationMode == I2C_MODE) {
    Wire.beginTransmission(devAddress);
    Wire.write(reg); 
    Wire.endTransmission(false);
    Wire.requestFrom(devAddress, count);
    
    for(uint8_t i = 0; i < count; i++) {
        data[i] = Wire.read();
    }
  } else if(communicationMode == SPI_MODE) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // SPI communication start
    digitalWrite(ssPin, LOW); // CS pin is pulled LOW
    
    SPI.transfer(reg | 0x80);
    for(uint8_t i = 0; i < count; i++) {
      data[i] = SPI.transfer(0x00);
    }
    
    digitalWrite(ssPin, HIGH); // CS pin is pulled HIGH
    SPI.endTransaction();
  }
}
