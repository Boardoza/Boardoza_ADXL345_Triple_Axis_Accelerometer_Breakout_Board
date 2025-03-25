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

#ifndef ADXL345_h
#define ADXL345_h

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define DEVID 0x00
#define ADXL345_ADDRESS 0x53
#define THRESH_TAP 0x1D
#define DUR 0x21
#define LATENT 0x22
#define WINDOW 0x23
#define THRESH_ACT 0x24
#define THRESH_INACT 0x25
#define TIME_INACT 0x26
#define ACT_INACT_CTL 0x27
#define FREEFALL_THRESHOLD 0x28
#define FREEFALL_TIME 0x29
#define TAP_AXES 0x2A
#define ACT_TAP_STATUS
#define ADXL345_REG_BW_RATE 0x2C
#define ADXL345_REG_POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_REG_DATAX1 0x33
#define ADXL345_REG_DATAY0 0x34
#define ADXL345_REG_DATAY1 0x35
#define ADXL345_REG_DATAZ0 0x36
#define ADXL345_REG_DATAZ1 0x37

#define POWER_CTL_MEASURE_BIT 0x08
#define POWER_CTL_SLEEP_BIT 0x04
#define POWER_CTL_LOW_POWER_BIT 0x2C

#define DATA_READY_BIT 0x80
#define SINGLE_TAP_BIT 0x40
#define DOUBLE_TAP_BIT 0x20
#define ACTIVITY_BIT 0x10
#define INACTIVITY_BIT 0x08
#define FREE_FALL_BIT 0x04
#define WATERMARK_BIT 0x02
#define OVERRUN_BIT 0x01
#define ADXL345_X_AXIS 0x04
#define ADXL345_Y_AXIS 0x02
#define ADXL345_Z_AXIS 0x01

#define FIFO_MODE_BYPASS 0b00
#define FIFO_MODE_FIFO 0b01
#define FIFO_MODE_STREAM 0b10
#define FIFO_MODE_TRIGGER 0b11

#define I2C_MODE (0)
#define SPI_MODE (1)

#define RANGE_2G (0x00)
#define RANGE_4G (0x01)
#define RANGE_8G (0x02)
#define RANGE_16G (0x03)

#define RESOLUTION_10_bit (0x00)
#define RESOLUTION_11_bit (0x01)
#define RESOLUTION_12_bit (0x02)
#define RESOLUTION_13_bit (0x03)

#define RIGHT_JUSTIFY   (0x00)
#define LEFT_JUSTIFY    (0x01)

#define RATE_3200_HZ 0b1111
#define RATE_1600_HZ 0b1110
#define RATE_800_HZ 0b1101
#define RATE_400_HZ 0b1100
#define RATE_200_HZ 0b1011
#define RATE_100_HZ 0b1010
#define RATE_50_HZ 0b1001
#define RATE_25_HZ 0b1000
#define RATE_12_5_HZ 0b0111
#define RATE_6_25_HZ 0b0110
#define RATE_3_13_HZ 0b0101
#define RATE_1_56_HZ 0b0100
#define RATE_0_78_HZ 0b0011
#define RATE_0_39_HZ 0b0010
#define RATE_0_20_HZ 0b0001
#define RATE_0_10_HZ 0b0000

extern uint16_t devAddress;
extern uint16_t communicationMode;
extern uint16_t range;
extern uint16_t dataRate;

class ADXL345 {
public:
  /*  enum CommunicationMode {
        I2C_MODE,
        SPI_MODE
    };*/

  /*  enum Range {
        RANGE_2G = 0x00,
        RANGE_4G = 0x01,
        RANGE_8G = 0x02,
        RANGE_16G = 0x03
    };*/

   /* enum DataRate {
    RATE_3200_HZ = 0b1111,
    RATE_1600_HZ = 0b1110,
    RATE_800_HZ = 0b1101,
    RATE_400_HZ = 0b1100,
    RATE_200_HZ = 0b1011,
    RATE_100_HZ = 0b1010,
    RATE_50_HZ = 0b1001,
    RATE_25_HZ = 0b1000,
    RATE_12_5_HZ = 0b0111,
    RATE_6_25_HZ = 0b0110,
    RATE_3_13_HZ = 0b0101,
    RATE_1_56_HZ = 0b0100,
    RATE_0_78_HZ = 0b0011,
    RATE_0_39_HZ = 0b0010,
    RATE_0_20_HZ = 0b0001,
    RATE_0_10_HZ = 0b0000
    };*/

enum PowerMode {
        POWER_NORMAL = 0b00,
        POWER_LOW = 0b01,
        POWER_STANDBY = 0b10
    };

    ADXL345();
    void begin(int communicationMode);
    void enableDataReadyInterrupt();
    void enableSingleTapInterrupt(uint16_t tapThreshold, uint16_t tapDuration, uint16_t tapLatent, uint16_t tapWindow);
    void enableDoubleTapInterrupt(uint16_t tapThreshold, uint16_t tapDuration, uint16_t tapLatent, uint16_t tapWindow);
    void enableActivityInterrupt(uint8_t threshold, uint8_t time);
    void enableInactivityInterrupt(uint8_t threshold, uint8_t time);
    void enableFreefallInterrupt(uint8_t threshold, uint8_t time);
    void enableWatermarkInterrupt(uint8_t threshold);
    void enableOverrunInterrupt();
    void setFifoMode(uint8_t fifoMode);
    void setThreshold(int threshold);
    void offsetCalibration();
    void setRange(int temp_range);
    float getRange();
    void setResolution(int temp_resolution);
    void setJustify(int justify);
    void setSelf_test(int test);
    void disableFullRes();
    void enableFullRes();
    void setDataRate(int dataRate);
    float getDataRate();
    void setPowerMode(PowerMode powerMode);
    ADXL345::PowerMode getPowerMode();
    void enableMeasurement();
    void disableMeasurement();
    bool readAcceleration(float& x, float& y, float& z);
    int16_t getX(void), getY(void), getZ(void);

private:
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

    void readRegisters(uint8_t reg, uint8_t count, uint8_t* data);

    PowerMode powerMode;
    //Range range;
    //CommunicationMode communicationMode;
    //DataRate dataRate;
};

#endif