# Boardoza ADXL345 Accelerometer Sensor Breakout Board

The ADXL345 Triple Axis Accelerometer Breakout Board is a high-performance accelerometer designed for dynamic acceleration measurement and static angle detection. It supports both SPI and I<sup>2</sup>C  interfaces, providing versatile options for integration into various applications, including motion detection, gaming, robotics, and industrial instrumentation.

## [Click here to purchase!](https://www.ozdisan.com/maker-and-iot-products/boardoza/boardoza-modules/ADXL345-BREAKOUT-BOARD/1065561)

|Front Side|Back Side|
|:---:|:---:|
| ![ADXL345 Front](./assets/ADXL345%20Front.png)| ![ADXL345 Back](./assets/ADXL345%20Back.png)|

---

## Key Features

- High Resolution: Up to 13-bit resolution allows for measurement of inclination changes less than 1.0°.
- Triple Axis Measurement: Measures acceleration on X, Y, and Z axes.
- Wide Range: It can measure the static acceleration of gravity in tilt-sensing applications, as well as dynamic acceleration resulting from motion or shock.
- Adjustable Sensitivity: It has selectable measurement ranges of ±2g, ±4g, ±8g, and ±16g.
- Ultra Low Power Consumption: Suitable for battery-powered devices.
  
---

## Technical Specifications

**Power Input Type:** Molex 2 pin 2.50 header

**Input Voltage:**	3.3V or 5V

**Functions:**	Digital accelerometer

**Measurement Range:** ±2g, ±4g, ±8g, and ±16g

**Resolution:** 10-bit (default), 13-bit (high resolution mode)

**Interface:** I<sup>2</sup>C, SPI (3 and 4 wire)

**Operating Temperature:**	-40°C ~ +85°C

**Board Dimensions:**	20mm x 20mm

---

## Board Pinout

| J1 Pin Number | Pin Name | Description |
| :---: | --- | --- |
| 1 | GND | Ground |
| 2 | SDA/SDI/SDIO | I<sup>2</sup>C - Serial Data (I2C)/Serial Data Input (SPI 4-Wire)/Serial Data Input and Output (SPI 3-Wire).  |
| 3 | SCL/SCLK |  Serial Communications Clock for I<sup>2</sup>C and SPI |
| 4 | +5V | Power Supply |

| Pin Header Pin Number | Pin Name | Description |
| :---: | --- | --- |
| 1 | CS | Chip Select |
| 2 | SDO / ADDR | Serial Data Output (SPI 4-Wire)/Alternate I2C Address Select (I2C) |
| 3 | INT1 | Interrupt 1 |
| 4 | INT2 | Interrupt 2 |

---

## Board Dimensions

<img src="./assets/ADXL345 Dimension.png" alt="ADXL345 Dimension" width="350"/>

---

## Step Files

[Boardoza ADXL345.step](./assets/ADXL345%20Step.step)

---

## Datasheet

[ADXL345 Datasheet.pdf](./assets/ADXL345%20Datasheet.pdf)

---

## Version History

- V1.0.0 - Initial Release

---

## Support

- If you have any questions or need support, please contact <support@boardoza.com>

---

## License

Shield: [![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg
