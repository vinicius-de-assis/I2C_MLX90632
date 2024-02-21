# Overview of the Program Functionality
This program establishes an I2C communication link between an MLX90632 temperature sensor and an ESP32 S3 microcontroller unit (MCU). 
The primary objective is to facilitate the transfer of temperature data readings from the MLX90632 sensor to the ESP32 S3 MCU for processing and analysis.

# Software
Specified Drivers are available on [melexis.github.io/mlx90632-library/tree/1.0.1](https://github.com/melexis/mlx90632-library/tree/1.0.1)
Compiled documentation is available on [melexis.github.io/mlx90632-library](https://melexis.github.io/mlx90632-library/).
Datasheet of mlx90632 is available in [Melexis documentation](https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90632).
Datasheet of Esp32 S3 is available in [Espressif.com](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf).

# Hardware connections
| MLX90632        | ESP32 S3               |
| --------------- |:----------------------:| 
| pin 1: SDA      | GPIO04: I2C_SDA        | 
| pin 2: VDD      | GPIO05: I2C_SCL        |
| pin 3: GND      | GPIO41: VDD_TEMP       |
| pin 4: SCL      |                        |