# TMAG5273
TMAG5273 magnetic sensor Arduino driver

Based on the TI [datasheet](https://www.ti.com.cn/lit/ds/symlink/tmag5273.pdf?ts=1668154869141&ref_url=https%253A%252F%252Fwww.ti.com.cn%252Fproduct%252Fzh-cn%252FTMAG5273%253FkeyMatch%253DTMAG5273%2526tisearch%253Dsearch-everything%2526usecase%253DGPN).

In current design the INT pin is not used and connected to the ground.

Features
* Config many IC modes without directly operating the registers.
* Read the magnetic data and temperature data in standard I2C read mode (stand-by mode or continuous measure mode).
* Read the magnetic data and temperature data in standard I2C read mode (continuous measure mode).
* Config all devices simultaneously by general call write.
* Change sensor I2C address.

TODO
* Operating sensor array.
* Support I2C read mode MODE_SENSOR8 1-byte read command for 8-bit data.
* Support angle read.
* Support CRC check.
* Support interrupt.
