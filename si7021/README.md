SPI-Flash driver for Micrpython

Example for STM32-Port:
```
from pyb import I2C
i2c = I2C(3, I2C.MASTER, baudrate=100000)
import si7021
si=si7021.SI7021(i2c)
si.measure_t()
si.measure_h()
si.read_t()
```
