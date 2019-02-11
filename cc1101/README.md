CC1101 driver for Micrpython

Example for STM32-Port using 2 SPI-Ports and 2 CC1101-Modules:
```
spi = pyb.SPI(2,pyb.SPI.MASTER, baudrate=100000, polarity=0, phase=0)
cs = pyb.Pin('PB12', pyb.Pin.OUT_PP)
import cc1101
cc=cc1101.CC1101(spi,cs)
cc.reset()
cc.selfTest()
cc.setConfig(cc.DEF_868_GFSK_38_4_kb)
cc.setRXState()

spi2= pyb.SPI(3,pyb.SPI.MASTER, baudrate=100000, polarity=0, phase=0)
cs2 = pyb.Pin('PB6', pyb.Pin.OUT_PP)
cc2=cc1101.CC1101(spi2,cs2)
cc2.reset()
cc2.selfTest()
cc2.setConfig(cc2.DEF_868_GFSK_38_4_kb)
cc2.sendData([1])

cc.recvData()
```
