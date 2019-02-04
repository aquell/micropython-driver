SPI-Flash driver for Micrpython

Example for STM32-Port:
```
import spiflash
import uos
spi = pyb.SPI(1,pyb.SPI.MASTER, baudrate=10000000, polarity=0, phase=0)
cs = pyb.Pin('PC15', pyb.Pin.OUT_PP)
spf=spiflash.SPIFlash(spi,cs)
fdev=spiflash.SPIFLASHBlockDev(spf, 4096,256)
uos.VfsFat.mkfs(fdev)
vfs = uos.VfsFat(fdev)
uos.mount(vfs,'/fdisk')
```
