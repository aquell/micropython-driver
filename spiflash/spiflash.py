import pyb

CMD_JEDEC_ID = 0x9F
CMD_READ_STATUS = 0x05 # Read status register
CMD_READ = 0x03 # Read @ low speed
CMD_READ_HI_SPEED = 0x0B # Read @ high speed
CMD_WRITE_ENABLE = 0x06 # Write enable
CMD_PROGRAM_PAGE = 0x02 # Write page
CMD_ERASE_4k = 0x20
CMD_ERASE_32k = 0x52
CMD_ERASE_64k = 0xD8
CMD_ERASE_CHIP = 0xC7
CMD_READ_UID = 0x4B
PAGE_SIZE = 256

cmds = {'4k':CMD_ERASE_4k, '32k':CMD_ERASE_32k, '64k':CMD_ERASE_64k}

class SPIFlash:
    def __init__(self, spi, cs):
        self.spi = spi;
        self.cs = cs;
        self.cs.high()
        
    def read_block(self, addr,buff):
        self.cs.low()
        self.spi.send(CMD_READ)
        self.spi.send(addr>>16)
        self.spi.send(addr>>8)
        self.spi.send(addr)
        self.spi.recv(buff)
        self.cs.high()

    def getid(self):
        self.cs.low()
        self.spi.send(CMD_JEDEC_ID)    # id
        r = self.spi.recv(3)
        self.cs.high()
        return r

    def wait(self):
      while True:
        self.cs.low()
        self.spi.send(CMD_READ_STATUS)
        r = self.spi.recv(1)[0]
        self.cs.high()
        if (r == 0) :
            return

    def write_block(self, addr,buff):
        # write in 256-byte chunks
        # could check that doesn't go past end of flash ...
        length = len(buff)
        pos = 0
        while (pos < length):
            size = min(length-pos, PAGE_SIZE)
            self.cs.low()
            self.spi.send(CMD_WRITE_ENABLE)
            self.cs.high()
            print ("write enable")
            self.cs.low()
            self.spi.send(CMD_PROGRAM_PAGE)
            self.spi.send(addr>>16)
            self.spi.send(addr>>8)
            self.spi.send(addr)
            self.spi.send(buff[pos:pos+size])
            self.cs.high()
            self.wait()
            addr += size
            pos += size

    def erase(self,cmd,addr):
        self.cs.low()
        self.spi.send(CMD_WRITE_ENABLE)
        self.cs.high()
        print ("write enable")
        self.cs.low()
        self.spi.send(cmds[cmd])
        self.spi.send(addr>>16)
        self.spi.send(addr>>8)
        self.spi.send(addr)
        self.cs.high()
        t = pyb.micros()
        self.wait()
        t = pyb.micros() - t
        print ("erase: ",cmd,addr,t,'us')

class SPIFLASHBlockDev:
    def __init__(self, spiflash, block_size, num_blocks):
        self.spiflash = spiflash
        self.block_size = block_size
        self.num_blocks = num_blocks

    def readblocks(self, block_num, buf):
        print ("readblock: ", block_num)
        addr = block_num*self.block_size
        self.spiflash.read_block(addr, buf)
        
    def writeblocks(self, block_num, buf):
        addr=block_num*self.block_size
        self.spiflash.erase('4k', addr)
        print ("writeblock: ",block_num)
        self.spiflash.write_block(addr, buf)

    def ioctl(self, op, arg):
        print ("ioctl: ", op ,", ", arg)
        if op == 3:
            return 0
        if op == 4: # get number of blocks
            return self.num_blocks
        if op == 5: # get block size
            return self.block_size