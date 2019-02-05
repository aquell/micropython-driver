import pyb

CMD_MEASURE_TEMP = 0xF3
CMD_MEASURE_HUM = 0xF5
CMD_READ_TEMP = 0xE0
CMD_RESET = 0xFE
CMD_READ_USER = 0xE6
CMD_FIRMWARE_V = b'\x84\xb8'
CMD_READ_SERIAL1 = b'\xFA\x0F'
CMD_READ_SERIAL2 = b'\xFC\xC9'

class SI7021:
    def __init__(self, i2c):
        self.i2c=i2c
        self.addr=64

    def exec_cmd(self, cmd, size):
        self.i2c.send(cmd, self.addr)
        while(self.i2c.is_ready(self.addr)==False):
            pyb.delay(1)
        tmp = self.i2c.recv(size, self.addr)
        return tmp

    def measure_t(self):
        dat=self.exec_cmd(CMD_MEASURE_TEMP, 3)
        return ((dat[0]<<8)+dat[1])*175.72/65536-46.85

    def read_t(self):
        dat=self.exec_cmd(CMD_READ_TEMP, 3)
        return ((dat[0]<<8)+dat[1])*175.72/65536-46.85

    def measure_h(self):
        dat=self.exec_cmd(CMD_MEASURE_HUM, 3)
        return ((dat[0]<<8)+dat[1])*125/65536-6