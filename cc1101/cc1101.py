import pyb

class CC1101(object):
    # Configuration Register Details - Registers with preserved values in SLEEP state
    # TI-CC1101 Datasheet

    IOCFG2 = 0x00  # GDO2 Output Pin Configuration
    IOCFG1 = 0x01  # GDO1 Output Pin Configuration
    IOCFG0 = 0x02  # GDO0 Output Pin Configuration
    FIFOTHR = 0x03  # RX FIFO and TX FIFO Thresholds
    SYNC1 = 0x04  # Sync Word, High Byte
    SYNC0 = 0x05  # Sync Word, Low Byte
    PKTLEN = 0x06  # Packet Length
    PKTCTRL1 = 0x07  # Packet Automation Control
    PKTCTRL0 = 0x08  # Packet Automation Control
    ADDR = 0x09  # Device Address
    CHANNR = 0x0A  # Channel Number
    FSCTRL1 = 0x0B  # Frequency Synthesizer Control
    FSCTRL0 = 0x0C  # Frequency Synthesizer Control
    FREQ2 = 0x0D  # Frequency Control Word, High Byte
    FREQ1 = 0x0E  # Frequency Control Word, Middle Byte
    FREQ0 = 0x0F  # Frequency Control Word, Low Byte
    MDMCFG4 = 0x10  # Modem Configuration
    MDMCFG3 = 0x11  # Modem Configuration
    MDMCFG2 = 0x12  # Modem Configuration
    MDMCFG1 = 0x13  # Modem Configuration
    MDMCFG0 = 0x14  # Modem Configuration
    DEVIATN = 0x15  # Modem Deviation Setting
    MCSM2 = 0x16  # Main Radio Control State Machine Configuration
    MCSM1 = 0x17  # Main Radio Control State Machine Configuration
    MCSM0 = 0x18  # Main Radio Control State Machine Configuration
    FOCCFG = 0x19  # Frequency Offset Compensation Configuration
    BSCFG = 0x1A  # Bit Synchronization Configuration
    AGCCTRL2 = 0x1B  # AGC Control
    AGCCTRL1 = 0x1C  # AGC Control
    AGCCTRL0 = 0x1D  # AGC Control
    WOREVT1 = 0x1E  # High Byte Event0 Timeout
    WOREVT0 = 0x1F  # Low Byte Event0 Timeout
    WORCTRL = 0x20  # Wake On Radio Control
    FREND1 = 0x21  # Front End RX Configuration
    FREND0 = 0x22  # Front End TX Configuration
    FSCAL3 = 0x23  # Frequency Synthesizer Calibration
    FSCAL2 = 0x24  # Frequency Synthesizer Calibration
    FSCAL1 = 0x25  # Frequency Synthesizer Calibration
    FSCAL0 = 0x26  # Frequency Synthesizer Calibration
    RCCTRL1 = 0x27  # RC Oscillator Configuration
    RCCTRL0 = 0x28  # RC Oscillator Configuration


    # Configuration Register Details - Registers that Loose Programming in SLEEP State

    FSTEST = 0x29  # Frequency Synthesizer Calibration Control
    PTEST = 0x2A  # Production Test
    AGCTEST = 0x2B  # AGC Test
    TEST2 = 0x2C  # Various Test Settings
    TEST1 = 0x2D  # Various Test Settings
    TEST0 = 0x2E  # Various Test Settings


    # Command Strobe Registers

    SRES = 0x30  # Reset chip
    SFSTXON = 0x31  # Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
    # If in RX (with CCA): Go to a wait state where only the synthesizer
    # is running (for quick RX / TX turnaround).

    SXOFF = 0x32  # Turn off crystal oscillator.
    SCAL = 0x33  # Calibrate frequency synthesizer and turn it off.
    # SCAL can be strobed from IDLE mode without setting manual calibration mode.

    SRX = 0x34  # Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
    STX = 0x35  # In IDLE state: Enable TX. Perform calibration first
    # if MCSM0.FS_AUTOCAL=1.
    # If in RX state and CCA is enabled: Only go to TX if channel is clear.

    SIDLE = 0x36  # Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
    SWOR = 0x38  # Start automatic RX polling sequence (Wake-on-Radio)
    # as described in Section 19.5 if WORCTRL.RC_PD=0.

    SPWD = 0x39  # Enter power down mode when CSn goes high.
    SFRX = 0x3A  # Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
    SFTX = 0x3B  # Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
    SWORRST = 0x3C  # Reset real time clock to Event1 value.
    SNOP = 0x3D  # No operation. May be used to get access to the chip status byte.

    PATABLE = 0x3E  # PATABLE
    TXFIFO = 0x3F  # TXFIFO
    RXFIFO = 0x3F  # RXFIFO


    # Status Register Details
    
    PARTNUM = 0xF0  # Chip ID
    VERSION = 0xF1  # Chip ID
    FREQEST = 0xF2  # Frequency Offset Estimate from Demodulator
    LQI = 0xF3  # Demodulator Estimate for Link Quality
    RSSI = 0xF4  # Received Signal Strength Indication
    MARCSTATE = 0xF5  # Main Radio Control State Machine State
    WORTIME1 = 0xF6  # High Byte of WOR Time
    WORTIME0 = 0xF7  # Low Byte of WOR Time
    PKTSTATUS = 0xF8  # Current GDOx Status and Packet Status
    VCO_VC_DAC = 0xF9  # Current Setting from PLL Calibration Module
    TXBYTES = 0xFA  # Underflow and Number of Bytes
    RXBYTES = 0xFB  # Overflow and Number of Bytes
    RCCTRL1_STATUS = 0xFC  # Last RC Oscillator Calibration Result
    RCCTRL0_STATUS = 0xFD  # Last RC Oscillator Calibration Result

    # Config values
    MOD_2_FSK = 0x00
    MOD_GFSK = 0x01
    MOD_ASK_OOK = 0x03
    MOD_4_FSK = 0x04
    MOD_MSK = 0x07

    PKT_LEN_FIXED = 0x00
    PKT_LEN_VAR = 0x01
    PKT_LEN_INF = 0x02

    ADR_CHK_OFF = 0x00
    ADR_CHK_ON_NO_BRTCAST = 0x01
    ADR_CHK_ON_BRTCAST_00 = 0x02
    ADR_CHK_ON_BRTCAST_00_255 = 0x03
    # Default configuration register arrays
    
    DEF_868_GFSK_1_2_kb = [0x07,0x2E,0x80,0x07,0x57,0x43,0x3E,0x0E,0x45,0xFF,0x00,0x08,0x00,0x21,0x65,0x6A,0xF5,0x83,0x13,0xA0,0xF8,0x15,0x07,0x0C,0x18,0x16,0x6C,0x03,0x40,0x91,0x02,0x26,0x09,0x56,0x17,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x81,0x3F,0x0B]
    DEF_868_GFSK_38_4_kb = [0x07,0x2E,0x80,0x07,0x57,0x43,0x3E,0x0E,0x45,0xFF,0x00,0x06,0x00,0x21,0x65,0x6A,0xCA,0x83,0x13,0xA0,0xF8,0x34,0x07,0x0C,0x18,0x16,0x6C,0x43,0x40,0x91,0x02,0x26,0x09,0x56,0x17,0xA9,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x81,0x3F,0x0B]
    DEF_868_GFSK_100_kb = [0x07,0x2E,0x80,0x07,0x57,0x43,0x3E,0x0E,0x45,0xFF,0x00,0x08,0x00,0x21,0x65,0x6A,0x5B,0xF8,0x13,0xA0,0xF8,0x47,0x07,0x0C,0x18,0x1D,0x1C,0xC7,0x00,0xB2,0x02,0x26,0x09,0xB6,0x17,0xEA,0x0A,0x00,0x11,0x41,0x00,0x59,0x7F,0x3F,0x81,0x3F,0x0B]

    DEF_EQ3_MAX = [0x08,0x29,0x46,0x7,0xC6,0x26,0xFF,0x0C,0x45,0x0,0x0,0x06,0x0,0x21,0x65,0x6A,0xC8,0x93,0x03,0x22,0xF8,0x34,0x07,0x00,0x18,0x16,0x6C,0x43,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xA9,0x0A,0x00,0x11,0x41,0x0,0x59,0x7F,0x3F,0x81,0x35,0xb,0x0,0x87,0xf,0x5f,0x3f,0x1f,0x2f,0xf,0xf,0xf,0x8,0xf,0xf,0xf,0xf,0xC3]
    
    
    def __init__(self, spi, nss, debug=True):
        self.debug = debug
        self.spi = spi
        self.nss = nss

    def usDelay(self, useconds):
        pyb.udelay(useconds)

    def readReg(self, adr):
        self.nss.value(0)
        self.spi.send(adr | 0x80)
        data = self.spi.read(1)
        self.nss.value(1)
        return data[0]

    def writeReg(self, adr, data):
        self.nss.value(0)
        self.spi.send(adr & 0x3f)
        self.spi.send(data)
        self.nss.value(1)
        
    def writeBurst(self, adr, data):
        self.nss.value(0)
        self.spi.send(adr | 0x40)
        self.spi.send(bytes(data))
        self.nss.value(1)

    def readBurst(self, adr, datalen):
        self.nss.value(0)
        self.spi.send(adr | 0xC0)
        data = self.spi.read(datalen)
        self.nss.value(1)
        return data
        
    def strobe(self, adr):
        self.nss.value(0)
        self.spi.send(adr)
        self.spi.send(0x00)
        self.nss.value(1)
    
    def reset(self):
        self.strobe(self.SRES)
        
    def selfTest(self):
        part_number = self.readReg(self.PARTNUM)
        component_version = self.readReg(self.VERSION)

        # These asserts are based on the documentation
        # Section 29.3 "Status Register Details"
        # On reset PARTNUM == 0x00
        # On reset VERSION == 0x14

        assert part_number == 0x00
        assert component_version == 0x14

        if self.debug:
            print ("Part Number: %x" % part_number)
            print ("Component Version: %x" % component_version)
            print ("Self test OK")

    def sidle(self):
        self.strobe(self.SIDLE)

        while (self.readReg(self.MARCSTATE) != 0x01):
            self.usDelay(100)

        self.strobe(self.SFTX)
        self.usDelay(100)

    def powerDown(self):
        self.sidle()
        self.strobe(self.SPWD)

    def setCarrierFrequency(self, freq=868e6):
        fr = int(freq / (26e6/(2**16)))
        self.writeReg(self.FREQ2, (fr >> 16)& 0xFF)
        self.writeReg(self.FREQ1, (fr >> 8)& 0xFF)
        self.writeReg(self.FREQ0, fr & 0xFF)

    def setChannel(self, channel=0x00):
        self.writeReg(self.CHANNR, channel)

    def setSyncWord(self, sync_word="FAFA"):
        assert len(sync_word) == 4

        self.writeReg(self.SYNC1, int(sync_word[:2], 16))
        self.writeReg(self.SYNC0, int(sync_word[2:], 16))

    def setConfig(self, config_arr):
        self.writeBurst(self.IOCFG2, config_arr)

    def setSyncMode(self, syncmode):
        val = self.readReg(self.MDMCFG2)
        self.writeReg(self.MDMCFG2, (val & 0xF8) | syncmode & 0x07)

    def setModulation(self, modulation):
        val = self.readReg(self.MDMCFG2)
        self.writeReg(self.MDMCFG2, (val & 0x8F) | ((modulation & 0x7)<<4))

    def flushRXFifo(self):
        self.strobe(self.SFRX)
        self.usDelay(2)

    def flushTXFifo(self):
        self.strobe(self.SFTX)
        self.usDelay(2)

    def setTXState(self):
        self.strobe(self.STX)
        self.usDelay(2)

    def setRXState(self):
        self.strobe(self.SRX)
        self.usDelay(2)

    def getRSSI(self):
        return self.readReg(self.RSSI)

    def getState(self):
        return (self.readReg(self.MARCSTATE) & 0x1F)

    def getPacketMode(self):
        return self.readReg(self.PKTCTRL0) & 0x03

    def setPacketMode(self, mode=PKT_LEN_VAR):
        val = self.readReg(self.PKTCTRL0)
        self.writeReg(self.PKTCTRL0, (val & 0xFC) | (val & 0x03))

    def setFilteringAddress(self, address=0x0E):
        self.writeReg(self.ADDR, address)

    def configureAddressCheck(self, value=ADR_CHK_OFF):
        val = self.readReg(self.PKTCTRL1)
        self.writeReg(self.PKTCTRL1, (val & 0xFC) | (value & 0x03))

    def sendData(self, dataBytes):
        self.setRXState()
        marcstate = self.getState()
        dataToSend = []

        while ((marcstate & 0x1F) != 0x0D):
            if self.debug:
                print ("marcstate = %x" % marcstate)
                print ("waiting for marcstate == 0x0D")

            if marcstate == 0x11:
                self.flushRXFifo()

            marcstate = self.getState()

        if len(dataBytes) == 0:
            if self.debug:
                print ("sendData | No data to send")
            return False

        sending_mode = self.getPacketMode()
        data_len = len(dataBytes)

        if sending_mode == self.PKT_LEN_FIXED:
            if data_len > self.readReg(self.PKTLEN):
                if self.debug:
                    print ("Len of data exceeds the configured packet len")
                return False

            if self.readReg(self.PKTCTRL1)!=self.ADR_CHK_OFF:
                dataToSend.append(self.readReg(self.ADDR))


            dataToSend.extend(dataBytes)
            dataToSend.extend([0] * (self.readReg(self.PKTLEN) - len(dataToSend)))

            if self.debug:
                print ("Sending a fixed len packet")
                print ("data len = %d" % (data_len))

        elif sending_mode == self.PKT_LEN_VAR:
            dataToSend.append(data_len)

            if self.readReg(self.PKTCTRL1)!=self.ADR_CHK_OFF:
                dataToSend.append(self.readReg(self.ADDR))
                dataToSend[0] += 1

            dataToSend.extend(dataBytes)

            if self.debug:
                print ("Sending a variable len packet")
                print ("Length of the packet is: %d" % data_len)

        elif sending_mode == self.PKT_LEN_INF:
            # ToDo
            raise Exception("MODE NOT IMPLEMENTED")

        print (dataToSend)
        self.writeBurst(self.TXFIFO, dataToSend)
        self.usDelay(2000)
        self.setTXState()
        marcstate = self.getState()

        if marcstate not in [0x13, 0x14, 0x15]:  # RX, RX_END, RX_RST
            self.sidle()
            self.flushTXFifo()
            self.setRXState()

            if self.debug:
                print ("senData | FAIL")
                print ("sendData | MARCSTATE: %x" % self.readReg(self.MARCSTATE))

            return False

        remaining_bytes = self.readReg(self.TXBYTES) & 0x7F
        while remaining_bytes != 0:
            self.usDelay(1000)
            remaining_bytes = self.readReg(self.TXBYTES) & 0x7F
            if self.debug:
                print ("Waiting until all bytes are transmited, remaining bytes: %d" % remaining_bytes)


        if (self.readReg(self.TXBYTES) & 0x7F) == 0:
            if self.debug:
                print ("Packet sent!")

            return True

        else:
            if self.debug:
                print (self.readReg(self.TXBYTES) & 0x7F)
                print ("sendData | MARCSTATE: %x" % self.getState())
                self.sidle()
                self.flushTXFifo()
                pyb.sleep(5)
                self.setRXState()

            return False

    def recvData(self):
        rx_bytes_val = self.readReg(self.RXBYTES)

        if (rx_bytes_val & 0x7F and not (rx_bytes_val & 0x80)):
            sending_mode = self.getPacketMode()

            if sending_mode == self.PKT_LEN_FIXED:
                data_len = self.readReg(self.PKTLEN)

            elif sending_mode == self.PKT_LEN_VAR:
                max_len = self.readReg(self.PKTLEN)
                data_len = self.readReg(self.RXFIFO)

                if data_len > max_len:
                    if self.debug:
                        print ("Len of data exceeds the configured maximum packet len")
                    return False

                if self.debug:
                    print ("Receiving a variable len packet")
                    print ("max len: %d" % max_len)
                    print ("Packet length: %d" % data_len)

            elif sending_mode == self.PKT_LEN_INF:
                # ToDo
                raise Exception("MODE NOT IMPLEMENTED")

            data = self.readBurst(self.RXFIFO, data_len)
            
            if (self.readReg(self.PKTCTRL1) & 0x04) == 0x04:
            # When enabled, two status bytes will be appended to the payload of the
            # packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

                rssi = self.readReg(self.RXFIFO)
                val = self.readReg(self.RXFIFO)
                lqi = val & 0x7f

                if self.debug:
                    print ("Packet information is enabled")
                    print ("RSSI: %d" % (rssi))
                    print ("VAL: %d" % (val))
                    print ("LQI: %d" % (lqi))
            if self.debug:
                print ("Data: " + str(data))

            return data