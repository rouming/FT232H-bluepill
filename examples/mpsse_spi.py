#!/usr/bin/python3

#
# FTDI FT232H
#
# https://ftdichip.com/wp-content/uploads/2020/08/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
#

import time, sys
import pylibftdi as ftdi

# Modes
BITMODE_RESET  = 0x00
BITMODE_MPSSE  = 0x02

# Shifting commands IN MPSSE Mode
MPSSE_WRITE_NEG = 0x01   # Write TDI/DO on negative TCK/SK edge
MPSSE_BITMODE   = 0x02   # Write bits, not bytes
MPSSE_READ_NEG  = 0x04   # Sample TDO/DI on negative TCK/SK edge
MPSSE_LSB       = 0x08   # LSB first
MPSSE_DO_WRITE  = 0x10   # Write TDI/DO
MPSSE_DO_READ   = 0x20   # Read TDO/DI
MPSSE_WRITE_TMS = 0x40   # Write TMS/CS

# MPSSE commands
SET_BITS_LOW   = 0x80
SET_BITS_HIGH  = 0x82
GET_BITS_LOW   = 0x81
GET_BITS_HIGH  = 0x83
LOOPBACK_START = 0x84
LOOPBACK_END   = 0x85
TCK_DIVISOR    = 0x86

# H Type specific commands
DIS_DIV_5       = 0x8a
EN_DIV_5        = 0x8b
EN_3_PHASE      = 0x8c
DIS_3_PHASE     = 0x8d
CLK_BITS        = 0x8e
CLK_BYTES       = 0x8f
CLK_WAIT_HIGH   = 0x94
CLK_WAIT_LOW    = 0x95
EN_ADAPTIVE     = 0x96
DIS_ADAPTIVE    = 0x97
CLK_BYTES_OR_HIGH = 0x9c
CLK_BYTES_OR_LOW  = 0x9d

# ADBUS pins
SK_PIN  = 1<<0
DO_PIN  = 1<<1
DI_PIN  = 1<<2
CS_PIN  = 1<<3

def ftdi_set_clock(d, hz):
    div = int((12000000 / (hz * 2)) - 1)
    ftdi_write(d, (TCK_DIVISOR, div%256, div//256))

def ftdi_read(d, nbytes):
    while True:
        # Read unless zero. FTDI periodically returns 2 byte modem
        # status with latency timer period, despite the lack of data
        # in the buffer, so higher level API reports 0 read bytes.
        # One of the solutions is to purge the TX USB buffer
        # (tciflush) or simply wait for data to appear on the pipe.
        s = d.read(nbytes)
        if len(s) > 0:
            break
    return list(s)

def ftdi_write(d, data):
    s = bytes(data)
    r = d.write(s)
    return r

def ftdi_make_hdr(cmd, n):
    n = n - 1
    return (cmd, n%256, n//256)

def ftdi_make_mpsse_pkg(cmd, data):
    hdr = ftdi_make_hdr(cmd, len(data))
    return hdr + tuple(data)

def main():
    d = ftdi.Device()
    d.ftdi_fn.ftdi_set_bitmode(0, BITMODE_MPSSE)

    # Set SPI clock frequency
    ftdi_set_clock(d, 1e6)

    # CS high, other low
    rt = ftdi_write(d, (SET_BITS_LOW, CS_PIN, SK_PIN|DO_PIN|CS_PIN))

    # Data byte
    byte = 0x81

    #
    # SPI mode 0:
    #    - initial clock LOW
    #    - write on NEG, read on POS
    #
    # SPI mode 1 (!!! BROKEN on FT232H and other families):
    #    - initial clock LOW
    #    - write on POS, read on NEG
    #
    # SPI mode 2:
    #    - initial clock HIGH
    #    - write on POS, read on NEG
    #
    # SPI mode 3 (!!! BROKEN on FT232H and other families)::
    #    - initial clock HIGH
    #    - write on NEG, read on POS
    #
    # NOTE: see regarding supported modes in the Application Note AN_114
    #       "It is recommended that designers review the SPI Slave data
    #        sheet to determine the SPI mode implementation.FTDI device
    #        can only support mode 0 and mode 2 due to the limitation of
    #        MPSSE engine."

    # Clear CS, do SPI write/read in mode 1 and set CS in one packet
    wr = ftdi_write(d,
               # CS low
               (SET_BITS_LOW, 0, SK_PIN|DO_PIN|CS_PIN) +
               # SPI write-read
               ftdi_make_mpsse_pkg(MPSSE_DO_READ | MPSSE_DO_WRITE, [byte]) +
               # CS high, other low
               (SET_BITS_LOW, CS_PIN, SK_PIN|DO_PIN|CS_PIN))
    if wr != 10:
        print("Error: unexpected return value on write '%d'" % wr)
        sys.exit(1)

    # Read reply back
    rd = ftdi_read(d, 1)
    if len(rd) != 1:
        print("Error: unexpected number of read bytes '%d'" % len(rd))
        sys.exit(1)

    print("Read: %x" % rd[0])


if __name__ == "__main__":
    main()
