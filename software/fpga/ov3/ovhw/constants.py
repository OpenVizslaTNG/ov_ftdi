RXCMD_MAGIC_SOP = 0x40
RXCMD_MAGIC_EOP = 0x41     # Packet ended with no error indication
RXCMD_MAGIC_OVF = 0x43     # Packet overflowed in RX path and was clipped
RXCMD_MAGIC_NOP = 0x44

RXCMD_MASK = 0xBF

# USB 2.0 ECN: Link Power Management (LPM) Table 2-1 PID Types
PID_EXT = 0b0000
# Universal Serial Bus Specification Revision 2.0 Table 8-1. PID Types
PID_OUT = 0b0001
PID_ACK = 0b0010
PID_DATA0 = 0b0011
PID_PING = 0b0100
PID_SOF = 0b0101
PID_NYET = 0b0110
PID_DATA2 = 0b0111
PID_SPLIT = 0b1000
PID_IN = 0b1001
PID_NAK = 0b1010
PID_DATA1 = 0b1011
PID_PRE_ERR = 0b1100
PID_SETUP = 0b1101
PID_STALL = 0b1110
PID_MDATA = 0b1111

# 1 byte PID + (HS interrupt/isochronous) 1024 bytes data + 2 byte CRC
MAX_PACKET_SIZE = 1027

# Timeout, in 100 MHz clocks, after which non-full host burst packet can be sent
FLUSH_TIMEOUT = 10000000

# Single byte filler sent inside frame to prevent data trapping on SDRAM write
FILLER_MAGIC = 0xA1
FILLER_TIMEOUT = FLUSH_TIMEOUT//2

#  Physical layer error
HF0_ERR =  0x01

# RX Path Overflow
HF0_OVF =  0x02

# Clipped by Filter
HF0_CLIP = 0x04

# Clipped due to packet length (> MAX_PACKET_SIZE bytes)
HF0_TRUNC = 0x08

# First packet of capture session; IE, when the cap hardware was enabled
HF0_FIRST = 0x10

# Last packet of capture session; IE, when the cap hardware was disabled
HF0_LAST = 0x20

# Transceiver speed
HF0_SPEED_MASK = 0xC0
HF0_SPEED_SHIFT = 6
