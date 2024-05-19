# Trimble Thunderbolt interface

import sys
import struct

from serialport import SerialPort
from utils import upython

if upython:
    import micro_logging as logging
    import uasyncio as asyncio
else:
    import asyncio
    import micro_logging as logging
#    import logging


# message bytes
ETX = 0x03  # end of message
DLE = 0x10  # start of message


class Thunderbolt:

    def __init__(self, port_name):
        logging.debug(f'Initializing Thunderbolt class, port_name={port_name}')
        self.port_name = port_name
        self.device_port = SerialPort(name=port_name, baudrate=9600, timeout=0)  # timeout is zero for non-blocking
        self.run = True
        # thunderbolt data
        self.pdop = 0.0
        self.hdop = 0.0
        self.vdop = 0.0
        self.tdop = 0.0
        self.satellites = []
        self.fix_dim = 0
        self.tm = ''

    async def serial_server(self):
        logging.debug('hi there')
        RS_INIT = 0  # initial reader state, waiting for DLE
        RS_READ = 1  # reading data into buffer
        RS_READ_DLE = 2  # reading data, last was DLE
        BUFFER_SIZE = 128
        reader_state = RS_INIT
        buffer = bytearray(BUFFER_SIZE)
        offset = 0
        device_port = self.device_port

        while self.run:
            if device_port.any():
                bs = device_port.read(1)
                b = bs[0]
                # print(f'{reader_state}, {b:02x}, {offset}')
                if reader_state == RS_INIT:
                    if b == DLE:
                        reader_state = RS_READ
                        offset = 0
                elif reader_state == RS_READ:
                    if b == DLE:
                        reader_state = RS_READ_DLE
                    else:
                        buffer[offset] = b
                        offset += 1
                elif reader_state == RS_READ_DLE:
                    if b == ETX:
                        self.process_buffer(buffer, offset)
                        reader_state = RS_INIT
                        offset = 0
                    else:
                        buffer[offset] = b
                        offset += 1
                        reader_state = RS_READ
                else:
                    logging.error('Unhandled reader state')
                if offset >= BUFFER_SIZE:
                    logging.error('buffer overrun!', 'thunderbolt:serial_server')
                    offset = 0
                    reader_state = RS_INIT
            else:
                await asyncio.sleep(0.01)

    def process_buffer(self, buffer, offset):
        logging.loglevel = logging.WARNING  # TODO FIXME
        pkt = buffer[0]
        print(pkt)
        try:
            if pkt == 0x13:   # I don't know what this packet it.  it is not in the doc.
                logging.debug(f'packet 0x013, len={offset}', 'thunderbolt:process_buffer:0x13')
                logging.debug('\n' + hexdump_buffer(buffer[:offset]))
            elif pkt == 0x45:
                # see Section A.9.22 in Thunderbolt book, page A-17
                # this packet is not normally sent, but it is requested by Lady Heather.
                logging.debug(f'software version information, len={offset}', 'thunderbolt:process_buffer:0x45')
                logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                #fmt = '>xBBBBBBBBBB'
                #print(f'fmt = {fmt}')
                #print(f'size={struct.calcsize(fmt)}')
                #stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 app major version
                # 01 app minor version
                # 02 Month
                # 03 Day
                # 04 Year - 2000
                # 05 GPS Core major mersion
                # 06 GPS core minor version
                # 07 Month
                # 08 Day
                # 09 year - 2000
                #print(f'stuff = {stuff}')
                # don't care about this data right now.
            elif pkt == 0x47:
                # see Section A.9.22 in Thunderbolt book, page A-17
                # this packet is not normally sent, but it is requested by Lady Heather.
                logging.debug(f'satellite signal levels list, len={offset}', 'thunderbolt:process_buffer:0x47')
                logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                count = buffer[1]
                fmt = '>xB' + ('Bf' * count)
                # print(count)
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 number of satellites
                # 01 satellite[0] number
                # 02 satellite[0] signal level
                # ...
                # print(stuff)
                i = 1
                signal_strengths = {}
                while i < len(stuff):
                    signal_strengths[stuff[i]] = round(stuff[i + 1], 1)
                    i += 2
                # print(signal_strengths)
            elif pkt == 0x49:
                # see Section A.9.23 in Thunderbolt book, page A-18
                # this packet is not normally sent, but it is requested by Lady Heather.
                logging.debug(f'Almanac Health Page, len={offset}', 'thunderbolt:process_buffer:0x47')
                logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                #        01234567890123456789012345678901  32 satellites
                fmt = '>xBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                #stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 - 31 byte containing 6-bits of almanac health data for 32 satellites.
                # print(stuff)
                # don't care about this right now.
            # 0x55
            # 0x57
            # 0x58
            # 0x59
            # 0x5a
            # 0x5c
            # 0x5f
            # 0x70
            # 0xbb
            elif pkt == 0x6d:
                # see Section A.9.34 in Thunderbolt Book, page A-29
                logging.debug(f'satellite selection list, len={offset}', 'thunderbolt:process_buffer:0x6d')
                #print(hexdump_buffer(buffer[:offset]))
                num_sats = offset - 18
                fmt = '>xBffff' + 'b' * num_sats
                # fmt = '>xcffff' + 'b' * num_sats
                #print(f'fmt = {fmt}')
                #print(f'size={struct.calcsize(fmt)}')
                stuff = struct.unpack(fmt, buffer[:offset])
                # print(stuff)
                bm = stuff[0]
                # results include
                # 00 bit field uchar
                # 01 PDOP float
                # 02 HDOP float
                # 03 VDOP float
                # 04 PDOP float
                # 05-end PRN # char (list)
                fix_dim = (bm & 0xe0) >> 5
                if fix_dim == 1:
                    self.fix_dim = 1  # 1d fix
                elif fix_dim == 3:
                    self.fix_dim = 2  # 2d fix
                elif fix_dim == 4:
                    self.fix_dim = 3  # 3d fix
                else:
                    logging.warning(f'fix_dim {fix_dim} not implemented', 'thunderbolt:process_buffer:0x6d')
                    logging.warning(f'bm bits: {bm:08b}, fix_dim bits: {fix_dim:03b}', 'thunderbolt:process_buffer:0x6d')
                    self.fix_dim = 0
                num_sats = bm & 0x0f
                logging.info(f'stuff[0] = {bm:02x}, fix_dim = {self.fix_dim}, num_sats = {num_sats}, bm={bm:08b}',
                             'thunderbolt:process_buffer:0x6d')
                self.pdop = stuff[1]
                self.hdop = stuff[2]
                self.vdop = stuff[3]
                self.tdop = stuff[4]
                self.satellites = list(stuff[5:])
                #print(stuff)
                #print(self.fix_dim, self.pdop, self.hdop, self.vdop, self.tdop, self.satellites)
                #print()
            elif pkt == 0x8f:
                subc = buffer[1]
                if subc == 0xa7:
                    # see Section A.10.29 in Thunderbolt Book, page A-53
                    logging.debug(f'individual satellite solutions, len={offset}', 'thunderbolt:process_buffer')
                    # print(hexdump_buffer(buffer[:offset]))
                    format = buffer[2]
                    if format == 0:  #  mode 0, floating point
                        num_sats = int((offset - 15) / 5)
                        #print(num_sats)
                        fmt = '>xxBIff' + ('B' * num_sats) + ('f' * num_sats)
                        #print(f'fmt = {fmt}')
                        #print(f'size={struct.calcsize(fmt)}')
                        stuff = struct.unpack(fmt, buffer[:offset])
                        #print(stuff)
                    else:
                        logging.error('Unhandled format 1', 'process_buffer')
                elif subc == 0xab:
                    # see Section A.10.30 in Thunderbolt Book, page A-56
                    logging.debug(f'primary timing packet, len={offset}', 'thunderbolt:process_buffer:0x8f 0xab')
                    # print(hexdump_buffer(buffer[:offset]))
                    fmt = '>xxIHhBBBBBBH'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    stuff = struct.unpack(fmt, buffer[:offset])
                    self.tm = f'{stuff[6]:02d}:{stuff[5]:02d}:{stuff[4]:02d}'
                    logging.info(f'time: {self.tm}', 'thunderbolt:process_buffer:0x8f 0xab')
                elif subc == 0xac:
                    # see Section A.10.31 in Thunderbolt Book, page A-59
                    logging.debug(f'secondary timing packet, len={offset}','thunderbolt:process_buffer')
                    # print(hexdump_buffer(buffer[:offset]))
                    fmt = '>xxBBBIHHBBBBffIffdddxxxxxxxx'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # results contain tuple of
                    #  0 receiver mode uint8
                    #  1 disciplining mode uint8
                    #  2 self-survey progress uint8
                    #  3 holdover duration sec uint32
                    #  4 critical alarms bitmask uint16
                    #  5 minor alarms bitmask uint16
                    #  6 gps decoding status uint8
                    #  7 disciplining activity uint8
                    #  8 spare status 1 uint8
                    #  9 spare status 2 uint8
                    # 10 pps offset ns float
                    # 11 10 mhz offset PPB float
                    # 12 DAC value uint32
                    # 13 DAC voltage volts float
                    # 14 temperature degrees C float
                    # 15 latitude radians double
                    # 16 longitude radians double
                    # 17 altitude meters double
                    #    8 bytes ignored
                    stuff = struct.unpack(fmt, buffer[:offset])
                    # print(stuff)
                    logging.info(f'receiver mode {stuff[0]}, disciplining mode {stuff[1]}, critical alarms {stuff[4]}, minor alarms {stuff[5]}, gps status {stuff[6]}',
                                 'thunderbolt:process_buffer:0x8f 0xac')
                else:
                    logging.debug(f'unknown packet type 8f {subc:02x}', 'thunderbolt:process_buffer')
                    logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
            else:
                logging.warning(f'unknown packet type {pkt:02x}', 'thunderbolt:process_buffer')
                logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
        except struct.error as exc:
            logging.error(exc)


def buffer_to_hexes(buffer):
    return ' '.join('{:02x}'.format(b) for b in buffer)


def hexdump_buffer(buffer):
    result = ''
    hex_bytes = ''
    printable = ''
    offset = 0
    ofs = '{:04x}'.format(offset)
    for b in buffer:
        hex_bytes += '{:02x} '.format(b)
        printable += chr(b) if 32 <= b <= 126 else '.'
        offset += 1
        if len(hex_bytes) >= 48:
            result += ofs + '  ' + hex_bytes + '  ' + printable +'\n'
            hex_bytes = ''
            printable = ''
            ofs = '{:04x}'.format(offset)
    if len(hex_bytes) > 0:
        hex_bytes += ' ' * 47
        hex_bytes = hex_bytes[0:47]
        result += ofs + '  ' + hex_bytes + '   ' + printable + '\n'
    return result


def dump_buffer(name, buffer, dump_all=False):
    if len(buffer) < 5:
        dump_all = True
    if dump_all:
        print('{} message {}'.format(name, buffer_to_hexes(buffer)))
    else:
        print('{} payload {}'.format(name, buffer_to_hexes(buffer[2:-2])))

