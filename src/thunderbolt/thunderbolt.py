# Trimble Thunderbolt interface
__author__ = 'J. B. Otterson'
__copyright__ = """
Copyright 2024, J. B. Otterson N1KDO.
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, 
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice, 
     this list of conditions and the following disclaimer in the documentation 
     and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
"""
__version__ = '0.0.9'
"""
This was written by listening to the serial output of a Trimble Thunderbolt,
both when running standalone, and also when being "managed" by 
Lady Heather's Disciplined Oscillator Control Program 5.00.

There is code to parse data out of most of the TSIP messages, every one that I saw during development.
A lot of the data parsing is commented out in the interest of time and space.
"""

import gc
import struct

from serialport import SerialPort
from utils import upython, milliseconds, get_timestamp_from_secs

if upython:
    import micro_logging as logging
    import uasyncio as asyncio
else:
    import asyncio
    import micro_logging as logging

# message bytes
ETX = 0x03  # end of message
DLE = 0x10  # start of message

# these are packets that I don't care about.  They are safe to ignore.
ignore_packets = [0x43, 0x45, 0x47, 0x49,
                  0x55, 0x56, 0x57, 0x58,
                  0x59, 0x5a, 0x5b, 0x5c,
                  0x5f, 0x70, 0x83, 0x84,
                  0xbb]
ignore_8f_packets = [0x15, 0x41, 0x42, 0x4a,
                     0x4c, 0xa0, 0xa1, 0xa2,
                     0xa5, 0xa7, 0xa8, 0xa9]

# state machine states
RS_INIT = 0  # initial reader state, waiting for DLE
RS_READ = 1  # reading data into buffer
RS_READ_DLE = 2  # reading data, last was DLE

# size of serial buffer
BUFFER_SIZE = 256  # message 0x58 can be 170 bytes.

# GPS Epoch date (January 6, 1980 at 00:00Z) as Unix Time
GPS_EPOCH_AS_UNIX_TIME = 315964800
WEEK_SECONDS = 7 * 86400

DEGREES_RADIAN = 57.29578
FEET_METER = 3.2808398950131


class Thunderbolt:
    def __init__(self, port_name):
        logging.debug(f'Initializing Thunderbolt class, port_name={port_name}')
        self.port_name = port_name
        self.device_port = SerialPort(name=port_name, baudrate=9600, timeout=0)  # timeout is zero for non-blocking
        self.run = True
        self.connected = False
        # thunderbolt data
        self.receiver_mode = -1
        self.discipline_mode = -1
        self.holdover_duration = -1
        self.gps_status = -1
        self.minor_alarms = 0
        self.critical_alarms = 0
        self.latitude = -1
        self.longitude = -1
        self.altitude = -1
        self.satellites = []
        self.fix_dim = 0
        self.time_of_week = 0
        self.week_number = 0
        self.utc_offset = 0
        self.tm = ''
        self.last_seen_tm = 0

    def get_status(self):
        return {'thunderbolt_data': {
            'connected': self.connected,
            'receiver_mode': self.receiver_mode,
            'discipline_mode': self.discipline_mode,
            'holdover_duration': self.holdover_duration,
            'gps_status': self.gps_status,
            'minor_alarms': self.minor_alarms,
            'critical_alarms': self.critical_alarms,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'satellites': self.satellites,
            'fix_dim': self.fix_dim,
            'time_of_week': self.time_of_week,
            'week_number': self.week_number,
            'utc_offset': self.utc_offset,
            'unixtime': self.get_datetime(),
            'time': self.tm,
        }
        }

    async def alarm_server(self, status_led, failed_led):
        while self.run:
            await asyncio.sleep(1.0)
            if self.last_seen_tm > milliseconds() - 5000:
                self.connected = True
                failed_led.off()
            else:
                self.connected = False
                failed_led.on()
            if self.connected and self.minor_alarms == 0:
                status_led.on()
            else:
                status_led.off()

    def get_datetime(self):
        unix_time = GPS_EPOCH_AS_UNIX_TIME + self.week_number * WEEK_SECONDS + self.time_of_week - self.utc_offset
        return get_timestamp_from_secs(unix_time)

    async def serial_server(self):
        reader_state = RS_INIT
        buffer = bytearray(BUFFER_SIZE)
        raw = bytearray(BUFFER_SIZE)
        raw_offset = 0
        offset = 0
        device_port = self.device_port

        # send init (8E A5) message to enable the messages I want.
        message = b'\x10\x8e\xa5\x00\x45\x00\x00\x10\x03'
        device_port.write(message)
        device_port.flush()

        while self.run:
            if device_port.any():
                bs = device_port.read(1)
                b = bs[0]
                raw[raw_offset] = b
                raw_offset += 1
                # print(f'{reader_state}, {b:02x}, {offset}')
                if reader_state == RS_INIT:
                    if b == DLE:
                        if offset != 0:
                            logging.warning('data lost.', 'thunderbolt:serial_server:RS_INIT')
                            logging.warning(f'buffer {offset}:\n' + hexdump_buffer(buffer[:offset]))
                        reader_state = RS_READ
                        offset = 0
                        raw[0] = b
                        raw_offset = 1
                elif reader_state == RS_READ:
                    if b == DLE:
                        reader_state = RS_READ_DLE
                    else:
                        buffer[offset] = b
                        offset += 1
                elif reader_state == RS_READ_DLE:
                    if b == ETX:
                        self.last_seen_tm = milliseconds()  # there is serial traffic
                        if not self.process_buffer(buffer, offset):
                            print('error processing buffer!')
                            print('raw bytes:\n' + hexdump_buffer(raw[0:raw_offset]))
                        reader_state = RS_INIT
                        offset = 0
                        raw_offset = 0
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
                if raw_offset >= BUFFER_SIZE:
                    logging.error('raw (debugging) buffer overflow!')
                    logging.error('raw_buffer:\n' + hexdump_buffer(raw))
                    raw_offset = 0
            else:
                await asyncio.sleep(0.020)  # wait 20 ms for more traffic

    def process_buffer(self, buffer, offset):
        pkt_id = buffer[0]
        # logging.debug(f'packet ID {pkt_id:02x}', 'Thunderbolt:process_buffer')
        if pkt_id in ignore_packets:
            return True
        try:
            if pkt_id == 0x13:
                # This is documented in the Thunderbolt E GPS Disciplined Clock User Guide, page 41
                # It indicates that a packet was received that the Thunderbolt does not recognise.
                bad_cmd = buffer[1]
                if bad_cmd == 0x1c:
                    # command 0x1c is not supported by Thunderbolt, but is supported by Thunderbolt-E
                    # quietly eat this error report.
                    return True
                elif bad_cmd == 0x3c:  # request satellite tracking status
                    sat_number = buffer[2]
                    if sat_number > 32:
                        #  Lady Heather appears to have a bug and is requesting satellite tracking
                        #  for satellites numbered > 32.  This should not be reported.
                        return True
                elif bad_cmd == 0x8e:
                    sub_cmd = buffer[2]
                    if sub_cmd == 0x4e:
                        #  0x8e 0x4e (PPS output qualifier) is not supported by Thunderbolt,
                        #  but is supported by Thunderbolt-E. Quietly eat this error.
                        return True
                logging.warning(f'unparsable packet 0x013, len={offset}', 'thunderbolt:process_buffer:0x13')
                logging.warning('\n' + hexdump_buffer(buffer[:offset]))
            elif pkt_id == 0x6d:
                # see Section A.9.34 in Thunderbolt Book, page A-29
                # logging.debug(f'satellite selection list, len={offset}', 'thunderbolt:process_buffer:0x6d')
                # print(hexdump_buffer(buffer[:offset]))
                num_sats = offset - 18
                # logging.debug(f'num_sats={num_sats}', 'thunderbolt:process_buffer:0x6d')
                fmt = '>xBffff' + 'b' * num_sats
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                stuff = struct.unpack(fmt, buffer[:offset])
                # print(stuff)
                bm = stuff[0]
                # logging.info(f'bits={bm:08b}', 'thunderbolt:process_buffer:0x6d')
                # results include
                # 00 bit field uchar
                # 01 PDOP float
                # 02 HDOP float
                # 03 VDOP float
                # 04 PDOP float
                # 05-end PRN # char (list)
                fix_dim = (bm & 0x07)
                if fix_dim == 0:
                    self.fix_dim = 0  # no fix
                elif fix_dim == 1:
                    self.fix_dim = 1  # 1d Clock fix
                elif fix_dim == 3:
                    self.fix_dim = 2  # 2d fix
                elif fix_dim == 4:
                    self.fix_dim = 3  # 3d fix
                elif fix_dim == 5:
                    self.fix_dim = 5  # OD Clock Fix
                else:
                    logging.warning(f'fix_dim {fix_dim} not implemented', 'thunderbolt:process_buffer:0x6d')
                    logging.warning(f'bm bits: {bm:08b}, fix_dim bits: {fix_dim:03b}',
                                    'thunderbolt:process_buffer:0x6d')
                    self.fix_dim = 0
                num_sats = (bm & 0xf0) >> 4
                # logging.debug(f'fix_dim = {self.fix_dim}, num_sats = {num_sats}',  'thunderbolt:process_buffer:0x6d')
                self.satellites = sorted(list(stuff[5:]))
                # print(stuff)
                # print(self.fix_dim, self.pdop, self.hdop, self.vdop, self.tdop, self.satellites)
                # print()
            elif pkt_id == 0x8f:
                pkt_sub_id = buffer[1]
                if pkt_sub_id in ignore_8f_packets:
                    return True
                # logging.debug(f'packet ID 8f {pkt_sub_id:02x}', 'Thunderbolt:process_buffer')

                if pkt_sub_id == 0xab:
                    # see Section A.10.30 in Thunderbolt Book, page A-56
                    # logging.debug(f'primary timing packet, len={offset}', 'thunderbolt:process_buffer:0x8f 0xab')
                    # print(hexdump_buffer(buffer[:offset]))
                    fmt = '>xxIHhBBBBBBH'
                    # results include:
                    # 00 time-of-week
                    # 01 week number
                    # 02 UTC offset seconds
                    # 03 timing flag
                    # 04 seconds
                    # 05 minutes
                    # 06 hours
                    # 07 day of month
                    # 08 month
                    # 09 year
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    stuff = struct.unpack(fmt, buffer[:offset])
                    # print(stuff)
                    self.time_of_week = stuff[0]
                    self.week_number = stuff[1] + 1024  # week number has wrapped again.
                    self.utc_offset = stuff[2]
                    # print(f'{stuff[3]}')
                    # calculate time as unix time
                    self.tm = f'{stuff[6]:02d}:{stuff[5]:02d}:{stuff[4]:02d}'
                    # logging.debug(f'time: {self.tm}', 'thunderbolt:process_buffer:0x8f 0xab')
                    self.connected = True
                elif pkt_sub_id == 0xac:
                    # see Section A.10.31 in Thunderbolt Book, page A-59
                    # logging.debug(f'secondary timing packet, len={offset}', 'thunderbolt::0x8f 0xac')
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
                    # logging.debug(f'receiver mode {stuff[0]}, disciplining mode {stuff[1]}, critical alarms {stuff[4]}, minor alarms {stuff[5]}, gps status {stuff[6]}',                                  'thunderbolt:process_buffer:0x8f 0xac')
                    self.receiver_mode = stuff[0]
                    self.discipline_mode = stuff[1]
                    self.holdover_duration = stuff[3]
                    self.critical_alarms = stuff[4]
                    self.minor_alarms = stuff[5]
                    self.gps_status = stuff[6]
                    self.latitude = stuff[15]
                    self.longitude = stuff[16]
                    self.altitude = stuff[17]
                else:
                    logging.warning(f'unknown packet type 8f {pkt_sub_id:02x}', 'thunderbolt:process_buffer')
                    logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    return False
            else:
                logging.warning(f'unknown packet type {pkt_id:02x}', 'thunderbolt:process_buffer')
                logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                return False
        except Exception as exc:
            logging.error(str(exc), 'thunderbolt:process_buffer:Exception')
        gc.collect()
        return True


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
            result += ofs + '  ' + hex_bytes + '  ' + printable + '\n'
            hex_bytes = ''
            printable = ''
            ofs = '{:04x}'.format(offset)
    if len(hex_bytes) > 0:
        hex_bytes += ' ' * 47
        hex_bytes = hex_bytes[0:47]
        result += ofs + '  ' + hex_bytes + '   ' + printable + '\n'
    return result
