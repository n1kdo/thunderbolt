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
__version__ = '0.0.1'
"""
This was written by listening to the serial output of a Trimble Thunderbolt,
both when running standalone, and also when being "managed" by 
Lady Heather's Disciplined Oscillator Control Program 5.00.

There is code to parse data out of most of the TSIP messages, every one that I saw during development.
A lot of the data parsing is commented out in the interest of time and space.
"""

import struct
from datetime import datetime, timezone

from serialport import SerialPort
from utils import upython, milliseconds

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
ignore_packets = [0x43, 0x45, 0x47, 0x49, 0x55, 0x56, 0x57, 0x58, 0x59,
                  0x5a, 0x5b, 0x5c, 0x5f, 0x70, 0x83, 0x84, 0xbb, ]
ignore_8f_packets = [0x15, 0x41, 0x42, 0x4a, 0x4c, 0xa0, 0xa1, 0xa2, 0xa5, ]

# state machine states
RS_INIT = 0  # initial reader state, waiting for DLE
RS_READ = 1  # reading data into buffer
RS_READ_DLE = 2  # reading data, last was DLE

# size of serial buffer
BUFFER_SIZE = 256  # message 0x58 can be 170 bytes.

# GPS Epoch date (January 6, 1980 at 00:00Z) as Unix Time
GPS_EPOCH_AS_UNIX_TIME = 315964800


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
        self.unixtime = -1
        self.last_unixtime = -1
        self.tm = ''
        self.last_seen_tm = 0

    def get_status(self):
        return {
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
            'unixtime': self.unixtime,
            'time': self.tm,
        }

    async def alarm_server(self):
        while self.run:
            await asyncio.sleep(1.0)
            if self.last_seen_tm > milliseconds() - 60000:
                self.connected = True
            else:
                self.connected = False

    def get_datetime(self):
        return datetime.fromtimestamp(self.unixtime, timezone.utc)

    async def serial_server(self):
        reader_state = RS_INIT
        buffer = bytearray(BUFFER_SIZE)
        raw = bytearray(BUFFER_SIZE)
        raw_offset = 0
        offset = 0
        device_port = self.device_port

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
        # logging.loglevel = logging.WARNING  # TODO FIXME
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
            elif pkt_id == 0x43:
                # see Section A.9.20 in Thunderbolt book, page A-20
                # this packet is not normally sent, but it is sent when doing a survey.
                pass
                # logging.info(f'Velocity Fix, XYZ ECEF, len={offset}', 'thunderbolt:process_buffer:0x43')
                # logging.info('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xfffff'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 x velocity
                # 01 y velocity
                # 02 z velocity
                # 03 bias rate
                # 04 time of fix
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x45:
                # see Section A.9.22 in Thunderbolt book, page A-17
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'software version information, len={offset}', 'thunderbolt:process_buffer:0x45')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBBBBBBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
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
                # print(f'stuff = {stuff}')
                # don't care about this data right now.
            elif pkt_id == 0x47:
                # see Section A.9.22 in Thunderbolt book, page A-17
                # this packet is not normally sent, but it is requested by Lady Heather.
                # logging.debug(f'satellite signal levels list, len={offset}', 'thunderbolt:process_buffer:0x47')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
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
            elif pkt_id == 0x49:
                # see Section A.9.23 in Thunderbolt book, page A-18
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Almanac Health Page, len={offset}', 'thunderbolt:process_buffer:0x49')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                #        01234567890123456789012345678901  32 satellites
                # fmt = '>xBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                #  = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 - 31 byte containing 6-bits of almanac health data for 32 satellites.
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x55:
                # see Section A.9.25 in Thunderbolt book, page A-19
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.info(f'I/O options, len={offset}', 'thunderbolt:process_buffer:0x55')
                # logging.info('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 position bitmap
                # 01 velocity bitmap
                # 02 timing bitmap
                # 03 aux bitmap
                # print(f'{stuff[0]:08b} {stuff[1]:08b} {stuff[2]:08b} {stuff[3]:08b}')
                # don't care about this right now.
            elif pkt_id == 0x56:
                # see Section A.9.26 in Thunderbolt book, page A-20
                # this packet is not normally sent, but it is sent when doing a survey.
                pass
                # logging.info(f'Velocity Fix, East-North-Up, len={offset}', 'thunderbolt:process_buffer:0x56')
                # logging.info('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xfffff'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 east velocity
                # 01 north velocity
                # 02 up velocity
                # 03 clock bias rate
                # 04 time of fix
                # print(f'{stuff}')
                # don't care about this right now.
            elif pkt_id == 0x57:
                # see Section A.9.27 in Thunderbolt book, page A-20
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Information about Last Computed Fix, len={offset}', 'thunderbolt:process_buffer:0x57')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBfH'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 source of info
                # 01 tracking mode
                # 02 time of last fix
                # 03 week of last fix
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x58:
                # see Section A.9.32 in Thunderbolt book, page A-26
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'GPS System Data from Receiver, len={offset}', 'thunderbolt:process_buffer:0x58')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # don't care about this right now.
            elif pkt_id == 0x59:
                # see Section A.9.29 in Thunderbolt book, page A-24
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Status of Satellite Disable or Ignore Health, len={offset}', 'thunderbolt:process_buffer:0x59')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 operation
                # 01-33 bitmask
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x5a:
                # see Section A.9.30 in Thunderbolt book, page A-25
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Raw Measurement Data, len={offset}', 'thunderbolt:process_buffer:0x5a')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBffffd'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 SV PRN number
                # 01 sample length
                # 02 signal level
                # 03 code phase
                # 04 doppler
                # 05 time of measurement
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x5b:
                # see Section A.9.31 in Thunderbolt book, page A-25
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Satellite Ephemeris Status, len={offset}', 'thunderbolt:process_buffer:0x5b')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBfBBfBf'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 SV PRN number
                # 01 time of collection sec
                # 02 health
                # 03 IODE number (whatever that is)
                # 04 toe (Sec)
                # 05 fix interval flag
                # 06 SV accruracy meters
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x5c:
                # see Section A.9.32 in Thunderbolt book, page A-26
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Satellite Tracking Status, len={offset}', 'thunderbolt:process_buffer:0x5c')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBBBffffBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 SV PRN number
                # 01 bitmasks: slot #, channel #
                # 02 acquisition flag
                # 03 ephemeris flag
                # 04 signal level
                # 05 time of last measurement
                # 06 elevation angle radians
                # 07 azimuth angle radian
                # 08 old measurement flag
                # 09 integer msec flag
                # 10 bad data flag
                # 11 data collection flag
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x5f:
                # see Section A.9.33 in Thunderbolt book, page A-28
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'EEPROM Segment Status, len={offset}', 'thunderbolt:process_buffer:0x5f')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBH'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 0x11 marker must be 0x11
                # 01 16-bit segment status bitmask
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x6d:
                # see Section A.9.34 in Thunderbolt Book, page A-29
                # logging.debug(f'satellite selection list, len={offset}', 'thunderbolt:process_buffer:0x6d')
                # print(hexdump_buffer(buffer[:offset]))
                num_sats = offset - 18
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
                if fix_dim == 1:
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
            elif pkt_id == 0x70:
                # see Section A.9.36 in Thunderbolt book, page A-30
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.debug(f'Filter Configuration, len={offset}', 'thunderbolt:process_buffer:0x70')
                # logging.debug('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 PV filter
                # 01 Static filter
                # 02 Altitude filter
                # 03 reserved
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x83:
                # see Section A.9.38 in Thunderbolt book, page A-31
                # this packet is not normally sent, but it is sent when doing a survey.
                pass
                # logging.info(f'Double Precision XYZ ECEF Position Fix, len={offset}', 'thunderbolt:process_buffer:0x83')
                # logging.info('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xddddf'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 X
                # 01 Y
                # 02 Z
                # 03 clock bias
                # 04 time of fix
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x84:
                # see Section A.9.38 in Thunderbolt book, page A-31
                # this packet is not normally sent, but it is sent when doing a survey.
                pass
                # logging.info(f'Double Precision LLA Position Fix, len={offset}', 'thunderbolt:process_buffer:0x84')
                # logging.info('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xddddf'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 latitude
                # 01 longitude
                # 02 altitude
                # 03 clock bias
                # 04 time of fix
                # print(stuff)
                # don't care about this right now.
            elif pkt_id == 0x8f:
                pkt_sub_id = buffer[1]
                if pkt_sub_id in ignore_8f_packets:
                    return True
                if pkt_sub_id == 0x15:
                    # this command is not in the Thunderbolt Book, the data here came from page 75 of
                    # the Thunderbolt E GPS Disciplined Clock User Guide
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'UTC/GPS Timing Mode, len={offset}', 'thunderbolt:process_buffer:0x8f 0x15')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxHddddd'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 datum index
                    # 01 dx
                    # 02 dy
                    # 03 dz
                    # 04 A-axis
                    # 05 eccentricity squared
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0x41:
                    # see Section A.10.17 in Thunderbolt book, page A-47
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'Stored Manufacturing Operating Parameters, len={offset}', 'thunderbolt:process_buffer:0x8f 0x41')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxhIBBBBfH'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 board serial number prefix
                    # 01 board serial number
                    # 02 year of build
                    # 03 month of build
                    # 04 day of build
                    # 05 hour of build
                    # 06 oscillator offset
                    # 07 test code identification number
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0x42:
                    # see Section A.10.18 in Thunderbolt book, page A-48
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'Stored Production Parameters, len={offset}', hunderbolt:process_buffer:0x8f 0x42')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxBBHIIHHH'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 production options prefix
                    # 01 Production number extension
                    # 02 Case serial number prefix
                    # 03 Case serial number
                    # 04 Production number
                    # 05 reserved
                    # 06 Machine identification number
                    # 07 reserved
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0x4a:
                    # see Section A.10.19 in Thunderbolt book, page A-48
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'PPS Characteristics, len={offset}', 'thunderbolt:process_buffer:0x8f 0x4a')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxBBBdf'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 PPS output enable
                    # 01 reserved
                    # 02 PPS polarity
                    # 03 PPS offset or cable delay seconds
                    # 04 bias uncertainly threshold or offset meters
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0x4c:
                    # see Section A.10.18 in Thunderbolt book, page A-48
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'Save Segments to EEPROM, len={offset}', 'thunderbolt:process_buffer:0x8f 0x4c')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxB'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 segment number
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0xa0:
                    # see Section A.10.22 in Thunderbolt book, page A-50
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'DAC values, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa0')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxIfBBff'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 DAC value
                    # 01 DAC voltage
                    # 02 DAC resolution
                    # 03 DAC data format 0=offset binary, 1=2's complement
                    # 04 min DAC value, volts
                    # 05 max DAV value, volts
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0xa1:
                    # see Section A.10.23 in Thunderbolt book, page A-51
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'10 MHz sense, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa1')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxB'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 10 MHz sense
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0xa2:
                    # see Section A.10.23 in Thunderbolt book, page A-51
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # .warning(f'UTC/GPS Timing Mode, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa2')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxB'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 bitmap
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0xa5:
                    # see Section A.10.26 in Thunderbolt book, page A-53
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.warning(f'UTC/GPS Timing Mode, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa5')
                    # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxHH'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 Mask 0 bitmap
                    # 01 Mask 2 bitmap reserved
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0xa7:
                    # see Section A.10.29 in Thunderbolt Book, page A-53
                    # logging.debug(f'individual satellite solutions, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa7')
                    # print(hexdump_buffer(buffer[:offset]))
                    msg_fmt = buffer[2]
                    if msg_fmt == 0:  # mode 0, floating point
                        pass
                        # num_sats = int((offset - 15) / 5)
                        # print(num_sats)
                        # fmt = '>xxBIff' + ('B' * num_sats) + ('f' * num_sats)
                        # print(f'fmt = {fmt}')
                        # print(f'size={struct.calcsize(fmt)}')
                        # stuff = struct.unpack(fmt, buffer[:offset])
                        # print(stuff)
                    else:
                        logging.error(f'Unhandled format {msg_fmt}', 'thunderbolt:process_buffer:0x8f 0xa7')
                elif pkt_sub_id == 0xa8:
                    # see Section A.10.28 in Thunderbolt book, page A-55
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    if False:
                        logging.warning(f'Disciplining Parameters, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa8')
                        logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                        typ = buffer[2]
                        if typ == 0:  # type 0, loop dynamics
                            fmt = '>xxxff'
                            print(f'fmt = {fmt}')
                            print(f'size={struct.calcsize(fmt)}')
                            stuff = struct.unpack(fmt, buffer[:offset])
                            # results include:
                            # 00 seconds
                            # 01 dimensionless
                            print(stuff)
                        elif typ == 1:  # type 1 oscillator parameters
                            fmt = '>xxxfff'
                            print(f'fmt = {fmt}')
                            print(f'size={struct.calcsize(fmt)}')
                            stuff = struct.unpack(fmt, buffer[:offset])
                            # results include:
                            # 00 Hz/Volt
                            # 01 minimum control voltage, volts
                            # 02 maximum control voltage, volts
                            print(stuff)
                        elif typ == 2:  # type 2 recovery mode parameters
                            fmt = '>xxxff'
                            print(f'fmt = {fmt}')
                            print(f'size={struct.calcsize(fmt)}')
                            stuff = struct.unpack(fmt, buffer[:offset])
                            # results include:
                            # 00 jam sync threshold ns
                            # 01 maximum frequency offset PPB
                            print(stuff)
                        elif typ == 3:  # type 1 recovery mode parameters
                            fmt = '>xxxf'
                            print(f'fmt = {fmt}')
                            print(f'size={struct.calcsize(fmt)}')
                            stuff = struct.unpack(fmt, buffer[:offset])
                            # results include:
                            # 00 initial DAC voltage
                            print(stuff)
                        elif typ == 4:  # type 4 is not documented in the Thunderbolt Book V5.
                            # I think this is the DAC lower and upper bounds.
                            fmt = '>xxxff'
                            print(f'fmt = {fmt}')
                            print(f'size={struct.calcsize(fmt)}')
                            stuff = struct.unpack(fmt, buffer[:offset])
                            # results include:
                            # 00 minimum DAC voltage
                            # 01 minimum DAC voltage
                            print(f'WHAT is this stuff: {stuff}')
                        else:
                            print(f'unhandled type {typ}')
                        # don't care about this right now.
                elif pkt_sub_id == 0xa9:
                    # see Section A.10.29 in Thunderbolt Book, page A-56
                    # this packet is not normally sent, but it is requested by Lady Heather.
                    pass
                    # logging.debug(f'Self-Survey Parameters, len={offset}', 'thunderbolt:process_buffer:0x8f 0xa9')
                    # print(hexdump_buffer(buffer[:offset]))
                    # fmt = '>xxBBII'
                    # print(f'fmt = {fmt}')
                    # print(f'size={struct.calcsize(fmt)}')
                    # stuff = struct.unpack(fmt, buffer[:offset])
                    # results include:
                    # 00 self-survey enable
                    # 01 position save flag
                    # 02 number of fixes
                    # 03 reserved
                    # print(stuff)
                    # don't care about this right now.
                elif pkt_sub_id == 0xab:
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
                    time_of_week = stuff[0]
                    week_number = stuff[1]
                    # fix week number underflow
                    while week_number < 2048:
                        week_number += 1024
                    # calculate time as unix time
                    self.unixtime = GPS_EPOCH_AS_UNIX_TIME + week_number * 7 * 86400 + time_of_week
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
                    self.latitude = round(stuff[15] * 57.29578, 4)
                    self.longitude = round(stuff[16] * 57.29578, 4)
                    self.altitude = round(stuff[17] * 3.2808398950131, 1)  # meters to Feet
                else:
                    logging.warning(f'unknown packet type 8f {pkt_sub_id:02x}', 'thunderbolt:process_buffer')
                    logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                    return False
            elif pkt_id == 0xbb:
                # see Section A.9.39 in Thunderbolt book, page A-28
                # this packet is not normally sent, but it is requested by Lady Heather.
                pass
                # logging.warning(f'Request or Set GPS Receiver Configuration, len={offset}', 'thunderbolt:process_buffer:0xbb')
                # logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                # fmt = '>xBBBffffBBBBBBBBBBBBBBBBBBBBB'
                # print(f'fmt = {fmt}')
                # print(f'size={struct.calcsize(fmt)}, offset={offset}')
                # stuff = struct.unpack(fmt, buffer[:offset])
                # results include:
                # 00 subcode
                # 01 receiver mode
                # 02 reserved
                # 03 dynamics code
                # 04 reserved
                # 05 elevation mask
                # 06 AMU mask
                # 07 PDOP mask
                # 08 PDOP switch
                # 09 reserved
                # 10 foliage mode
                # 11 reserved
                # print(stuff)
                # don't care about this right now.
            else:
                logging.warning(f'unknown packet type {pkt_id:02x}', 'thunderbolt:process_buffer')
                logging.warning('buffer:\n' + hexdump_buffer(buffer[:offset]))
                return False
        except struct.error as exc:
            logging.error(exc)
            # exit(1)  # DIE HORRIBLY WHILE DEBUGGING.
            return False
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
