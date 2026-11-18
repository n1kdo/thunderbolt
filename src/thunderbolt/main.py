#
# main.py -- this is the Raspberry Pi Pico W Trimble Thunderbolt Network Server.
#
__author__ = 'J. B. Otterson'
__copyright__ = 'Copyright 2023, 2024, 2025 J. B. Otterson N1KDO.'
__version__ = '0.9.9'  # 2026-11-18

#
# Copyright 2024, 2025, 2026 J. B. Otterson N1KDO.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

# disable pylint import error
# pylint: disable=E0401

import asyncio
import time

import micro_logging as logging

from http_server import (HttpServer,
                         HTTP_STATUS_OK, HTTP_STATUS_BAD_REQUEST, HTTP_STATUS_MOVED_PERMANENTLY,
                         HTTP_VERB_GET, HTTP_VERB_POST)
from thunderbolt import Thunderbolt
from morse_code import MorseCode
from utils import upython, safe_int, is_hostname, is_ipv4
from picow_network import PicowNetwork
from config_data import ConfigData


if upython:
    import machine
    try:
        from watchdog import Watchdog
    except ImportError:
        Watchdog = None
else:
    from not_machine import machine
    Watchdog = None

_rtc = machine.RTC()

morse_led = machine.Pin(2, machine.Pin.OUT, value=0)  # morse code LED GP2
failed_led = machine.Pin(16, machine.Pin.OUT, value=0)  # comms failed LED GP16 pin 21
status_led = machine.Pin(15, machine.Pin.OUT, value=0)  # status LED GP15 pin 20
ap_mode_button = machine.Pin(3, machine.Pin.IN, machine.Pin.PULL_UP)  # GP3

CONTENT_DIR = 'content/'

# noinspection SpellCheckingInspection
DEFAULT_SECRET = 'thunderbolt'
DEFAULT_SSID = 'thunderbolt'
DEFAULT_WEB_PORT = 80

# globals...
keep_running = True
thunderbolt = None

# http server
http_server = HttpServer(content_dir=CONTENT_DIR)

config = ConfigData()

# noinspection PyUnusedLocal
@http_server.route(b'/')
async def slash_callback(http, verb, args, reader, writer, request_headers=None):  # callback for '/'
    http_status = HTTP_STATUS_MOVED_PERMANENTLY
    bytes_sent = await http.send_simple_response(writer, http_status, None, None, [b'Location: /thunderbolt.html'])
    return bytes_sent, http_status


# noinspection PyUnusedLocal
@http_server.route(b'/api/config')
async def api_config_callback(http, verb, args, reader, writer, request_headers=None):  # callback for '/api/config'
    if verb == HTTP_VERB_GET:
        payload = dict(config.get_data())
        payload['secret'] = ''  # do not return the secret
        http_status = HTTP_STATUS_OK
        bytes_sent = await http.send_simple_response(writer, http_status, http.CT_APP_JSON, payload)
    elif verb == HTTP_VERB_POST:
        # A POST with no recognized configuration keys has nothing to do. That includes malformed JSON
        # bodies, which the server decodes as an empty dict -- do not pretend they succeeded.
        if not isinstance(args, dict) or not any(key in args for key in ('web_port', 'SSID', 'secret',
                                                                         'hostname', 'dhcp',
                                                                         'ip_address', 'netmask',
                                                                         'gateway', 'dns_server')):
            response = b'no configuration data\r\n'
            http_status = HTTP_STATUS_BAD_REQUEST
            bytes_sent = await http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
            return bytes_sent, http_status
        errors = False
        web_port = args.get('web_port')
        if web_port is not None:
            web_port_int = safe_int(web_port, -2)
            if 1 <= web_port_int <= 65535:
                config['web_port'] = web_port
            else:
                errors = True
                logging.warning('web_port %s not valid' % web_port_int, 'main:api_config_callback')
        ssid = args.get('SSID')
        if ssid is not None:
            if 0 < len(ssid) <= 32:
                config['SSID'] = ssid
            else:
                errors = True
                logging.warning('ssid  %s not valid' % ssid, 'main:api_config_callback')
        secret = args.get('secret')
        if secret is not None and len(secret) > 0:
            if 8 <= len(secret) <= 64:
                config['secret'] = secret
            else:
                errors = True
                logging.warning('secret %s not valid' % secret, 'main:api_config_callback')
        hostname = args.get('hostname')
        if hostname is not None:
            if is_hostname(hostname):
                config['hostname'] = hostname
            else:
                errors = True
                logging.warning('hostname %s not valid' % hostname, 'main:api_config_callback')
        dhcp_arg = args.get('dhcp')
        if dhcp_arg is not None:
            config['dhcp'] = safe_int(dhcp_arg, 0) == 1
        ip_address = args.get('ip_address')
        if ip_address is not None:
            if ip_address == '' or is_ipv4(ip_address):
                config['ip_address'] = ip_address
            else:
                errors = True
                logging.warning('ip_address %s not valid' % ip_address, 'main:api_config_callback')
        netmask = args.get('netmask')
        if netmask is not None:
            if netmask == '' or (is_ipv4(netmask) and netmask != '0.0.0.0'):
                config['netmask'] = netmask
            else:
                errors = True
                logging.warning('netmask %s not valid' % netmask, 'main:api_config_callback')
        gateway = args.get('gateway')
        if gateway is not None:
            if gateway == '' or is_ipv4(gateway):
                config['gateway'] = gateway
            else:
                errors = True
                logging.warning('gateway %s not valid' % gateway, 'main:api_config_callback')
        dns_server = args.get('dns_server')
        if dns_server is not None:
            if dns_server == '' or is_ipv4(dns_server):
                config['dns_server'] = dns_server
            else:
                errors = True
                logging.warning('dns_server %s not valid' % dns_server, 'main:api_config_callback')
        if not errors:
            response = b'ok\r\n'
            http_status = HTTP_STATUS_OK
            bytes_sent = await http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
        else:
            response = b'parameter out of range\r\n'
            http_status = HTTP_STATUS_BAD_REQUEST
            bytes_sent = await http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    else:
        response = b'GET or POST only.'
        http_status = HTTP_STATUS_BAD_REQUEST
        bytes_sent = await http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    return bytes_sent, http_status


# noinspection PyUnusedLocal
@http_server.route(b'/api/restart')
async def api_restart_callback(http, verb, args, reader, writer, request_headers=None):
    global keep_running
    if upython:
        keep_running = False
        response = b'ok\r\n'
        http_status = HTTP_STATUS_OK
        bytes_sent = await http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    else:
        http_status = HTTP_STATUS_BAD_REQUEST
        response = b'not permitted except on PICO-W'
        bytes_sent = await http.send_simple_response(writer, http_status, http.CT_APP_JSON, response)
    return bytes_sent, http_status


# Thunderbolt specific APIs
# noinspection PyUnusedLocal
@http_server.route(b'/api/status')
async def api_status_callback(http, verb, args, reader, writer, request_headers=None):
    if thunderbolt is not None:
        try:
            response = thunderbolt.get_status()
            http_status = HTTP_STATUS_OK
        except Exception as e:
            logging.error('Error in api_status_callback: %s' % e, 'main:api_status_callback')
            response = {'error': 'internal server error during status collection'}
            http_status = HTTP_STATUS_INTERNAL_SERVER_ERROR
    else:
        response = {'error': 'no thunderbolt'}
        http_status = HTTP_STATUS_OK
    bytes_sent = await http.send_simple_response(writer, http_status, http.CT_APP_JSON, response)
    return bytes_sent, http_status


async def main():
    global keep_running, thunderbolt

    logging.info('Starting...', 'main:main')
    time_set = False

    if upython:
        thunderbolt_port = '0'
        ap_mode = ap_mode_button.value() == 0
    else:
        thunderbolt_port = 'com1'
        ap_mode = False

    web_port = safe_int(config.get('web_port') or DEFAULT_WEB_PORT, DEFAULT_WEB_PORT)
    if web_port < 0 or web_port > 65535:
        web_port = DEFAULT_WEB_PORT
        config['web_port'] = str(web_port)

    if upython:
        picow_network = PicowNetwork(config, DEFAULT_SSID, DEFAULT_SECRET, access_point_mode=ap_mode)
        morse_code_sender = MorseCode(morse_led)
        if logging.get_level() != logging.DEBUG and Watchdog is not None:
            _ = Watchdog()

    else:
        picow_network = None
        morse_code_sender = None

    if thunderbolt_port is not None:
        thunderbolt = Thunderbolt(port_name=thunderbolt_port)
        logging.info(f'Starting Thunderbolt serial port service', 'main:main')
        thunderbolt_server = asyncio.create_task(thunderbolt.serial_server())
        thunderbolt_alarms = asyncio.create_task(thunderbolt.alarm_server(status_led, failed_led))

    logging.info(f'Starting web service on port {web_port}', 'main:main')
    web_server = asyncio.create_task(asyncio.start_server(http_server.serve_http_client, '0.0.0.0', web_port))

    four_count = 0
    last_message = b''
    while keep_running:
        if upython:
            await asyncio.sleep(0.25)
            four_count += 1
            if four_count >= 4:  # check for new message every one second
                msg = picow_network.get_message()
                if msg and msg != last_message:
                    last_message = msg
                    morse_code_sender.set_message(msg)
                four_count = 0
            # set the RP2 RTC to the current time... because... why not?
            if not time_set:
                unix_time = thunderbolt.get_unix_time()
                tt = time.gmtime(unix_time)
                if logging.should_log(logging.DEBUG):
                    logging.debug(f'unixtime: {unix_time}, time.time() {time.time()}, tt:{tt}', 'main:main')
                try:
                    _rtc.datetime((tt[0], tt[1], tt[2], tt[6], tt[3], tt[4], tt[5], 0))
                except OSError as e:
                    logging.exception('OSerror setting RTC', 'main:main', exc_info=e)
                if time.time() > 1700000000:
                    time_set = True
        else:
            await asyncio.sleep(10.0)
    if upython:
        # cleanup resources
        if thunderbolt is not None and hasattr(thunderbolt, 'device_port'):
            thunderbolt.device_port.close()
        if picow_network is not None:
            picow_network.deinit()
        config.flush()
        machine.soft_reset()


if __name__ == '__main__':
    logging.set_level(logging.INFO)
    #logging.set_level(logging.DEBUG)
    logging.info('starting', 'main:__main__')
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info('bye', 'main:__main__')
    finally:
        config.flush()
    logging.info('done', 'main:__main__')
