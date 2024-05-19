#
# main.py -- this is the Raspberry Pi Pico W KAT500 & KPA500 Network Server.
#
__author__ = 'J. B. Otterson'
__copyright__ = 'Copyright 2023, 2024 J. B. Otterson N1KDO.'
__version__ = '0.9.1'

#
# Copyright 2023, 2024 J. B. Otterson N1KDO.
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

import json

from http_server import (HttpServer,
                         api_rename_file_callback,
                         api_remove_file_callback,
                         api_upload_file_callback,
                         api_get_files_callback)
from thunderbolt import Thunderbolt
from morse_code import MorseCode
from utils import upython, safe_int
from picow_network import connect_to_network


if upython:
    import machine
    import micro_logging as logging
    import uasyncio as asyncio
else:
    import asyncio
#    import logging
    import micro_logging as logging


    class Machine:
        """
        fake micropython stuff
        """

        @staticmethod
        def soft_reset():
            logging.debug('Machine.soft_reset()', 'main:Machine.soft_reset()')

        @staticmethod
        def reset():
            logging.debug('Machine.reset()', 'main:Machine.reset()')

        class Pin:
            OUT = 1
            IN = 0
            PULL_UP = 0

            def __init__(self, name, options=0, value=0):
                self.name = name
                self.options = options
                self.state = value

            def on(self):
                self.state = 1

            def off(self):
                self.state = 0

            def value(self):
                return self.state


    machine = Machine()

onboard = machine.Pin('LED', machine.Pin.OUT, value=0)
morse_led = machine.Pin(2, machine.Pin.OUT, value=0)  # status LED
reset_button = machine.Pin(3, machine.Pin.IN, machine.Pin.PULL_UP)

BUFFER_SIZE = 4096
CONFIG_FILE = 'data/config.json'
DANGER_ZONE_FILE_NAMES = (
    'config.html',
    'files.html',
    'kat500.html',
    'kpa500.html',
)
# noinspection SpellCheckingInspection
DEFAULT_SECRET = 'thunderbolt'
DEFAULT_SSID = 'thunderbolt'
DEFAULT_KPA500_TCP_PORT = 4626
DEFAULT_KAT500_TCP_PORT = 4627
DEFAULT_WEB_PORT = 80

# globals...
keep_running = True
username = ''
password = ''
http_server = HttpServer(content_dir='content/')
thunderbolt = None

morse_code_sender = MorseCode(morse_led)


def read_config():
    try:
        with open(CONFIG_FILE, 'r') as config_file:
            config = json.load(config_file)
    except Exception as ex:
        logging.error(f'failed to load configuration! {type(ex)} {ex}', 'main:read_config')
        config = {
            'SSID': DEFAULT_SSID,
            'secret': DEFAULT_SSID,
            'username': 'admin',
            'password': 'admin',
            'dhcp': True,
            'hostname': 'thunderbolt',
            'ip_address': '192.168.1.73',
            'netmask': '255.255.255.0',
            'gateway': '192.168.1.1',
            'dns_server': '8.8.8.8',
            'web_port': str(DEFAULT_WEB_PORT),
        }
    return config


def save_config(config):
    with open(CONFIG_FILE, 'w') as config_file:
        json.dump(config, config_file)


# noinspection PyUnusedLocal
async def slash_callback(http, verb, args, reader, writer, request_headers=None):  # callback for '/'
    http_status = 301
    bytes_sent = http.send_simple_response(writer, http_status, None, None, ['Location: /thunderbolt.html'])
    return bytes_sent, http_status


# noinspection PyUnusedLocal
async def api_config_callback(http, verb, args, reader, writer, request_headers=None):  # callback for '/api/config'
    if verb == 'GET':
        payload = read_config()
        payload.pop('secret')  # do not return the secret
        response = json.dumps(payload).encode('utf-8')
        http_status = 200
        bytes_sent = http.send_simple_response(writer, http_status, http.CT_APP_JSON, response)
    elif verb == 'POST':
        config = read_config()
        dirty = False
        errors = False
        tcp_port = args.get('tcp_port')
        if tcp_port is not None:
            tcp_port_int = safe_int(tcp_port, -2)
            if 0 <= tcp_port_int <= 65535:
                config['tcp_port'] = tcp_port
                dirty = True
            else:
                errors = True
        web_port = args.get('web_port')
        if web_port is not None:
            web_port_int = safe_int(web_port, -2)
            if 0 <= web_port_int <= 65535:
                config['web_port'] = web_port
                dirty = True
            else:
                errors = True
        ssid = args.get('SSID')
        if ssid is not None:
            if 0 < len(ssid) < 64:
                config['SSID'] = ssid
                dirty = True
            else:
                errors = True
        secret = args.get('secret')
        if secret is not None:
            if 8 <= len(secret) < 32:
                config['secret'] = secret
                dirty = True
            else:
                errors = True
        remote_username = args.get('username')
        if remote_username is not None:
            if 1 <= len(remote_username) <= 16:
                config['username'] = remote_username
                dirty = True
            else:
                errors = True
        remote_password = args.get('password')
        if remote_password is not None:
            if 1 <= len(remote_password) <= 16:
                config['password'] = remote_password
                dirty = True
            else:
                errors = True
        ap_mode_arg = args.get('ap_mode')
        if ap_mode_arg is not None:
            ap_mode = ap_mode_arg == '1'
            config['ap_mode'] = ap_mode
            dirty = True
        dhcp_arg = args.get('dhcp')
        if dhcp_arg is not None:
            dhcp = dhcp_arg == 1
            config['dhcp'] = dhcp
            dirty = True
        hostname = args.get('hostname')
        if hostname is not None:
            if 1 <= len(hostname) <= 16:
                config['hostname'] = hostname
                dirty = True
            else:
                errors = True
        ip_address = args.get('ip_address')
        if ip_address is not None:
            config['ip_address'] = ip_address
            dirty = True
        netmask = args.get('netmask')
        if netmask is not None:
            config['netmask'] = netmask
            dirty = True
        gateway = args.get('gateway')
        if gateway is not None:
            config['gateway'] = gateway
            dirty = True
        dns_server = args.get('dns_server')
        if dns_server is not None:
            config['dns_server'] = dns_server
            dirty = True
        if not errors:
            if dirty:
                save_config(config)
            response = b'ok\r\n'
            http_status = 200
            bytes_sent = http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
        else:
            response = b'parameter out of range\r\n'
            http_status = 400
            bytes_sent = http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    else:
        response = b'GET or PUT only.'
        http_status = 400
        bytes_sent = http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    return bytes_sent, http_status


# noinspection PyUnusedLocal
async def api_restart_callback(http, verb, args, reader, writer, request_headers=None):
    global keep_running
    if upython:
        keep_running = False
        response = b'ok\r\n'
        http_status = 200
        bytes_sent = http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    else:
        http_status = 400
        response = b'not permitted except on PICO-W'
        bytes_sent = http.send_simple_response(writer, http_status, http.CT_APP_JSON, response)
    return bytes_sent, http_status


# Thunderbolt specific APIs
# noinspection PyUnusedLocal
async def api_status_callback(http, verb, args, reader, writer, request_headers=None):
    response = b'ok\r\n'
    http_status = 200
    bytes_sent = http.send_simple_response(writer, http_status, http.CT_TEXT_TEXT, response)
    return bytes_sent, http_status


async def main():
    global keep_running, username, password, thunderbolt

    logging.info('Starting...', 'main:main')

    config = read_config()
    username = config.get('username')
    password = config.get('password')

    if upython:
        thunderbolt_port = '0'
    else:
        thunderbolt_port = 'com4'

    web_port = safe_int(config.get('web_port') or DEFAULT_WEB_PORT, DEFAULT_WEB_PORT)
    if web_port < 0 or web_port > 65535:
        web_port = DEFAULT_WEB_PORT
        config['web_port'] = str(web_port)

    ap_mode = config.get('ap_mode', False)

    connected = True
    if upython:
        try:
            ip_address = connect_to_network(config, DEFAULT_SSID, DEFAULT_SECRET, morse_code_sender)
            connected = ip_address is not None
        except Exception as ex:
            logging.error(f'Network did not connect, {ex}', 'main:main')

        morse_sender_task = asyncio.create_task(morse_code_sender.morse_sender())

    if connected:
        http_server.add_uri_callback('/', slash_callback)
        http_server.add_uri_callback('/api/config', api_config_callback)
        http_server.add_uri_callback('/api/get_files', api_get_files_callback)
        http_server.add_uri_callback('/api/upload_file', api_upload_file_callback)
        http_server.add_uri_callback('/api/remove_file', api_remove_file_callback)
        http_server.add_uri_callback('/api/rename_file', api_rename_file_callback)
        http_server.add_uri_callback('/api/restart', api_restart_callback)

        # KPA500 specific
        if thunderbolt_port is not None:
            thunderbolt = Thunderbolt(port_name=thunderbolt_port)
#            http_server.add_uri_callback('/api/kpa_clear_fault', api_kpa_clear_fault_callback)
            http_server.add_uri_callback('/api/status', api_status_callback)
            # this task talks to the thunderbolt hardware.
            logging.info(f'Starting Thunderbolt serial port service', 'main:main')
            thunderbolt_server = asyncio.create_task(thunderbolt.serial_server())

        logging.info(f'Starting web service on port {web_port}', 'main:main')
        web_server = asyncio.create_task(asyncio.start_server(http_server.serve_http_client, '0.0.0.0', web_port))
    else:
        logging.error('no network connection', 'main:main')

    reset_button_pressed_count = 0
    while keep_running:
        if upython:
            await asyncio.sleep(0.25)
            pressed = reset_button.value() == 0
            if pressed:
                reset_button_pressed_count += 1
            else:
                if reset_button_pressed_count > 0:
                    reset_button_pressed_count -= 1
            if reset_button_pressed_count > 7:
                logging.info('reset button pressed', 'main:main')
                ap_mode = not ap_mode
                config['ap_mode'] = ap_mode
                save_config(config)
                keep_running = False
        else:
            await asyncio.sleep(10.0)
    if upython:
        machine.soft_reset()


if __name__ == '__main__':
    logging.loglevel = logging.INFO
    # logging.loglevel = logging.DEBUG
    logging.info('starting', 'main:__main__')
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info('bye', 'main:__main__')
    finally:
        asyncio.new_event_loop()
    logging.info('done', 'main:__main__')
