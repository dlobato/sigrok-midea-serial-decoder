##
## Copyright (C) 2020 David Lobato <dav.lobato@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from math import ceil
from .util import crc8, checksum

RX = 0
TX = 1
rxtx_channels = ('RX', 'TX')

HEADER_LENGTH = 8  # sync byte not included
LENGTH_OFFSET = 0
APPLIANCE_TYPE_OFFSET = 1
MSG_ID_OFFSET = 5
FRAMEWORK_VERSION_OFFSET = 6
APPLIANCE_VERSION_OFFSET = 7
MSG_TYPE_OFFSET = 8
MSG_BODY_OFFSET = 9


# message bits from http://chipsc.com/home/views/default/resource/images/111.pdf [1]
class DecoderState:
    IDLE, READ_HEADER, READ_MESSAGE = range(3)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'midea-serial'
    name = 'Midea Serial'
    longname = 'Midea Serial protocol'
    desc = 'Midea Serial protocol decoder.'
    license = 'gplv3+'
    inputs = ['uart']
    outputs = []
    tags = ['Embedded/industrial']
    annotations = (
        ('am-sync', 'AM sync'),  # 0
        ('am-header', 'AM header'),  # 1
        ('am-msg', 'AM msg'),  # 2
        ('ma-sync', 'MA sync'),  # 3
        ('ma-header', 'MA header'),  # 4
        ('ma-msg', 'MA msg'),  # 5
        ('am-cmd', 'AM Cmd'),  # 6
        ('ma-cmd', 'MA Cmd'),  # 7
        ('error-indication', 'Error indication'),  # 8
    )
    bytes_annotations_stride = 3
    annotation_rows = (
        ('am', 'Appliance->Module', tuple(range(3))),
        ('ma', 'Module->Appliance', tuple(range(3, 6))),
        ('req-res', 'Request/Response', (6, 7)),
        ('error-indicators', 'Errors in frame', (8,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = [DecoderState.IDLE, DecoderState.IDLE]
        self.data = [[], []]
        self.ss_header_block = [None, None]
        self.ss_msg_block = [None, None]

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        # ignore all UART packets except the actual data packets.
        if ptype != 'DATA':
            return

        if self.state[rxtx] != DecoderState.IDLE:
            self.data[rxtx].append(pdata[0])

        if self.state[rxtx] == DecoderState.IDLE:
            if pdata[0] == 0xAA:  # 0xAA sync header
                self.data[rxtx] = []
                self.ss_header_block[rxtx] = None
                self.ss_msg_block[rxtx] = None

                self.state[rxtx] = DecoderState.READ_HEADER

                self.put(ss, es, self.out_ann, [0 + (rxtx * Decoder.bytes_annotations_stride), ['Sync']])
        elif self.state[rxtx] == DecoderState.READ_HEADER:
            if self.ss_header_block[rxtx] is None:
                self.ss_header_block[rxtx] = ss

            if len(self.data[rxtx]) == HEADER_LENGTH:
                self.state[rxtx] = DecoderState.READ_MESSAGE
                self.put(self.ss_header_block[rxtx], es, self.out_ann,
                         [1 + (rxtx * Decoder.bytes_annotations_stride), [
                             'L: {:d}, At: {:02X}, Mid:{:02X}, v:{:02X}{:02X}'.format(self.data[rxtx][LENGTH_OFFSET],
                                                                                      self.data[rxtx][APPLIANCE_TYPE_OFFSET],
                                                                                      self.data[rxtx][MSG_ID_OFFSET],
                                                                                      self.data[rxtx][FRAMEWORK_VERSION_OFFSET],
                                                                                      self.data[rxtx][APPLIANCE_VERSION_OFFSET])]])
        elif self.state[rxtx] == DecoderState.READ_MESSAGE:
            if self.ss_msg_block[rxtx] is None:
                self.ss_msg_block[rxtx] = ss

            if len(self.data[rxtx]) == self.data[rxtx][LENGTH_OFFSET]:
                self.state[rxtx] = DecoderState.IDLE
                msg_body = ''.join(['{:02X}'.format(n) for n in self.data[rxtx][MSG_BODY_OFFSET:-1]])
                self.put(self.ss_msg_block[rxtx], es, self.out_ann,
                         [2 + (rxtx * Decoder.bytes_annotations_stride),
                          ['Type: {:02X}, {}, Checksum: {:02X}'.format(self.data[rxtx][MSG_TYPE_OFFSET], msg_body,
                                                                       self.data[rxtx][-1])]])
                if checksum(self.data[rxtx][MSG_BODY_OFFSET:]) != 0:
                    msg_type = self.data[rxtx][MSG_TYPE_OFFSET]
                    cmd_handler = getattr(self, 'cmd_handler_0x{:02X}'.format(msg_type), self.cmd_handler_default)
                    cmd_handler(self.ss_msg_block[rxtx], es, (rxtx, self.data[rxtx]))
                else:
                    self.put(ss, es, self.out_ann, [8, ['Checksum failed']])

                data_str = ''.join(['{:02X}'.format(n) for n in self.data[rxtx][APPLIANCE_TYPE_OFFSET:-1]])
                print('RXTX[{}]: L={} D={}'.format(rxtx, self.data[rxtx][LENGTH_OFFSET], data_str))

    def cmd_handler_default(self, ss, es, data):
        rxtx, read_data = data

        self.put(ss, es, self.out_ann, [6 + rxtx, ['msg type 0x{:02X}'.format(read_data[MSG_TYPE_OFFSET])]])

    def cmd_handler_0x63(self, ss, es, data):
        rxtx, read_data = data

        if rxtx == 0:  # request from appliance
            self.put(ss, es, self.out_ann, [6 + rxtx, ['Query network status']])
        else:
            msg_body = read_data[MSG_BODY_OFFSET:]
            module_type = 'RF' if msg_body[0] == 0 else 'Wi-Fi'
            module_mode = msg_body[1]
            wifi_signal_strength = msg_body[2]
            ip_address = msg_body[3:7]
            rf_signal_strength = msg_body[7]
            network_status = 'OK' if msg_body[8] == 0 else msg_body[8]
            cloud_status = 'OK' if msg_body[9] == 0 else msg_body[9]
            conn_status = msg_body[10]
            tcp_conn_count = msg_body[11]
            self.put(ss, es, self.out_ann, [6 + rxtx, [
                '{}({}), IP: {}.{}.{}.{}, Net Status: {}, Cloud status: {}'.format(module_type, module_mode,
                                                                                   ip_address[3], ip_address[2],
                                                                                   ip_address[1], ip_address[0],
                                                                                   network_status, cloud_status)]])
