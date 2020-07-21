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

APPLIANCE_TYPE_OFFSET = 1
MSG_ID_OFFSET = 5
MSG_TYPE_OFFSET = 8
MSG_BODY_OFFSET = 9


# message bits from http://chipsc.com/home/views/default/resource/images/111.pdf [1]
class DecoderState:
    IDLE, READ_MESSAGE_LENGTH, READ_APPLIANCE_TYPE, READ_FRAME_SYNC_CHECK, READ_KEEP, \
     READ_MESSAGE_ID, READ_FRAMEWORK_AGREEMENT_VERSION, READ_APPLIANCE_AGREEMENT_VERSION, \
     READ_MESSAGE_TYPE, READ_MESSAGE_BODY, READ_CHECKSUM = range(11)


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
        ('am-body', 'AM body'),  # 2
        ('am-checksum', 'AM checksum'),  # 3
        ('ma-sync', 'MA sync'),  # 4
        ('ma-header', 'MA header'),  # 5
        ('ma-body', 'MA body'),  # 6
        ('ma-checksum', 'MA checksum'),  # 7
        ('am-cmd', 'AM Cmd'),  # 8
        ('ma-cmd', 'MA Cmd'),  # 9
        ('error-indication', 'Error indication'),  # 10
    )
    bytes_annotations_stride = 4
    annotation_rows = (
        ('am', 'Appliance->Module', tuple(range(4))),
        ('ma', 'Module->Appliance', tuple(range(4, 8))),
        ('req-res', 'Request/Response', (8, 9)),
        ('error-indicators', 'Errors in frame', (10,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = [DecoderState.IDLE, DecoderState.IDLE]
        self.msg = [[], []]
        self.ss_msg_body_block = [None, None]
        self.data_lenght = [0, 0]
        self.data_read = [0, 0]

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        # ignore all UART packets except the actual data packets.
        if ptype != 'DATA':
            return

        if self.state[rxtx] != DecoderState.IDLE:
            self.msg[rxtx].append(pdata[0])
            self.data_read[rxtx] += 1

        if self.state[rxtx] == DecoderState.IDLE:
            if pdata[0] == 0xAA:  # 0xAA sync header
                self.msg[rxtx] = []
                self.data_read[rxtx] = 0
                self.ss_msg_body_block[rxtx] = None

                self.state[rxtx] = DecoderState.READ_MESSAGE_LENGTH

                self.put(ss, es, self.out_ann, [0 + (rxtx * Decoder.bytes_annotations_stride), ['Sync']])
        elif self.state[rxtx] == DecoderState.READ_MESSAGE_LENGTH:
            self.data_lenght[rxtx] = pdata[0]
            self.state[rxtx] = DecoderState.READ_APPLIANCE_TYPE

            self.put(ss, es, self.out_ann,
                     [1 + (rxtx * Decoder.bytes_annotations_stride), ['L: {:d}'.format(self.data_lenght[rxtx])]])
        elif self.state[rxtx] == DecoderState.READ_APPLIANCE_TYPE:
            self.state[rxtx] = DecoderState.READ_FRAME_SYNC_CHECK
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['At: {:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_FRAME_SYNC_CHECK:
            self.state[rxtx] = DecoderState.READ_KEEP
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['Fs: {:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_KEEP:
            if self.data_read[rxtx] == 5:
                self.state[rxtx] = DecoderState.READ_MESSAGE_ID
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['{:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_MESSAGE_ID:
            self.state[rxtx] = DecoderState.READ_FRAMEWORK_AGREEMENT_VERSION
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['Mid:{:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_FRAMEWORK_AGREEMENT_VERSION:  # according to [1] not used
            self.state[rxtx] = DecoderState.READ_APPLIANCE_AGREEMENT_VERSION
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['Fv:{:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_APPLIANCE_AGREEMENT_VERSION:  # according to [1] not used
            self.state[rxtx] = DecoderState.READ_MESSAGE_TYPE
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['Av:{:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_MESSAGE_TYPE:
            self.state[rxtx] = DecoderState.READ_MESSAGE_BODY
            self.put(ss, es, self.out_ann, [1 + (rxtx * Decoder.bytes_annotations_stride), ['Mt:{:02X}'.format(pdata[0])]])
        elif self.state[rxtx] == DecoderState.READ_MESSAGE_BODY:
            if self.ss_msg_body_block[rxtx] is None:
                self.ss_msg_body_block[rxtx] = ss

            if self.data_read[rxtx] == (self.data_lenght[rxtx] - 1):
                self.state[rxtx] = DecoderState.READ_CHECKSUM

                self.put(self.ss_msg_body_block[rxtx], es, self.out_ann,
                         [2 + (rxtx * Decoder.bytes_annotations_stride),
                          [' '.join(['{:02X}'.format(n) for n in self.msg[rxtx][MSG_BODY_OFFSET:]]), 'Data']])
                msg_type = self.msg[rxtx][MSG_TYPE_OFFSET]
                cmd_handler = getattr(self, 'cmd_handler_0x{:02X}'.format(msg_type), self.cmd_handler_unknown)
                cmd_handler(self.ss_msg_body_block[rxtx], es, (rxtx, self.msg[rxtx][APPLIANCE_TYPE_OFFSET], msg_type, self.msg[rxtx][MSG_BODY_OFFSET:]))
        elif self.state[rxtx] == DecoderState.READ_CHECKSUM:
            self.state[rxtx] = DecoderState.IDLE

            self.put(ss, es, self.out_ann, [3 + (rxtx * Decoder.bytes_annotations_stride), ['Csum: {:02X}'.format(pdata[0])]])
            if checksum(self.msg[rxtx]) != 0:
                self.put(ss, es, self.out_ann, [10, ['Checksum failed']])
            data_str = ''.join(['{:02X}'.format(n) for n in self.msg[rxtx][APPLIANCE_TYPE_OFFSET:-1]])
            print('RXTX[{}]: L={} D={}'.format(rxtx, self.data_lenght[rxtx], data_str))

    def cmd_handler_unknown(self, ss, es, data):
        rxtx, appliance_type, msg_type, msg_body = data

        self.put(ss, es, self.out_ann, [8 + rxtx, ['Unknown msg type 0x{:02X}'.format(msg_type)]])

    def cmd_handler_0x03(self, ss, es, data):
        rxtx, appliance_type, msg_type, msg_body = data

        crc_status = 'OK' if crc8(msg_body) == 0 else 'Error'

        self.put(ss, es, self.out_ann, [8 + rxtx, ['msg type 0x03 CRC {}'.format(crc_status)]])

    def cmd_handler_0x63(self, ss, es, data):
        rxtx, appliance_type, msg_type, msg_body = data

        ann = None
        if rxtx == 0:  # request from appliance
            ann = ['Query network status']
        else:
            module_type = 'RF' if msg_body[0] == 0 else 'Wi-Fi'
            module_mode = msg_body[1]
            wifi_signal_strength = msg_body[2]
            ip_address = msg_body[3:7]
            rf_signal_strength = msg_body[7]
            network_status = 'OK' if msg_body[8] == 0 else msg_body[8]
            cloud_status = 'OK' if msg_body[9] == 0 else msg_body[9]
            conn_status = msg_body[10]
            tcp_conn_count = msg_body[11]
            ann = ['{}({}), IP: {}.{}.{}.{}, Net Status: {}, Cloud status: {}'.format(module_type, module_mode, ip_address[3], ip_address[2], ip_address[1], ip_address[0], network_status, cloud_status)]
        self.put(ss, es, self.out_ann, [8 + rxtx, ann])
