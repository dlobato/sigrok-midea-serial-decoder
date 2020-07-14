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
from .crc8 import calculate

RX = 0
TX = 1
rxtx_channels = ('RX', 'TX')

DATA_OFFSET = 3
CRC8_DATA_OFFSET = 10
CHECKSUM_DATA_OFFSET = 1


class DecoderState:
    IDLE, READ_LENGTH, READ_DEVICE_TYPE, READ_DATA, READ_CRC8, READ_CHECKSUM, ERROR = range(7)


def checksum(data):
    return (~ sum(data) + 1) & 0xff


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
        ('uc-start', 'UC start'),
        ('uc-length', 'UC length'),
        ('uc-device-type', 'UC device type'),
        ('uc-data', 'UC data'),
        ('uc-crc8', 'UC crc8'),
        ('uc-checksum', 'UC checksum'),
        ('cu-start', 'CU start'),
        ('cu-length', 'CU length'),
        ('cu-device-type', 'CU device type'),
        ('cu-data', 'CU data'),
        ('cu-crc8', 'CU crc8'),
        ('cu-checksum', 'CU checksum'),
        ('error-indication', 'Error indication'),
    )
    annotation_rows = (
        ('uc', 'Unit->controller', (0, 1, 2, 3, 4, 5)),
        ('cu', 'Controller->unit', (6, 7, 8, 9, 10, 11)),
        ('error-indicators', 'Errors in frame', (12,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = [DecoderState.IDLE, DecoderState.IDLE]
        self.cmd = [[], []]
        self.ss_data_block = [None, None]
        self.data_lenght = [0, 0]
        self.data_read = [0, 0]

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        # For now, ignore all UART packets except the actual data packets.
        if ptype != 'DATA':
            return

        if self.state[rxtx] == DecoderState.IDLE:
            if pdata[0] == 0xAA:  # 0xAA marks the start of a message
                self.cmd[rxtx] = [0xAA]
                self.ss_data_block[rxtx] = None
                self.state[rxtx] = DecoderState.READ_LENGTH
                self.put(ss, es, self.out_ann, [0 + (rxtx * 6), ['Start', 'ST', 'S']])
        elif self.state[rxtx] == DecoderState.READ_LENGTH:
            self.cmd[rxtx].append(pdata[0])
            self.data_lenght[rxtx] = pdata[0]
            self.data_read[rxtx] = 1  # length includes length byte
            self.state[rxtx] = DecoderState.READ_DEVICE_TYPE
            self.put(ss, es, self.out_ann,
                     [1 + (rxtx * 6), ['Length: {:d}'.format(self.data_lenght[rxtx]),
                                       'L{:d}'.format(self.data_lenght[rxtx]), 'L']])
        elif self.state[rxtx] == DecoderState.READ_DEVICE_TYPE:
            self.cmd[rxtx].append(pdata[0])
            self.data_read[rxtx] += 1
            self.state[rxtx] = DecoderState.READ_DATA
            self.put(ss, es, self.out_ann, [2 + (rxtx * 6), ['Type: {:02X}'.format(pdata[0]),
                                                             'T{:02X}'.format(pdata[0]), 'T']])
        elif self.state[rxtx] == DecoderState.READ_DATA:
            self.cmd[rxtx].append(pdata[0])
            if self.ss_data_block[rxtx] is None:
                self.ss_data_block[rxtx] = ss
            self.data_read[rxtx] += 1
            if self.data_read[rxtx] == (self.data_lenght[rxtx] - 2):  # data without crc8 and checksum bytes
                self.state[rxtx] = DecoderState.READ_CRC8
                self.put(self.ss_data_block[rxtx], es, self.out_ann,
                         [3 + (rxtx * 6),
                          [' '.join(['{:02X}'.format(n) for n in self.cmd[rxtx][DATA_OFFSET:]]), 'Data']])
        elif self.state[rxtx] == DecoderState.READ_CRC8:
            self.cmd[rxtx].append(pdata[0])
            self.data_read[rxtx] += 1
            self.state[rxtx] = DecoderState.READ_CHECKSUM
            crc8_result = calculate(self.cmd[rxtx][CRC8_DATA_OFFSET:-1])
            self.put(ss, es, self.out_ann, [4 + (rxtx * 6), ['CRC8: {:02X}'.format(crc8_result), 'CRC8']])
            if crc8_result != pdata[0]:
                self.put(ss, es, self.out_ann, [12, ['CRC8 failed']])
        elif self.state[rxtx] == DecoderState.READ_CHECKSUM:
            self.cmd[rxtx].append(pdata[0])
            self.data_read[rxtx] += 1
            self.state[rxtx] = DecoderState.IDLE
            checksum_result = checksum(self.cmd[rxtx][CHECKSUM_DATA_OFFSET:-1])
            self.put(ss, es, self.out_ann, [5 + (rxtx * 6), ['Checksum: {:02X}'.format(checksum_result), 'CKS']])
            if checksum_result != pdata[0]:
                self.put(ss, es, self.out_ann, [12, ['Checksum failed']])
            data_str = ''.join(['{:02X}'.format(n) for n in self.cmd[rxtx]])
            print('RXTX[{}]: L={} D={}'.format(rxtx, self.data_lenght[rxtx], data_str))
