#!/usr/bin/env python
'''
unpack a param.pck file from @PARAM/param.pck via mavlink FTP
'''

import struct

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("file", metavar="LOG")

args = parser.parse_args()

data = open(args.file).read()
ofs = 0
last_name = ""

def pad_data(vdata, vlen):
    while len(vdata) < vlen:
        vdata += chr(0)
    return vdata

def decode_value(ptype, vdata):
    if ptype == 1:
        vdata = pad_data(vdata, 1)
        return struct.unpack("<b", vdata)[0]
    if ptype == 2:
        vdata = pad_data(vdata, 2)
        return struct.unpack("<h", vdata)[0]
    if ptype == 3:
        vdata = pad_data(vdata, 4)
        return struct.unpack("<i", vdata)[0]
    if ptype == 4:
        vdata = pad_data(vdata, 4)
        return struct.unpack("<f", vdata)[0]
    print("bad ptype %u" % ptype)
    return 0


while True:
    while len(data) > 0 and ord(data[0]) == 0:
        data = data[1:]
    if len(data) == 0:
        break
    ptype, plen = struct.unpack("<BB", data[0:2])
    type_len = (ptype>>4) & 0x0F
    ptype &= 0x0F
    name_len = ((plen>>4) & 0x0F) + 1
    common_len = (plen & 0x0F)
    name = last_name[0:common_len] + data[2:2+name_len]
    vdata = data[2+name_len:2+name_len+type_len]
    last_name = name
    data = data[2+name_len+type_len:]
    v = decode_value(ptype, vdata)
    print("%-16s %f" % (name, float(v)))
