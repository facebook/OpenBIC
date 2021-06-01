#!/usr/bin/env python3
import sys


def insert_bytearray(src, dst, offset):
    if offset+src.__len__() > dst.__len__():
        dst.extend(bytearray(offset-dst.__len__()+src.__len__()))

    dst[offset:offset+src.__len__()] = src


image_path = sys.argv[1]
otp_conf_path = sys.argv[2]
otp_data_path = sys.argv[3]

with open(image_path, 'rb') as fd:
    image_bin = bytearray(fd.read())

with open(otp_conf_path, 'rb') as fd:
    otp_conf_bin = bytearray(fd.read())
    otp_conf_bin.extend(bytearray(16))

with open(otp_data_path, 'rb') as fd:
    otp_data_bin = bytearray(fd.read())

insert_bytearray(otp_conf_bin, image_bin, 0x100000)
insert_bytearray(otp_data_bin, image_bin, 0x100080)


with open(image_path, 'w+b') as fd:
    fd.write(image_bin)