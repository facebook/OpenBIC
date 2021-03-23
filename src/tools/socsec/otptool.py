#!/usr/bin/env python3

import argparse
import sys
import array
import jstyleson
import struct
import os
from bitarray import bitarray
from jsonschema import validate
from Crypto.Hash import SHA256
from socsec import parse_path
from socsec import insert_bytearray
from socsec import rsa_bit_length
from socsec import rsa_key_to_bin
from socsec import OTP_info


class OtpError(Exception):
    """Application-specific errors.

    These errors represent issues for which a stack-trace should not be
    presented.

    Attributes:
        message: Error message.
    """

    def __init__(self, message):
        Exception.__init__(self, message)


class ECC (object):
    def byte2bits(self, data_bytearray: bytearray) -> array:
        bits = ''
        bits_array = []
        for data_byte in data_bytearray:
            data_bits = bin(data_byte)[2:].zfill(8)[::-1]
            bits += data_bits
        for binary in bits:
            bits_array.append(int(binary, 2))
        return bits_array

    def xor_array(self, bits_array: array, x: int, y: int) -> int:
        out = 0
        if x < y:
            for i in range(x, y + 1):
                out ^= bits_array[i]
        elif x > y:
            for i in range(y, x + 1):
                out ^= bits_array[i]
        else:
            return bits_array[x]
        return out

    def bitsarray_to_bytes(self, s: array) -> int:
        r = 0
        offset = 0
        for bits in s:
            r += bits << offset
            offset += 1
        return r

    def do_ecc(self, data: bytearray) -> bytearray:
        result = bytearray(1024)
        offset = 0
        for cursor in range(0, data.__len__(), 8):
            eight_bytes = data[cursor:cursor+8]
            bits = self.byte2bits(eight_bytes)
            tmp = [0, 0, 0, 0, 0, 0, 0, 0]
            tmp[0] = bits[0] ^ bits[1] ^ bits[3] ^ bits[4] ^ bits[6] ^ \
                bits[8] ^ bits[10] ^ bits[11] ^ bits[13] ^ bits[15] ^ \
                bits[17] ^ bits[19] ^ bits[21] ^ bits[23] ^ bits[25] ^ \
                bits[26] ^ bits[28] ^ bits[30] ^ bits[32] ^ bits[34] ^ \
                bits[36] ^ bits[38] ^ bits[40] ^ bits[42] ^ bits[44] ^ \
                bits[46] ^ bits[48] ^ bits[50] ^ bits[52] ^ bits[54] ^ \
                bits[56] ^ bits[57] ^ bits[59] ^ bits[61] ^ bits[63]
            tmp[1] = bits[0] ^ bits[2] ^ bits[3] ^ bits[5] ^ bits[6] ^ \
                bits[9] ^ bits[10] ^ bits[12] ^ bits[13] ^ bits[16] ^ \
                bits[17] ^ bits[20] ^ bits[21] ^ bits[24] ^ bits[25] ^ \
                bits[27] ^ bits[28] ^ bits[31] ^ bits[32] ^ bits[35] ^ \
                bits[36] ^ bits[39] ^ bits[40] ^ bits[43] ^ bits[44] ^ \
                bits[47] ^ bits[48] ^ bits[51] ^ bits[52] ^ bits[55] ^ \
                bits[56] ^ bits[58] ^ bits[59] ^ bits[62] ^ bits[63]
            tmp[2] = self.xor_array(bits, 3, 1) ^ self.xor_array(bits, 10, 7) ^ \
                self.xor_array(bits, 17, 14) ^ self.xor_array(bits, 25, 22) ^ \
                self.xor_array(bits, 32, 29) ^ self.xor_array(bits, 40, 37) ^ \
                self.xor_array(bits, 48, 45) ^ self.xor_array(bits, 56, 53) ^ \
                self.xor_array(bits, 63, 60)
            tmp[3] = self.xor_array(bits, 10, 4) ^ self.xor_array(bits, 25, 18) ^ \
                self.xor_array(bits, 40, 33) ^ self.xor_array(bits, 56, 49)
            tmp[4] = self.xor_array(
                bits, 25, 11) ^ self.xor_array(bits, 56, 41)
            tmp[5] = self.xor_array(bits, 56, 26)
            tmp[6] = self.xor_array(bits, 63, 57)
            tmp[7] = self.xor_array(bits, 63, 0)
            result[offset] = self.bitsarray_to_bytes(tmp)
            offset += 1
        return result


def load_file(file_path: str):
    with open(file_path, 'rb') as f:
        file_bin = f.read()
        f.close()
    return file_bin


def dw_hex_to_bin(dw_hex_bytes: bytes):
    dw_hex_bytearray = bytearray(dw_hex_bytes.replace(b'\n', b''))
    if dw_hex_bytearray.__len__() % 8 != 0:
        raise ValueError("input string is not dw aligned")

    bin_array = bytearray()
    for i in range(0, dw_hex_bytearray.__len__(), 8):
        dw_string = dw_hex_bytearray[i:i+8]
        dw_bytes = bytearray.fromhex(dw_string.decode())
        dw_bytes.reverse()
        bin_array += dw_bytes
    return bin_array


def writeBinFile(in_bin: bytearray, dst_path: str):
    with open(dst_path, 'w+b')as outf:
        outf.write(bytes(in_bin))
        outf.close()


def writeHexFile(in_bin: bytearray, dst_path: str):
    output = ''
    for c in in_bin:
        output += "{0:#0{1}x}".format(c, 4)[2:]
        output += '\n'
    with open(dst_path, 'w+b')as outf:
        outf.write(output.encode())
        outf.close()


def writeDWHexFile(in_bin: bytearray, dst_path: str):
    output = ''
    sub_output = ''
    bc = 0
    for c in bytes(in_bin):
        sub_output = "{0:#0{1}x}".format(c, 4)[2:] + sub_output
        bc += 1
        if bc == 4:
            output += sub_output + '\n'
            sub_output = ''
            bc = 0
    with open(dst_path, 'w+b')as outf:
        outf.write(output.encode())
        outf.close()


class OTP(object):
    """otptool command-line tool."""
    otp_info = OTP_info

    def OTPValidate(self, otp_config, otp_info):
        schema = {
            "type": "object",
            "required": [
                "name",
                "version"
            ],
            "additionalProperties": False,
            "properties": {
                "name": {
                    "type": "string"
                },
                "version": {
                    "type": "string",
                    "enum": [
                        "A0",
                        "A1",
                        "A2",
                        "1030A0"
                    ]
                },
                "data_region": {
                    "type": "object",
                    "additionalProperties": False,
                    "required": [
                        "ecc_region"
                    ],
                    "properties": {
                        "patch": {
                            "type": "boolean"
                        },
                        "ecc_region": {
                            "type": "boolean"
                        },
                        "key": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "additionalProperties": False,
                                "properties": {
                                    "types": {
                                        "type": "string",
                                        "enum": [
                                            "aes_oem",
                                            "aes_vault",
                                            "rsa_pub_oem",
                                            "rsa_pub_aes",
                                            "rsa_priv_aes"
                                        ]
                                    },
                                    "key_bin": {
                                        "type": "string"
                                    },
                                    "key_bin2": {
                                        "type": "string"
                                    },
                                    "iv_bin": {
                                        "type": "string"
                                    },
                                    "key_pem": {
                                        "type": "string"
                                    },
                                    "rsa_pem": {
                                        "type": "string"
                                    },
                                    "number_id": {
                                        "type": "integer",
                                        "minimum": 0,
                                        "maximum": 7
                                    },
                                    "sha_mode": {
                                        "type": "string",
                                        "enum": [
                                            "SHA224",
                                            "SHA256",
                                            "SHA384",
                                            "SHA512"
                                        ]
                                    },
                                    "offset": {
                                        "type": "string",
                                        "pattern": "0[xX][0-9a-fA-F]+"
                                    },
                                    "key_length": {
                                        "type": "integer"
                                    }
                                }
                            }
                        },
                        "user_data": {
                            "type": "array",
                            "additionalProperties": False,
                            "items": {
                                "type": "object",
                                "additionalProperties": False,
                                "properties": {
                                    "types": {
                                        "type": "string",
                                        "enum": [
                                            "hex",
                                            "dw_hex",
                                            "bin"
                                        ]
                                    },
                                    "file": {
                                        "type": "string"
                                    },
                                    "offset": {
                                        "type": "string",
                                        "pattern": "0[xX][0-9a-fA-F]+"
                                    }
                                }
                            }
                        }
                    }
                },
                "config_region": {
                },
                "otp_strap": {},
            }
        }
        config_schema = {}
        strap_schema = {}

        with open(otp_info['config'], 'r') as config_info_fd:
            config_info = jstyleson.load(config_info_fd)

        with open(otp_info['strap'], 'r') as strap_info_fd:
            strap_info = jstyleson.load(strap_info_fd)

        for i in config_info:
            if i['type'] == 'boolean':
                config_schema[i['key']] = {
                    'type': 'boolean'
                }
            elif i['type'] == 'string':
                config_schema[i['key']] = {
                    'type': 'string',
                    'enum': []
                }
                for j in i['value']:
                    config_schema[i['key']]['enum'].append(j['value'])
            elif i['type'] == 'bit_shift':
                config_schema[i['key']] = {
                    'type': 'integer',
                    'minimum': 0,
                    'maximum': i['bit_length'] - 1
                }
            elif i['type'] == 'hex':
                config_schema[i['key']] = {
                    'type': 'string',
                    'pattern': '0[xX][0-9a-fA-F]+'
                }

        for i in strap_info:
            val = {}
            if i['type'] == 'boolean':
                val = {
                    'type': 'boolean'
                }
            elif i['type'] == 'string':
                val = {
                    'type': 'string',
                    'enum': []
                }
                for j in i['value']:
                    val['enum'].append(j['value'])

            strap_schema[i['key']] = {
                "type": "object",
                "additionalProperties": False,
                "required": [
                    "value"
                ],
                "properties": {
                    "otp_protect": {
                        "type": "boolean"
                    },
                    "reg_protect": {
                        "type": "boolean"
                    },
                    "ignore": {
                        "type": "boolean"
                    },
                    "value": val
                }
            }

        schema['properties']['config_region'] = {
            "type": "object",
            "additionalProperties": False,
            "properties": config_schema
        }
        schema['properties']['strap_region'] = {
            "type": "object",
            "additionalProperties": False,
            "properties": strap_schema
        }
        validate(otp_config, schema)

    def genKeyHeader_a0(self, key_config, key_folder):
        types = key_config['types']
        offset = int(key_config['offset'], 16)
        header = 0
        header |= offset

        if types == 'aes_oem':
            header |= 0
        elif types == 'aes_vault':
            header |= 1 << 14
        elif types == 'rsa_pub_oem':
            header |= 8 << 14
        elif types == 'rsa_pub_aes':
            header |= 10 << 14
        elif types == 'rsa_priv_aes':
            header |= 14 << 14

        if 'number_id' in key_config:
            number_id = key_config['number_id']
            header |= number_id

        if types in ['rsa_pub_oem', 'rsa_pub_soc', 'rsa_pub_aes', 'rsa_priv_soc', 'rsa_priv_aes']:
            rsa_key_file = key_folder + key_config['key_pem']
            mod_length = rsa_bit_length(rsa_key_file, 'n')
            if mod_length == 1024:
                header |= 0 << 18
            elif mod_length == 2048:
                header |= 1 << 18
            elif mod_length == 3072:
                header |= 2 << 18
            elif mod_length == 4096:
                header |= 3 << 18
            else:
                raise ValueError("key_length is not supported")

            if types in ['rsa_pub_oem', 'rsa_pub_soc', 'rsa_pub_aes']:
                exp_length = rsa_bit_length(rsa_key_file, 'e')
            else:
                exp_length = rsa_bit_length(rsa_key_file, 'd')
            header |= exp_length << 20
        return header

    def genKeyHeader_a1(self, key_config, key_folder):
        types = key_config['types']
        offset = int(key_config['offset'], 16)
        header = 0
        header |= offset

        if types == 'aes_vault':
            header |= 1 << 14
        elif types == 'aes_oem':
            header |= 2 << 14
        elif types == 'rsa_pub_oem':
            header |= 8 << 14
        elif types == 'rsa_pub_aes':
            header |= 10 << 14
        elif types == 'rsa_priv_aes':
            header |= 14 << 14

        if 'number_id' in key_config:
            number_id = key_config['number_id']
            header |= number_id

        if types in ['rsa_pub_oem', 'rsa_pub_aes', 'rsa_priv_aes']:
            rsa_key_file = key_folder + key_config['key_pem']
            mod_length = rsa_bit_length(rsa_key_file, 'n')
            if mod_length == 1024:
                header |= 0 << 18
            elif mod_length == 2048:
                header |= 1 << 18
            elif mod_length == 3072:
                header |= 2 << 18
            elif mod_length == 4096:
                header |= 3 << 18
            else:
                raise ValueError("key_length is not supported")

            if types in ['rsa_pub_oem', 'rsa_pub_aes']:
                exp_length = rsa_bit_length(rsa_key_file, 'e')
            else:
                exp_length = rsa_bit_length(rsa_key_file, 'd')
            header |= exp_length << 20
        return header

    def key_to_bytearray_a0(self, key_config, key_folder):
        types = key_config['types']

        if types in ['rsa_pub_oem', 'rsa_pub_soc', 'rsa_pub_aes', 'rsa_priv_soc', 'rsa_priv_aes']:
            rsa_key_file = key_folder + key_config['key_pem']
            if types in ['rsa_pub_oem', 'rsa_pub_soc', 'rsa_pub_aes']:
                insert_key_bin = rsa_key_to_bin(rsa_key_file, 'public')
            else:
                insert_key_bin = rsa_key_to_bin(rsa_key_file, 'private')
        else:
            aes_key_bin = load_file(key_folder + key_config['key_bin'])
            aes_iv_bin = load_file(key_folder + key_config['iv_bin'])
            insert_key_bin = bytearray(aes_key_bin)
            insert_bytearray(bytearray(aes_iv_bin), insert_key_bin, 0x20)

        return insert_key_bin

    def key_to_bytearray_a1(self, key_config, key_folder):
        types = key_config['types']

        if types in ['rsa_pub_oem', 'rsa_pub_aes', 'rsa_priv_aes']:
            rsa_key_file = key_folder + key_config['key_pem']
            if types in ['rsa_pub_oem', 'rsa_pub_aes']:
                insert_key_bin = rsa_key_to_bin(rsa_key_file, 'public')
            else:
                insert_key_bin = rsa_key_to_bin(rsa_key_file, 'private')
        elif types in ['aes_vault']:
            aes_key_bin = load_file(key_folder + key_config['key_bin'])
            aes_key_bin2 = load_file(key_folder + key_config['key_bin2'])
            insert_key_bin = bytearray(aes_key_bin)
            insert_bytearray(bytearray(aes_key_bin2), insert_key_bin, 0x20)
        else:
            insert_key_bin = load_file(key_folder + key_config['key_bin'])

        return insert_key_bin

    def file_to_bytearray(self, data_config, user_data_folder):
        types = data_config['types']

        with open(user_data_folder + data_config['file'], 'rb') as data_bin_file:
            file_bin = data_bin_file.read()
            data_bin_file.close()

        if types == 'bin':
            return bytearray(file_bin)

        if types == 'dw_hex':
            return dw_hex_to_bin(file_bin)

        return None

    def genDataMask(self, data_region_ignore, src, offset, ecc_region_enable, data_region_size, ecc_region_offset):
        if ecc_region_enable:
            if (offset + src.__len__() >= ecc_region_offset):
                raise OtpError("Data region is out off range")
            start = int(offset / 8)
            end = int((offset + src.__len__() - 1) / 8)
            for i in range(start, end + 1):
                data_region_ignore[ecc_region_offset+i] = 0
        else:
            if (offset + src.__len__() >= data_region_size):
                raise OtpError("Data region is out off range")

        start = offset
        end = offset + src.__len__()
        for i in range(start, end):
            if data_region_ignore[i] == 0:
                raise OtpError("Data region is overlapping")

            data_region_ignore[i] = 0

    def genUserRegion(self, data_config, user_data_folder, data_region, data_region_ignore, ecc_region_enable, data_region_size, ecc_region_offset):
        for conf in data_config:
            offset = int(conf['offset'], 16)

            data_bin = self.file_to_bytearray(conf, user_data_folder)
            insert_bytearray(data_bin, data_region, offset)
            self.genDataMask(data_region_ignore, data_bin,
                             offset, ecc_region_enable, data_region_size, ecc_region_offset)

    def genKeyRegion(self, key_config, key_folder, data_region, data_region_ignore,
                     genKeyHeader, key_to_bytearray, ecc_region_enable, data_region_size, ecc_region_offset):

        key_header = []
        for conf in key_config:
            key_header.append(genKeyHeader(conf, key_folder))
        key_header[-1] |= 1 << 13
        header_byteArray = bytearray(array.array('I', key_header).tobytes())
        insert_bytearray(header_byteArray, data_region, 0)

        self.genDataMask(data_region_ignore, header_byteArray, 0,
                         ecc_region_enable, data_region_size, ecc_region_offset)

        for conf in key_config:
            offset = int(conf['offset'], 16)
            key_bin = key_to_bytearray(conf, key_folder)
            insert_bytearray(key_bin, data_region, offset)
            self.genDataMask(data_region_ignore, key_bin,
                             offset, ecc_region_enable, data_region_size, ecc_region_offset)

    def make_data_region(self, data_config, key_folder, user_data_folder, genKeyHeader, key_to_bytearray, data_region_size, ecc_region_offset):
        patch_reserved_offset = 0x1B80
        patch_reserved_lne = 0x80

        data_region = bytearray(data_region_size)
        data_region_ignore = bytearray(data_region_size)
        for i in range(0, data_region_size):
            data_region_ignore[i] = 0xff

        if data_config['ecc_region']:
            ecc_region_enable = True
        else:
            ecc_region_enable = False

        if 'key' in data_config:
            self.genKeyRegion(data_config['key'], key_folder,
                              data_region, data_region_ignore,
                              genKeyHeader, key_to_bytearray, ecc_region_enable, data_region_size, ecc_region_offset)
        if 'user_data' in data_config:
            self.genUserRegion(
                data_config['user_data'], user_data_folder, data_region, data_region_ignore, ecc_region_enable, data_region_size, ecc_region_offset)
        if 'patch' not in data_config:
            for i in data_region_ignore[patch_reserved_offset:patch_reserved_offset+patch_reserved_lne]:
                if i != 0xff:
                    raise OtpError('region {0:#08x} to {1:#08x} is reserved for patch'.format(
                        patch_reserved_offset, patch_reserved_offset+patch_reserved_lne))

        if ecc_region_enable:
            ecc_byteArray = ECC().do_ecc(data_region)
            insert_bytearray(ecc_byteArray, data_region, ecc_region_offset)

        return data_region, data_region_ignore

    def make_config_region(self, config_region_config, config_info, config_region_size):
        config_region = bitarray(config_region_size*8, endian='little')
        config_region_ignore = bitarray(config_region_size*8, endian='little')

        config_region.setall(False)
        config_region_ignore.setall(True)

        for i in config_info:
            if 'default' in i and i['type'] == 'boolean':
                dw_offset = i['dw_offset']
                bit_offset = i['bit_offset']
                offset = dw_offset*4+bit_offset
                if i['default']:
                    config_region[offset] = 1
                    config_region_ignore[offset] = 0

        for config in config_region_config:
            info = None
            key = config
            value = config_region_config[config]
            for i in config_info:
                if key == i['key']:
                    info = i
                    break
            if not info:
                raise OtpError('"{}" config is not supported'.format(key))

            if info['type'] == 'boolean':
                dw_offset = info['dw_offset']
                bit_offset = info['bit_offset']
                offset = dw_offset*32+bit_offset
                if value:
                    config_region[offset] = 1
                config_region_ignore[offset] = 0
            elif info['type'] == 'string':
                info_value = info['value']
                dw_offset = info['dw_offset']
                bit_offset = info['bit_offset']
                bit_length = info['bit_length']
                offset = dw_offset*32+bit_offset

                for t in info_value:
                    if t['value'] == value:
                        bit = t['bit']
                bit_value = bitarray(bin(bit)[2:][::-1])
                tmp = bitarray(bit_length)
                tmp.setall(False)
                config_region_ignore[offset:offset+bit_length] = tmp
                config_region[offset:offset+bit_length] = tmp
                config_region[offset:offset+bit_value.__len__()] = bit_value
            elif info['type'] == 'hex':
                dw_offset = info['dw_offset']
                bit_offset = info['bit_offset']
                bit_length = info['bit_length']
                hex_value = int(value, 16)
                offset = dw_offset*32+bit_offset

                if hex_value > 2 ** bit_length:
                    raise OtpError(
                        '"{}": config value out of range'.format(key))
                bit_value = bitarray(bin(hex_value)[2:][::-1])
                tmp = bitarray(bit_length)
                tmp.setall(False)
                config_region_ignore[offset:offset+bit_length] = tmp
                config_region[offset:offset+bit_length] = tmp
                config_region[offset:offset+bit_value.__len__()] = bit_value

            elif info['type'] == 'bit_shift':
                dw_offset = info['dw_offset']
                bit_offset = info['bit_offset']
                offset = dw_offset*32 + bit_offset
                bit_length = info['bit_length']
                value_start = info['value_start']
                offset_value = value - value_start

                if offset_value < 0 or offset_value > bit_length:
                    raise OtpError('"{}": value is out of range'.format(key))

                config_region_ignore[offset+offset_value] = 0
                config_region[offset+offset_value] = 1

            else:
                raise OtpError('"{}": value is invalid'.format(key))

        return bytearray(config_region.tobytes()), \
            bytearray(config_region_ignore.tobytes())

    def make_otp_strap(self, otp_strap_config, strap_info, otp_strap_bit_size):
        otp_strap = bitarray(otp_strap_bit_size, endian='little')
        otp_strap_reg_protect = bitarray(otp_strap_bit_size, endian='little')
        otp_strap_protect = bitarray(otp_strap_bit_size, endian='little')
        otp_strap_ignore = bitarray(otp_strap_bit_size, endian='little')

        otp_strap.setall(False)
        otp_strap_reg_protect.setall(False)
        otp_strap_protect.setall(False)
        otp_strap_ignore.setall(True)

        for config in otp_strap_config:
            info = None
            key = config
            value = otp_strap_config[config]['value']

            for i in strap_info:
                if key == i['key']:
                    info = i
                    break
            if not info:
                raise OtpError('"{}" strap is not supported'.format(key))
            bit_offset = info['bit_offset']
            if info['type'] == 'boolean':
                bit_length = 1
                if value:
                    otp_strap[bit_offset] = 1
            elif info['type'] == 'string':
                info_value = info['value']
                bit_length = info['bit_length']

                for t in info_value:
                    if t['value'] == str(value):
                        bit = t['bit']
                        break

                tmp = bitarray(bit_length)
                tmp.setall(False)
                otp_strap[bit_offset:bit_offset+bit_length] = tmp
                otp_strap[bit_offset:bit_offset+bit_length] = \
                    bitarray(bin(bit)[2:][::-1])

            tmp = bitarray(bit_length)
            tmp.setall(False)
            otp_strap_ignore[bit_offset:bit_offset+bit_length] = tmp
            if 'reg_protect' in otp_strap_config[config]:
                if otp_strap_config[config]['reg_protect']:
                    tmp.setall(True)
                    otp_strap_reg_protect[bit_offset:bit_offset+bit_length] = \
                        tmp

            if 'otp_protect' in otp_strap_config[config]:
                if otp_strap_config[config]['otp_protect']:
                    tmp.setall(True)
                    otp_strap_protect[bit_offset:bit_offset+bit_length] = \
                        tmp
            if 'ignore' in otp_strap_config[config]:
                if otp_strap_config[config]['ignore']:
                    tmp.setall(True)
                    otp_strap_ignore[bit_offset:bit_offset+bit_length] = \
                        tmp

        return bytearray(otp_strap.tobytes()), \
            bytearray(otp_strap_reg_protect.tobytes()), \
            bytearray(otp_strap_protect.tobytes()), \
            bytearray(otp_strap_ignore.tobytes())

    def make_otp_image(self, config_file, key_folder,
                       user_data_folder, output_folder):
        otp_config = jstyleson.load(config_file)

        if otp_config['version'] == 'A0':
            otp_info = self.otp_info.OTP_INFO['A0']
            version = 'A0'
            genKeyHeader = self.genKeyHeader_a0
            key_to_bytearray = self.key_to_bytearray_a0
        elif otp_config['version'] == 'A1':
            otp_info = self.otp_info.OTP_INFO['A1']
            version = 'A1'
            genKeyHeader = self.genKeyHeader_a1
            key_to_bytearray = self.key_to_bytearray_a1
        elif otp_config['version'] == 'A2':
            otp_info = self.otp_info.OTP_INFO['A2']
            version = 'A2'
            genKeyHeader = self.genKeyHeader_a1
            key_to_bytearray = self.key_to_bytearray_a1
        elif otp_config['version'] == '1030A0':
            otp_info = self.otp_info.OTP_INFO['A2']
            version = '1030A0'
            genKeyHeader = self.genKeyHeader_a1
            key_to_bytearray = self.key_to_bytearray_a1
        else:
            raise OtpError('version is invalid')

        self.OTPValidate(otp_config, otp_info)
        os.system('mkdir -p '+output_folder)

        all_image_output = output_folder + 'otp-all.image'
        data_image_output = output_folder + 'otp-data.image'
        data_binary_output = output_folder + 'otp-data.bin'
        data_hex_output = output_folder + 'otp-data.hex'
        config_image_output = output_folder + 'otp-conf.image'
        config_binary_output = output_folder + 'otp-conf.bin'
        strap_image_output = output_folder + 'otp-strap.image'
        strap_binary_output = output_folder + 'otp-strap.bin'

        data_region = bytearray()
        data_region_ignore = bytearray()
        data_all = bytearray()
        config_region = bytearray()
        config_region_ignore = bytearray()
        config_all = bytearray()
        otp_strap = bytearray()
        otp_strap_reg_protect = bytearray()
        otp_strap_protect = bytearray()
        otp_strap_ignore = bytearray()
        otp_strap_all = bytearray()

        image_info_all = 0
        data_size = 0
        config_size = 0
        strap_size = 0

        if 'data_region' in otp_config:
            print("Generating Data Image ...")
            data_region, data_region_ignore = self.make_data_region(
                otp_config['data_region'], key_folder,
                user_data_folder, genKeyHeader,
                key_to_bytearray, otp_info['data_region_size'],
                otp_info['ecc_region_offset'])

            data_size = data_region.__len__() + data_region_ignore.__len__()
            image_size = self.otp_info.HEADER_SIZE + data_size
            image_info = image_size | self.otp_info.INC_DATA
            image_info_all = image_info_all | self.otp_info.INC_DATA
            data_offset = self.otp_info.HEADER_SIZE
            data_info = data_offset | (data_size << 16)
            checksum_offset = data_offset + data_size
            header = struct.pack(
                self.otp_info.HEADER_FORMAT,
                self.otp_info.MAGIC_WORD_OTP.encode(),
                version.encode(),
                image_info,
                data_info,
                0,
                0,
                checksum_offset
            )

            data_all = data_region + data_region_ignore

            sha = SHA256.new(header+data_all)
            checksum = sha.digest()

            writeBinFile(header+data_all+checksum, data_image_output)
            writeBinFile(data_region, data_binary_output)
            writeDWHexFile(data_region, data_hex_output)
            writeBinFile(data_region_ignore,
                         output_folder + 'otp-data_mask.bin')

        if 'config_region' in otp_config:
            print("Generating Config Image ...")
            with open(otp_info['config'], 'r') as config_info_fd:
                config_info = jstyleson.load(config_info_fd)

            config_region, config_region_ignore = self.make_config_region(
                otp_config['config_region'], config_info,
                otp_info['config_region_size'])

            config_size = config_region.__len__() + config_region_ignore.__len__()
            image_size = self.otp_info.HEADER_SIZE + config_size
            image_info = image_size | self.otp_info.INC_CONF
            image_info_all = image_info_all | self.otp_info.INC_CONF
            config_offset = self.otp_info.HEADER_SIZE
            config_info = config_offset | (config_size << 16)
            checksum_offset = config_offset + config_size
            header = struct.pack(
                self.otp_info.HEADER_FORMAT,
                self.otp_info.MAGIC_WORD_OTP.encode(),
                version.encode(),
                image_info,
                0,
                config_info,
                0,
                checksum_offset
            )

            config_all = config_region + config_region_ignore

            sha = SHA256.new(header+config_all)
            checksum = sha.digest()

            writeBinFile(header+config_all+checksum, config_image_output)
            writeBinFile(config_region, config_binary_output)

        if 'otp_strap' in otp_config:
            print("Generating Strap Image ...")
            with open(otp_info['strap'], 'r') as strap_info_fd:
                strap_info = jstyleson.load(strap_info_fd)

            otp_strap, otp_strap_reg_protect, \
                otp_strap_protect, otp_strap_ignore = self.make_otp_strap(
                    otp_config['otp_strap'], strap_info,
                    otp_info['otp_strap_bit_size'])

            strap_size = otp_strap.__len__() + otp_strap_reg_protect.__len__() + \
                otp_strap_protect.__len__() + otp_strap_ignore.__len__()
            image_size = self.otp_info.HEADER_SIZE + strap_size
            image_info = image_size | self.otp_info.INC_STRAP
            image_info_all = image_info_all | self.otp_info.INC_STRAP
            strap_offset = self.otp_info.HEADER_SIZE
            strap_info = strap_offset | (strap_size << 16)
            checksum_offset = strap_offset + strap_size
            header = struct.pack(
                self.otp_info.HEADER_FORMAT,
                self.otp_info.MAGIC_WORD_OTP.encode(),
                version.encode(),
                image_info,
                0,
                0,
                strap_info,
                checksum_offset
            )

            otp_strap_all = otp_strap + otp_strap_reg_protect + \
                otp_strap_protect + otp_strap_ignore

            sha = SHA256.new(header+otp_strap_all)
            checksum = sha.digest()

            writeBinFile(header+otp_strap_all+checksum, strap_image_output)
            writeBinFile(otp_strap, strap_binary_output)

        print("Generating OTP-all Image ...")
        image_size_all = self.otp_info.HEADER_SIZE + data_size + config_size + strap_size
        image_info_all = image_info_all | image_size_all
        data_offset = self.otp_info.HEADER_SIZE
        config_offset = data_offset + data_size
        strap_offset = config_offset + config_size
        checksum_offset = strap_offset + strap_size
        data_info = data_offset | (data_size << 16)
        config_info = config_offset | (config_size << 16)
        strap_info = strap_offset | (strap_size << 16)

        header = struct.pack(
            self.otp_info.HEADER_FORMAT,
            self.otp_info.MAGIC_WORD_OTP.encode(),
            version.encode(),
            image_info_all,
            data_info,
            config_info,
            strap_info,
            checksum_offset
        )

        sha = SHA256.new(header+data_all+config_all +
                         otp_strap_all)
        checksum = sha.digest()

        writeBinFile(header+data_all+config_all +
                     otp_strap_all+checksum, all_image_output)


class otpTool(object):
    """Object for otptool command-line tool."""

    def __init__(self):
        """Initializer method."""
        self.otp = OTP()

    def run(self, argv):
        """Command-line processor.

        Arguments:
            argv: Pass sys.argv from main.
        """
        parser = argparse.ArgumentParser()
        parser.add_argument('config',
                            help='configuration file',
                            type=argparse.FileType('r'))

        parser.add_argument('--key_folder',
                            help='key folder',
                            type=parse_path,
                            default='')
        parser.add_argument('--user_data_folder',
                            help='user data folder',
                            type=parse_path,
                            default='')
        parser.add_argument('--output_folder',
                            help='output folder',
                            type=parse_path,
                            default='')
        parser.set_defaults(func=self.make_otp_image)
        args = parser.parse_args(argv[1:])

        try:
            args.func(args)
        except AttributeError:
            parser.error("too few arguments")
            sys.exit(1)

    def make_otp_image(self, args):
        self.otp.make_otp_image(args.config,
                                args.key_folder,
                                args.user_data_folder,
                                args.output_folder)


if __name__ == '__main__':

    tool = otpTool()
    tool.run(sys.argv)
