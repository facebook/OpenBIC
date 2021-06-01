#!/usr/bin/env python3


import argparse
import sys
import re
import os
import struct
import subprocess
import tempfile
import hexdump
from Crypto.Hash import SHA
from Crypto.Hash import SHA224
from Crypto.Hash import SHA256
from Crypto.Hash import SHA384
from Crypto.Hash import SHA512
from Crypto.Util import Counter
from Crypto.Cipher import AES
from Crypto.PublicKey import RSA
from bitarray import bitarray
from Crypto.Cipher import PKCS1_v1_5 as Cipher_pkcs1_v1_5

RSA_SHA = 1        # mode 2
AES_RSA_SHA = 2    # mode 2 with aes encryption
AES_GCM = 3        # mode gcm
HASH_BINDING = 4

RSA_OEM = 0x8
RSA_SOC_PUB = 0xa
RSA_SOC_PRI = 0xe
AES_OEM = 0x2
AES_VAULT = 0x1


class SecError(Exception):
    """Application-specific errors.

    These errors represent issues for which a stack-trace should not be
    presented.

    Attributes:
        message: Error message.
    """

    def __init__(self, message):
        Exception.__init__(self, message)


class Algorithm(object):
    """Contains details about an algorithm.

    See the avb_vbmeta_image.h file for more details about algorithms.

    Attributes:
      algorithm_type: Integer code corresponding to |AvbAlgorithmType|.
      hash_name: Empty or a name from |hashlib.algorithms|.
      hash_num_bytes: Number of bytes used to store the hash.
      signature_num_bytes: Number of bytes used to store the signature.
      public_key_num_bytes: Number of bytes used to store the public key.
      padding: Padding used for signature, if any.
    """

    def __init__(self, algorithm_type, hash_alg, hash_num_bytes,
                 signature_num_bytes, public_key_num_bytes):
        self.algorithm_type = algorithm_type
        self.hash_alg = hash_alg
        self.hash_num_bytes = hash_num_bytes
        self.signature_num_bytes = signature_num_bytes
        self.public_key_num_bytes = public_key_num_bytes


class ChainPartitionDescriptor(object):
    def __init__(self, image_path, out_path, sign_key_path, verify_key_path,
                 next_verify_key_path):
        self.image_path = image_path
        self.out_path = out_path
        self.sign_key_path = sign_key_path
        self.verify_key_path = verify_key_path
        self.next_verify_key_path = next_verify_key_path


def parse_path(path):
    if path is None or path == '':
        return os.path.dirname(os.path.abspath(__file__))+'/'
    if path[-1] != '/':
        path += '/'
    return path


def parse_number(string):
    """Parse a string as a number.

    This is just a short-hand for int(string, 0) suitable for use in the
    |type| parameter of |ArgumentParser|'s add_argument() function. An
    improvement to just using type=int is that this function supports
    numbers in other bases, e.g. "0x1234".

    Arguments:
        string: The string to parse.

    Returns:
        The parsed integer.

    Raises:
        ValueError: If the number could not be parsed.
    """
    return int(string, 0)


def rsa_bit_length(rsa_key_file: str, var: str) -> int:
    with open(rsa_key_file, 'rb') as f:
        key_file_bin = f.read()
        f.close()
    rsa_key = RSA.importKey(key_file_bin)

    return _rsa_bit_length(rsa_key, var)


def _rsa_bit_length(rsa_key, var):
    if var == 'n':
        key_bit_length = bitarray(bin(rsa_key.n)[2:]).length()
    elif var == 'e':
        key_bit_length = bitarray(bin(rsa_key.e)[2:]).length()
    elif var == 'd':
        key_bit_length = bitarray(bin(rsa_key.d)[2:]).length()
    return key_bit_length


def rsa_key_to_bin(rsa_key_file, types):
    with open(rsa_key_file, 'rb') as f:
        key_file_bin = f.read()
        f.close()
    rsa_key = RSA.importKey(key_file_bin)
    rsa_len = _rsa_bit_length(rsa_key, 'n')

    n = bitarray(bin(rsa_key.n)[2:])
    e = bitarray(bin(rsa_key.e)[2:])
    n_remain = (8-(n.length() % 8)) % 8
    e_remain = (8-(e.length() % 8)) % 8
    for _ in range(0, n_remain):
        n.insert(0, 0)
    for _ in range(0, e_remain):
        e.insert(0, 0)

    n = n.tobytes()
    e = e.tobytes()
    n_b = bytearray(n)
    e_b = bytearray(e)
    n_b.reverse()
    e_b.reverse()

    if types == 'public':
        exp = e_b
    elif types == 'private':
        d = bitarray(bin(rsa_key.d)[2:])
        d_remain = (8-(d.length() % 8)) % 8
        for _ in range(0, d_remain):
            d.insert(0, 0)
        d = d.tobytes()
        d_b = bytearray(d)
        d_b.reverse()
        exp = d_b
    else:
        raise ValueError("types error")

    if rsa_len == 1024:
        m_len = 128
    elif rsa_len == 2048:
        m_len = 256
    elif rsa_len == 3072:
        m_len = 384
    else:
        m_len = 512

    key_bin = bytearray(m_len * 2)
    insert_bytearray(n_b, key_bin, 0)
    insert_bytearray(exp, key_bin, m_len)

    return key_bin


def rsa_signature(alg_data, rsa_key_file, src_bin,
                  signing_helper, signing_helper_with_files):
    src_bin = bytearray(src_bin)
    src_bin.reverse()

    if signing_helper_with_files is not None:
        signing_file = tempfile.NamedTemporaryFile()
        signing_file.write(src_bin)
        signing_file.flush()
        p = subprocess.Popen([
            signing_helper_with_files, rsa_key_file, signing_file.name])
        retcode = p.wait()
        if retcode != 0:
            raise ValueError('Error signing')
        signing_file.seek(0)
        read_f = signing_file.read()

        signature = bytearray(read_f)
        signature.reverse()
    else:
        if signing_helper is not None:
            p = subprocess.Popen(
                [signing_helper, rsa_key_file],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
        else:
            p = subprocess.Popen(
                ['openssl', 'rsautl', '-sign', '-inkey', rsa_key_file],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
        (pout, perr) = p.communicate(input=src_bin)
        retcode = p.wait()
        if retcode != 0:
            raise ValueError('Error signing: {}'.format(perr))

        signature = bytearray(pout)
        signature.reverse()

    if signature.__len__() < 512:
        signature = signature + bytearray(512 - signature.__len__())
    return signature


def rsa_verify(alg_data, rsa_key_file, signature, digest):
    rev_signature = bytearray(signature)
    rev_signature.reverse()

    if rev_signature.__len__() < alg_data.signature_num_bytes:
        rev_signature = rev_signature + \
            bytearray(alg_data.signature_num_bytes - rev_signature.__len__())

    with open(rsa_key_file, 'rb') as f:
        key_file_bin = f.read()
        f.close()
    rsakey = RSA.importKey(key_file_bin)
    try:
        if rsakey.d:
            cmd = "openssl rsautl -verify -raw -inkey " + rsa_key_file
    except (AttributeError):
        cmd = "openssl rsautl -verify -raw --pubin -inkey " + rsa_key_file

    p = subprocess.Popen(
        cmd,
        shell=True,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)

    sign_dec, stderr = p.communicate(input=rev_signature)

    if stderr or p.returncode != 0:
        print(stderr)

    sign_dec = bytearray(sign_dec)
    sign_dec.reverse()
    if sign_dec[:digest.__len__()] == digest:
        return True
    else:
        return False


def rsa_encrypt(rsa_key_file, src_bin):
    with open(rsa_key_file, 'rb') as f:
        key_bin = f.read()
        f.close()
    rsa_key = RSA.importKey(key_bin)

    src_bin = bytearray(src_bin)
    src_bin.reverse()

    if rsa_key.has_private():
        t = RSA.construct((rsa_key.n, rsa_key.d, rsa_key.e))
        cipher = Cipher_pkcs1_v1_5.new(t)
        src_enc = cipher.encrypt(src_bin)
    else:
        cipher = Cipher_pkcs1_v1_5.new(rsa_key)
        src_enc = cipher.encrypt(src_bin)

    src_enc = bytearray(src_enc)
    src_enc.reverse()

    key_bit_length = bitarray(bin(rsa_key.n)[2:]).length()
    key_byte_length = int((key_bit_length + 7) / 8)
    if src_enc.__len__() < key_byte_length:
        src_enc = src_enc + bytearray(key_byte_length - src_enc.__len__())
    return src_enc


def rsa_decrypt(key_type, rsa_key_file, src_bin):
    with open(rsa_key_file, 'rb') as f:
        key_bin = f.read()
        f.close()
    rsa_key = RSA.importKey(key_bin)

    src_bin = bytearray(src_bin)
    src_bin.reverse()
    if key_type == 'public':
        cipher = Cipher_pkcs1_v1_5.new(rsa_key)
        src_enc = cipher.decrypt(src_bin, "")
    elif key_type == 'private':
        t = RSA.construct((rsa_key.n, rsa_key.d, rsa_key.e))
        cipher = Cipher_pkcs1_v1_5.new(t)
        src_enc = cipher.decrypt(src_bin, "")
    src_enc = bytearray(src_enc)
    src_enc.reverse()
    key_bit_length = bitarray(bin(rsa_key.n)[2:]).length()
    key_byte_length = int((key_bit_length + 7) / 8)
    if src_enc.__len__() < key_byte_length:
        src_enc = src_enc + bytearray(key_byte_length - src_enc.__len__())
    return src_enc


def insert_bytearray(src, dst, offset):
    if offset+src.__len__() > dst.__len__():
        dst.extend(bytearray(offset-dst.__len__()+src.__len__()))

    dst[offset:offset+src.__len__()] = src


class OTP_info(object):
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))+'/'
    MAGIC_WORD_OTP = 'SOCOTP'
    HEADER_FORMAT = '<8s8s5I'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
    CHECKSUM_LEN = 32
    INC_DATA = 1 << 31
    INC_CONF = 1 << 30
    INC_STRAP = 1 << 29
    INC_ECC = 1 << 28
    INC_DUMP = 1 << 27

    OTP_INFO = {
        'A0': {
            'config': ROOT_DIR+'otp_info/a0_config.json',
            'strap': ROOT_DIR+'otp_info/a0_strap.json',
            'data_region_size': 8192,
            'ecc_region_offset': 7168,
            'config_region_size': 64,
            'otp_strap_bit_size': 64,
        },
        'A1': {
            'config': ROOT_DIR+'otp_info/a1_config.json',
            'strap': ROOT_DIR+'otp_info/a1_strap.json',
            'data_region_size': 8192,
            'ecc_region_offset': 7168,
            'config_region_size': 64,
            'otp_strap_bit_size': 64,
        },
        'A2': {
            'config': ROOT_DIR+'otp_info/a2_config.json',
            'strap': ROOT_DIR+'otp_info/a2_strap.json',
            'data_region_size': 8192,
            'ecc_region_offset': 7168,
            'config_region_size': 64,
            'otp_strap_bit_size': 64,
        }
    }


class Sec(object):
    """Business logic for socsec command-line tool."""

    MAGIC_WORD_VB = 'SOCSEC'
    ROT_HEADER_FORMAT = '<8I'
    ROT_HEADER_SIZE = struct.calcsize(ROT_HEADER_FORMAT)
    COT_INFO_FORMAT = '<2I'
    COT_INFO_SIZE = struct.calcsize(COT_INFO_FORMAT)
    COT_HEADER_FORMAT = '<16s6I472s'
    COT_HEADER_SIZE = struct.calcsize(COT_HEADER_FORMAT)

    def parse_algorithm(self, algorithm_name):
        algorithm_type = -1
        rsa_len = 0
        sha_len = 0
        match_alg = re.match(
            r'AES_RSA(1024|2048|3072|4096)_SHA(224|256|384|512)',
            algorithm_name, re.M)

        if match_alg:
            algorithm_type = AES_RSA_SHA
            rsa_len = match_alg.group(1)
            sha_len = match_alg.group(2)

        match_alg = re.match(
            r'RSA(1024|2048|3072|4096)_SHA(224|256|384|512)',
            algorithm_name, re.M)

        if match_alg:
            algorithm_type = RSA_SHA
            rsa_len = match_alg.group(1)
            sha_len = match_alg.group(2)

        if algorithm_name == 'AES_GCM':
            algorithm_type = AES_GCM

        match_alg = re.match(
            r'SHA(224|256|384|512)',
            algorithm_name, re.M)

        if match_alg:
            algorithm_type = HASH_BINDING
            sha_len = match_alg.group(1)

        if algorithm_type == -1:
            raise SecError('Algorithm is invalid')

        if rsa_len == '1024':
            signature_num_bytes = 128
            public_key_num_bytes = 2*1024//8
        elif rsa_len == '2048':
            signature_num_bytes = 256
            public_key_num_bytes = 2*2048//8
        elif rsa_len == '3072':
            signature_num_bytes = 384
            public_key_num_bytes = 2*3072//8
        elif rsa_len == '4096':
            signature_num_bytes = 512
            public_key_num_bytes = 2*4096//8
        else:
            signature_num_bytes = 0
            public_key_num_bytes = 0

        if sha_len == '224':
            hash_alg = SHA224
            hash_num_bytes = 28
        elif sha_len == '256':
            hash_alg = SHA256
            hash_num_bytes = 32
        elif sha_len == '384':
            hash_alg = SHA384
            hash_num_bytes = 48
        elif sha_len == '512':
            hash_alg = SHA512
            hash_num_bytes = 64
        else:
            hash_alg = None
            hash_num_bytes = 0

        return Algorithm(algorithm_type, hash_alg, hash_num_bytes,
                         signature_num_bytes, public_key_num_bytes)

    def verify_bl1_mode_2_image(self, sec_image, verify_key_path, alg_data,
                                sign_image_size, signature_offset):
        sha = alg_data.hash_alg.new(sec_image[0:sign_image_size])
        digest = sha.digest()
        image_signature = sec_image[signature_offset:
                                    (signature_offset+alg_data.signature_num_bytes)]
        if not rsa_verify(alg_data, verify_key_path,
                          image_signature,
                          digest):
            raise SecError("signature verify failed")

    def decode_bl1_mode_2_enc_image(self, image, sec_image, header_offset,
                                    enc_offset, aes_data_offset, aes_key,
                                    rsa_aes_key_path, alg_data,
                                    sign_image_size,
                                    key_in_otp):
        # if not key_in_otp:
        #     rsa_key_length = alg_data.signature_num_bytes
        #     key_obj = sec_image[aes_data_offset:aes_data_offset + rsa_key_length]
        #     aes_object = rsa_decrypt(
        #         'public', rsa_aes_key_path, bytes(key_obj))
        #     aes_key = bytes(aes_object[0:0x20])
        #     aes_iv = bytes(aes_object[0x20:0x30])
        # else:
        aes_iv = sec_image[aes_data_offset:aes_data_offset+16]

        ctr = Counter.new(128, initial_value=int.from_bytes(
            aes_iv, byteorder='big'))
        aes = AES.new(aes_key, AES.MODE_CTR, counter=ctr)
        dec_image = bytearray(aes.decrypt(
            bytes(sec_image[enc_offset:sign_image_size])))
        if image[enc_offset:] != dec_image:
            raise SecError("image decrypt failed")

    def verify_bl1_mode_gcm_image(self, image, sec_image, signature_offset,
                                  sign_image_size,
                                  enc_offset, aes_data_offset, aes_key):
        image_signature = sec_image[signature_offset:signature_offset+16]
        rev_signature = bytearray(16)
        for i in range(0, 15, 4):
            rev_signature[i] = image_signature[12-i]
            rev_signature[i+1] = image_signature[13-i]
            rev_signature[i+2] = image_signature[14-i]
            rev_signature[i+3] = image_signature[15-i]

        aes_iv = sec_image[aes_data_offset:aes_data_offset+12]
        try:
            aes = AES.new(aes_key, AES.MODE_GCM, nonce=aes_iv[0:12])
            aes.update(sec_image[:enc_offset])
            plaintext = aes.decrypt_and_verify(
                sec_image[enc_offset:sign_image_size], rev_signature)
        except (ValueError, KeyError):
            raise SecError("gcm verify failed")

        # if image[enc_offset:] != plaintext[:image.__len__()-enc_offset]:
        if image[enc_offset:] != plaintext:
            raise SecError("image decrypt failed")

    def verify_bl1_image(self, image, sec_image, header_offset, verify_key_path,
                         gcm_aes_key, aes_key,
                         rsa_aes_key_path, alg_data, key_in_otp):
        header = sec_image[header_offset:header_offset+self.ROT_HEADER_SIZE]
        (aes_data_offset, enc_offset, sign_image_size,
         signature_offset, revision_low, revision_high,
         reserved, bl1_header_checksum) = struct.unpack(self.ROT_HEADER_FORMAT, header)

        if((aes_data_offset + enc_offset + sign_image_size +
            signature_offset + revision_low + revision_high +
                reserved + bl1_header_checksum) & 0xffffffff) != 0:
            raise SecError("header checksum verify failed")
        if sign_image_size % 512:
            raise SecError("The sign_image_size should be 512 bytes aligned")

        print("check header PASS")

        if alg_data.algorithm_type in [RSA_SHA, AES_RSA_SHA]:
            self.verify_bl1_mode_2_image(sec_image, verify_key_path, alg_data,
                                         sign_image_size, signature_offset)
            print("check integrity PASS")

        if alg_data.algorithm_type == AES_RSA_SHA and key_in_otp:
            self.decode_bl1_mode_2_enc_image(image, sec_image, header_offset,
                                             enc_offset, aes_data_offset, aes_key,
                                             rsa_aes_key_path, alg_data,
                                             sign_image_size,
                                             key_in_otp)
            print("check aes decode PASS")

        if alg_data.algorithm_type == AES_GCM:
            self.verify_bl1_mode_gcm_image(image, sec_image, signature_offset,
                                           sign_image_size, enc_offset,
                                           aes_data_offset, gcm_aes_key)
            print("check gcm integrity and decode PASS")

    def verify_sv_chain_image(self, image_list, first_layer_vk_path, alg_data):
        for i in range(0, len(image_list)):
            print('BL{:d} verifying'.format(i + 2))
            part_image = image_list[i]
            header = part_image[:self.COT_HEADER_SIZE]

            (magic_word, info, sign_image_size, signature_offset,
             npkey_offset, revision_low, revision_high, reserved) = struct.unpack(self.COT_HEADER_FORMAT, header)

            mw = self.MAGIC_WORD_VB.encode()

            if magic_word[0:len(mw)] == mw:
                print("check header magic word PASS")
            else:
                raise SecError("header magic word verify failed")

            if i == 0:
                va = alg_data
                vk = first_layer_vk_path
            else:
                va = next_alg_data
                vk = next_vk

            if i != len(image_list) - 1:
                next_alg_data = self.parse_cot_info(info)
                next_vk_n = part_image[npkey_offset:npkey_offset +
                                       int(next_alg_data.public_key_num_bytes / 2)]
                next_vk_e = part_image[int(npkey_offset + next_alg_data.public_key_num_bytes / 2):
                                       npkey_offset + next_alg_data.public_key_num_bytes]
                next_vk_n = int.from_bytes(
                    next_vk_n, byteorder='little', signed=False)
                next_vk_e = int.from_bytes(
                    next_vk_e, byteorder='little', signed=False)
                next_vk_obj = RSA.construct((next_vk_n, next_vk_e))
                signing_file = tempfile.NamedTemporaryFile()
                next_vk_obj.exportKey
                signing_file.write(next_vk_obj.publickey().exportKey('PEM'))
                signing_file.flush()
                next_vk = signing_file.name

            self.verify_bl1_mode_2_image(part_image, vk, va,
                                         sign_image_size, signature_offset)
            print("check integrity PASS")

    def make_bl1_mode_2_image(self, image, alg_data, sign_key_path,
                              header_offset,
                              sign_image_size, signature_offset,
                              signing_helper, signing_helper_with_files):

        # sign the image
        sha = alg_data.hash_alg.new(image[0:sign_image_size])
        digest = sha.digest()
        signature = rsa_signature(
            alg_data, sign_key_path, digest,
            signing_helper, signing_helper_with_files)

        insert_bytearray(signature, image, signature_offset)

    def make_bl1_mode_2_enc_image(self, image, alg_data, sign_key_path,
                                  header_offset,
                                  sign_image_size, signature_offset,
                                  enc_offset, aes_key, aes_iv, aes_data_offset,
                                  key_in_otp, rsa_aes_key_path,
                                  signing_helper, signing_helper_with_files):

        # encrypt image
        ctr = Counter.new(128, initial_value=int.from_bytes(
            aes_iv, byteorder='big'))
        aes = AES.new(aes_key, AES.MODE_CTR, counter=ctr)
        enc_image = bytearray(aes.encrypt(
            bytes(image[enc_offset:sign_image_size])))

        insert_bytearray(enc_image, image, enc_offset)

        # insert aes object
        if key_in_otp:
            insert_bytearray(aes_iv, image, aes_data_offset)
        else:
            aes_object = bytearray(48)
            insert_bytearray(aes_key, aes_object, 0)
            insert_bytearray(aes_iv, aes_object, 0x20)
            enc_aes_object = rsa_encrypt(rsa_aes_key_path, bytes(aes_object))
            insert_bytearray(enc_aes_object, image, aes_data_offset)

        # sign the image
        sha = alg_data.hash_alg.new(image[0:sign_image_size])
        digest = sha.digest()
        signature = rsa_signature(
            alg_data, sign_key_path, digest,
            signing_helper, signing_helper_with_files)
        insert_bytearray(signature, image, signature_offset)

    def make_bl1_mode_gcm_image(self, image, alg_data, gcm_aes_key, gcm_aes_iv,
                                header_offset, enc_offset,
                                sign_image_size, signature_offset,
                                aes_data_offset,
                                signing_helper, signing_helper_with_files):

        aes = AES.new(gcm_aes_key, AES.MODE_GCM, nonce=gcm_aes_iv[0:12])
        aes.update(image[:enc_offset])
        ciphertext, signature = aes.encrypt_and_digest(
            image[enc_offset:sign_image_size])

        # reverse signature for secure boot rom code
        rev_signature = bytearray(16)
        for i in range(0, 15, 4):
            rev_signature[i] = signature[12-i]
            rev_signature[i+1] = signature[13-i]
            rev_signature[i+2] = signature[14-i]
            rev_signature[i+3] = signature[15-i]

        insert_bytearray(ciphertext, image, enc_offset)
        insert_bytearray(gcm_aes_iv, image, aes_data_offset)
        insert_bytearray(rev_signature, image, signature_offset)

    def _cot_info(self, cot_alg_data, rsa_e_bits_len):
        info = 0
        if cot_alg_data.algorithm_type == RSA_SHA:
            info = 1
            if cot_alg_data.signature_num_bytes == 128:
                info |= 0
            elif cot_alg_data.signature_num_bytes == 256:
                info |= 1 << 3
            elif cot_alg_data.signature_num_bytes == 384:
                info |= 2 << 3
            elif cot_alg_data.signature_num_bytes == 512:
                info |= 3 << 3
            info |= rsa_e_bits_len << 20
        elif cot_alg_data.algorithm_type == HASH_BINDING:
            info = 0
        else:
            raise SecError(
                "COT only support RSA_SHA and HASH_BINDING algorithm")

        if cot_alg_data.hash_num_bytes == 28:
            info |= 0
        elif cot_alg_data.hash_num_bytes == 32:
            info |= 1 << 1
        elif cot_alg_data.hash_num_bytes == 48:
            info |= 2 << 1
        elif cot_alg_data.hash_num_bytes == 64:
            info |= 3 << 1
        return info

    def parse_cot_info(self, info):
        if info & 1:
            algorithm_type = RSA_SHA
        else:
            algorithm_type = HASH_BINDING

        rsa_info = (info >> 3) & 3

        if rsa_info == 0:
            signature_num_bytes = 128
            public_key_num_bytes = 2*1024//8
        elif rsa_info == 1:
            signature_num_bytes = 256
            public_key_num_bytes = 2*2048//8
        elif rsa_info == 2:
            signature_num_bytes = 384
            public_key_num_bytes = 2*3072//8
        else:
            signature_num_bytes = 512
            public_key_num_bytes = 2*4096//8

        hash_info = (info >> 1) & 3

        if hash_info == 0:
            hash_alg = SHA224
            hash_num_bytes = 28
        elif hash_info == 1:
            hash_alg = SHA256
            hash_num_bytes = 32
        elif hash_info == 2:
            hash_alg = SHA384
            hash_num_bytes = 48
        else:
            hash_alg = SHA512
            hash_num_bytes = 64

        return Algorithm(algorithm_type, hash_alg, hash_num_bytes,
                         signature_num_bytes, public_key_num_bytes)

    def insert_bl1_cot_info(self, image, cot_alg_data, cot_header_offset,
                            verify_key_path, cot_digest_fd, cot_data_offset):

        if cot_alg_data.algorithm_type == RSA_SHA:
            e_bits = rsa_bit_length(verify_key_path, 'e')
            cot_data = bytearray(rsa_key_to_bin(verify_key_path, 'public'))
        elif cot_alg_data.algorithm_type == HASH_BINDING:
            cot_data = bytearray(cot_digest_fd.read())
        else:
            raise SecError(
                "COT only support RSA_SHA and HASH_BINDING algorithm")

        info = self._cot_info(cot_alg_data, e_bits)
        cot_header = struct.pack(
            self.COT_INFO_FORMAT,
            info,
            cot_data_offset
        )

        insert_bytearray(cot_header, image, cot_header_offset)

        insert_bytearray(cot_data, image, cot_data_offset)

    def make_secure_bl1_image(self, soc_version, bl1_image_fd, sign_key_path, gcm_aes_key_fd, output_fd,
                              algorithm_name, header_offset, rollback_index,
                              enc_offset, aes_key_fd, rsa_aes_key_path,
                              key_in_otp,
                              cot_algorithm_name,
                              cot_verify_key_path,
                              cot_digest_fd,
                              signing_helper,
                              signing_helper_with_files):
        """Implements the 'make_vbmeta_image' command.

        Arguments:
            bl1_image: Bootloader 1 image.
            sign_key_path: Path to rsa signing key to use or None.
            output: File to write the image to.
            algorithm_name: Name of algorithm to use.
            rollback_index: The rollback index to use.
            enc_offset:
            aes_key_fd;
            rsa_aes_key_path;
            key_in_otp:
            cot_algorithm_name:
            cot_verify_key_path:
            cot_digest_fd:
            signing_helper: Program which signs a hash and return signature.
            signing_helper_with_files: Same as signing_helper but uses files instead.

        Raises:
            SecError: If a chained partition is malformed.
        """
        aes_key = None
        gcm_aes_key = None

        bl1_image = bytearray(bl1_image_fd.read())
        bl1_image_len = len(bl1_image)
        if soc_version == '2600':
            if header_offset == None:
                header_offset = 0x20
            if enc_offset == None:
                enc_offset = 0x50
            if bl1_image_len > (60 * 1024):
                raise SecError("The maximum size of BL1 image is 60 KBytes.")
        elif soc_version == '1030':
            if header_offset == None:
                header_offset = 0x400
            if enc_offset == None:
                enc_offset = 0x430
            if bl1_image_len > (768 * 1024):
                raise SecError("The maximum size of BL1 image is 768 KBytes.")
        else:
            raise SecError("SOC version is not avaliable")

        if enc_offset % 16:
            raise SecError("The enc_offset should be 16 bytes aligned")

        if enc_offset < header_offset + 0x30:
            raise SecError(
                "The enc_offset should 0x30 more than header_offset")

        alg_data = self.parse_algorithm(algorithm_name)

        aes_data_offset = 0
        sign_image_size = 0
        signature_offset = 0
        revision_low = 0
        revision_high = 0
        bl1_header_checksum = 0

        if cot_algorithm_name:
            cot_alg_data = self.parse_algorithm(cot_algorithm_name)
            cot_header_offset = header_offset + self.ROT_HEADER_SIZE
            cot_data_offset = max((bl1_image_len + 0xf) & (~0xf),
                                  (cot_header_offset + self.COT_INFO_SIZE + 0xf) & (~0xf))
            if cot_alg_data.algorithm_type == RSA_SHA:
                sign_image_size = cot_data_offset + cot_alg_data.public_key_num_bytes
            elif cot_alg_data.algorithm_type == HASH_BINDING:
                sign_image_size = cot_data_offset + cot_alg_data.hash_num_bytes
            else:
                raise SecError(
                    "COT only support RSA_SHA and HASH_BINDING algorithm")
            sign_image_size = (sign_image_size + 511) & (~511)
        else:
            sign_image_size = (bl1_image_len + 511) & (~511)

        if alg_data.algorithm_type == AES_RSA_SHA:
            aes_data_offset = sign_image_size
            if key_in_otp:
                signature_offset = sign_image_size + 16
            else:
                signature_offset = sign_image_size + alg_data.signature_num_bytes
        elif alg_data.algorithm_type == AES_GCM:
            aes_data_offset = sign_image_size
            signature_offset = sign_image_size + 16
        else:
            enc_offset = 0
            signature_offset = sign_image_size

        bl1_header_checksum = -(aes_data_offset + enc_offset +
                                sign_image_size + signature_offset +
                                revision_low + revision_high) & 0xFFFFFFFF
        bl1_header = struct.pack(
            self.ROT_HEADER_FORMAT,
            aes_data_offset,
            enc_offset,
            sign_image_size,
            signature_offset,
            revision_low,
            revision_high,
            0,
            bl1_header_checksum
        )
        bl1_image.extend(bytearray(sign_image_size - bl1_image_len))
        output_image = bytearray(sign_image_size)
        insert_bytearray(bl1_image, output_image, 0)
        # insert header
        insert_bytearray(bl1_header, output_image, header_offset)

        if cot_algorithm_name:
            self.insert_bl1_cot_info(output_image, cot_alg_data,
                                     cot_header_offset, cot_verify_key_path,
                                     cot_digest_fd, cot_data_offset)
            plain_image = output_image.copy()
        else:
            plain_image = bl1_image

        if alg_data.algorithm_type == RSA_SHA:
            if not sign_key_path:
                raise SecError("Missing sign key")
            self.make_bl1_mode_2_image(output_image, alg_data, sign_key_path,
                                       header_offset,
                                       sign_image_size, signature_offset,
                                       signing_helper, signing_helper_with_files)
        elif alg_data.algorithm_type == AES_RSA_SHA:
            aes_iv = open('/dev/urandom', 'rb').read(16)

            if aes_key_fd:
                aes_key = bytearray(aes_key_fd.read())
            else:
                raise SecError("Missing aes_key for AES_RSA_SHA")
            if (not key_in_otp) & (not rsa_aes_key_path):
                raise SecError(
                    "Missing --rsa_aes when key is not in otp")
            self.make_bl1_mode_2_enc_image(output_image, alg_data, sign_key_path,
                                           header_offset,
                                           sign_image_size, signature_offset,
                                           enc_offset, aes_key,
                                           aes_iv, aes_data_offset,
                                           key_in_otp, rsa_aes_key_path,
                                           signing_helper,
                                           signing_helper_with_files)
        elif alg_data.algorithm_type == AES_GCM:
            gcm_aes_iv = open('/dev/urandom', 'rb').read(12)
            gcm_aes_iv = gcm_aes_iv + b"\x00\x00\x00\x01"

            if gcm_aes_key_fd:
                gcm_aes_key = bytearray(gcm_aes_key_fd.read())
            else:
                raise SecError("Missing gcm_aes_key for AES_GCM")
            self.make_bl1_mode_gcm_image(output_image, alg_data, gcm_aes_key, gcm_aes_iv,
                                         header_offset, enc_offset,
                                         sign_image_size, signature_offset,
                                         aes_data_offset,
                                         signing_helper, signing_helper_with_files)
        else:
            raise SecError("Algorithm not supported")

        self.verify_bl1_image(plain_image, output_image, header_offset,
                              sign_key_path, gcm_aes_key,
                              aes_key, rsa_aes_key_path,
                              alg_data, key_in_otp)

        output_fd.write(bytes(output_image))
        output_fd.close()

    def make_sv_partation_image(self, alg_data, descriptor,
                                signing_helper, signing_helper_with_files):

        image = bytearray(open(descriptor.image_path, 'rb').read())
        image[:0] = bytearray(self.COT_HEADER_SIZE)
        image_len = len(image)

        sign_image_size = 0
        signature_offset = 0
        npkey_offset = 0
        revision_low = 0
        revision_high = 0

        if descriptor.next_verify_key_path:
            npkey_bin = bytearray(rsa_key_to_bin(
                descriptor.next_verify_key_path, 'public'))
            npkey_offset = (image_len + 0xf) & (~0xf)
            sign_image_size = npkey_offset + alg_data.public_key_num_bytes
            insert_bytearray(npkey_bin, image, npkey_offset)

            e_bits = rsa_bit_length(descriptor.next_verify_key_path, 'e')
            info = self._cot_info(alg_data, e_bits)
        else:
            sign_image_size = (image_len + 0xf) & (~0xf)
            info = 0

        sign_image_size = (sign_image_size + 511) & (~511)
        signature_offset = sign_image_size

        header = struct.pack(
            self.COT_HEADER_FORMAT,
            self.MAGIC_WORD_VB.encode(),
            info,
            sign_image_size,
            signature_offset,
            npkey_offset,
            revision_low,
            revision_high,
            bytearray(472)
        )
        insert_bytearray(header, image, 0)

        image.extend(bytearray(sign_image_size - image_len))
        sha = alg_data.hash_alg.new(image[0:sign_image_size])
        digest = sha.digest()

        signature = rsa_signature(
            alg_data, descriptor.sign_key_path, digest,
            signing_helper, signing_helper_with_files)

        insert_bytearray(signature, image, signature_offset)

        return image

    def make_sv_chain_image(self, algorithm, cot_part, rollback_index,
                            image_relative_path,
                            signing_helper, signing_helper_with_files):

        cot_alg_data = self.parse_algorithm(algorithm)

        descriptors = []
        part_count = len(cot_part)
        for i in range(0, part_count):
            part_list = cot_part[i].split(':')
            if len(part_list) != 4:
                raise SecError(
                    'Malformed chained partition "{}".'.format(cot_part[i]))
            if i < part_count - 1:
                npart_list = cot_part[i + 1].split(':')
                if len(npart_list) != 4:
                    raise SecError(
                        'Malformed chained partition "{}".'.format(cot_part[i + 1]))
                next_part_verify_key_path = npart_list[2]
            else:
                next_part_verify_key_path = None

            input_image_path = image_relative_path + part_list[0]
            output_image_path = image_relative_path + part_list[1]
            sign_key_path = part_list[2]
            verify_key_path = part_list[3]

            desc = ChainPartitionDescriptor(input_image_path,
                                            output_image_path,
                                            sign_key_path,
                                            verify_key_path,
                                            next_part_verify_key_path)
            descriptors.append(desc)

        image_list = []
        for desc in descriptors:

            sec_image = self.make_sv_partation_image(
                cot_alg_data, desc, signing_helper, signing_helper_with_files)

            open(desc.out_path, 'w+b').write(bytes(sec_image))
            image_list.append(sec_image)

        self.verify_sv_chain_image(image_list, descriptors[0].verify_key_path,
                                   cot_alg_data)


class SecureBootVerify(object):
    otp = OTP_info
    sec = Sec()

    def parse_otp(self, otp_image):
        header = otp_image[0:self.otp.HEADER_SIZE]

        (magic_b, ver_b, image_info, data_info, config_info, strap_info,
         checksum_offset) = struct.unpack(self.otp.HEADER_FORMAT, header)

        magic = magic_b[0:len(self.otp.MAGIC_WORD_OTP)].decode()
        if magic != self.otp.MAGIC_WORD_OTP:
            raise SecError('OTP image magic word is invalid')
        ver = ver_b.decode()
        if ver[:2] == 'A0':
            otp_info = self.otp.OTP_INFO['A0']
        elif ver[:2] == 'A1':
            otp_info = self.otp.OTP_INFO['A1']
        elif ver[:2] == 'A2':
            otp_info = self.otp.OTP_INFO['A2']
        elif ver[:6] == '1030A0':
            otp_info = self.otp.OTP_INFO['A2']
        else:
            raise SecError('OTP image version is invalid')

        image_size = image_info & 0xffff
        sha = SHA256.new(otp_image[:image_size])
        digest = sha.digest()

        if digest != otp_image[checksum_offset:checksum_offset+32]:
            raise SecError('OTP image checksum is invalid')

        data_offset = data_info & 0xffff
        data_size = (data_info >> 16) & 0xffff
        conf_offset = config_info & 0xffff
        conf_size = (config_info >> 16) & 0xffff

        if image_info & self.otp.INC_DUMP:
            data_region = otp_image[data_offset:data_offset+data_size]
            config_region = otp_image[conf_offset:conf_offset+conf_size]
        else:
            d_size = otp_info['data_region_size']
            c_size = otp_info['config_region_size']
            data_region = bytearray(d_size)
            config_region = bytearray(c_size)
            img_data = otp_image[data_offset:data_offset+d_size]
            img_dmask = otp_image[data_offset+d_size:data_offset+d_size*2]
            img_conf = otp_image[conf_offset:conf_offset+c_size]
            img_cmask = otp_image[conf_offset+c_size:conf_offset+c_size*2]
            for i in range(d_size):
                data_region[i] = img_data[i] & ~img_dmask[i]
            for i in range(c_size):
                config_region[i] = img_conf[i] & ~img_cmask[i]

        return data_region, config_region

    def parse_config(self, soc_version, config_region):
        cfg0 = struct.unpack('<I', config_region[0:4])[0]
        cfg3 = struct.unpack('<I', config_region[8:12])[0]
        cfg4 = struct.unpack('<I', config_region[12:16])[0]

        sb_mode = (cfg0 >> 7) & 0x1
        rsa_len = (cfg0 >> 10) & 0x3
        sha_len = (cfg0 >> 12) & 0x3
        enc_mode = (cfg0 >> 27) & 0x1
        header_offset = cfg3 & 0xffff
        key_retire = (cfg4 & 0x7f) | ((cfg4 >> 16) & 0x7f)
        retire_list = [0]*7

        if header_offset == 0:
            if soc_version == '2600':
                header_offset = 0x20
            elif soc_version == '1030':
                header_offset = 0x400

        for i in range(7):
            bit = (key_retire >> i) & 0x1
            if bit == 1:
                retire_list[i] = 1

        if sb_mode == 0:
            algorithm_type = AES_GCM
            print('Algorithm: AES_GCM')
        else:
            if enc_mode == 1:
                algorithm_type = AES_RSA_SHA
                print('Algorithm: AES_RSA_SHA')
            else:
                algorithm_type = RSA_SHA
                print('Algorithm: RSA_SHA')

        if rsa_len == 0:
            signature_num_bytes = 128
            public_key_num_bytes = 2*1024//8
            print('RSA length: 1024')
        elif rsa_len == 1:
            signature_num_bytes = 256
            public_key_num_bytes = 2*2048//8
            print('RSA length: 2048')
        elif rsa_len == 2:
            signature_num_bytes = 384
            public_key_num_bytes = 2*3072//8
            print('RSA length: 3072')
        else:
            signature_num_bytes = 512
            public_key_num_bytes = 2*4096//8
            print('RSA length: 4096')

        if sha_len == 0:
            hash_alg = SHA224
            hash_num_bytes = 28
            print('HASH length: 224')
        elif sha_len == 1:
            hash_alg = SHA256
            hash_num_bytes = 32
            print('HASH length: 256')
        elif sha_len == 2:
            hash_alg = SHA384
            hash_num_bytes = 48
            print('HASH length: 384')
        else:
            hash_alg = SHA512
            hash_num_bytes = 64
            print('HASH length: 512')

        return Algorithm(algorithm_type, hash_alg, hash_num_bytes,
                         signature_num_bytes, public_key_num_bytes), \
            header_offset, retire_list

    def parse_data(self, alg_data, data_region, retire_list):
        key_list = []
        find_last = 0
        for i in range(16):
            h = struct.unpack('<I', data_region[(i*4):(i*4+4)])[0]
            kl = {}
            kl['ID'] = h & 0x7
            kl['RETIRE'] = retire_list[kl['ID']]
            kl['OFFSET'] = ((h >> 3) & 0x3ff) << 3
            kl['TYPE'] = (h >> 14) & 0xf
            kl['PAR'] = (h >> 18) & 0x3
            key_list.append(kl)
            if h & (1 << 13):
                find_last = 1
                break

        if find_last == 0:
            raise SecError("Can not find Last List in OTP data region")

        for kl in key_list:
            if kl['TYPE'] in [RSA_OEM, RSA_SOC_PUB, RSA_SOC_PRI]:
                if kl['PAR'] == 0:
                    rsa_len = 1024//8
                elif kl['PAR'] == 1:
                    rsa_len = 2048//8
                elif kl['PAR'] == 2:
                    rsa_len = 3072//8
                else:
                    rsa_len = 4096//8
                if rsa_len != alg_data.signature_num_bytes:
                    raise SecError(
                        "OTP key type is not compatible with config")

                kl['M'] = data_region[kl['OFFSET']: kl['OFFSET']+rsa_len]
                if kl['TYPE'] == RSA_SOC_PRI:
                    e_len = rsa_len
                else:
                    e_len = 3
                kl['E'] = data_region[kl['OFFSET'] +
                                      rsa_len: kl['OFFSET']+rsa_len + e_len]
            elif kl['TYPE'] == AES_OEM:
                kl['AES'] = data_region[kl['OFFSET']: kl['OFFSET']+32]
            elif kl['TYPE'] == AES_VAULT:
                kl['AES1'] = data_region[kl['OFFSET']: kl['OFFSET']+32]
                kl['AES2'] = data_region[kl['OFFSET']+32: kl['OFFSET']+64]

        return key_list

    def mode2_verify(self, sec_image, sign_image_size, signature_offset,
                     alg_data, key_list):
        verify_pass = 0
        v_id = 0
        _signature = sec_image[signature_offset:signature_offset +
                               alg_data.signature_num_bytes]
        signature = int.from_bytes(
            _signature, byteorder='little', signed=False)
        sign_image = sec_image[:sign_image_size]
        sha = alg_data.hash_alg.new(sign_image)
        image_hash = sha.digest()
        for kl in key_list:
            if kl['TYPE'] != RSA_OEM:
                continue
            if kl['RETIRE'] == 1:
                continue
            M = int.from_bytes(kl['M'], byteorder='little', signed=False)
            E = int.from_bytes(kl['E'], byteorder='little', signed=False)
            hashFromSignature = pow(signature, E, M)
            hb = hashFromSignature.to_bytes(
                alg_data.signature_num_bytes, byteorder='little', signed=False)
            if hb[:alg_data.hash_num_bytes] == image_hash:
                v_id = kl['ID']
                verify_pass = 1
                break

        if verify_pass == 0:
            raise SecError("Mode 2 verify failed")

        return v_id

    def mode2_decrypt(self, sec_image, sign_image_size, enc_offset,
                      aes_data_offset, v_id, alg_data, key_list):
        option = 0
        for kl in key_list:
            if kl['TYPE'] == AES_OEM:
                key = kl
                option = 1
                break
        if option == 0:
            for kl in key_list:
                if kl['TYPE'] in [RSA_SOC_PUB, RSA_SOC_PRI]:
                    key = kl
                    option = 2
                    break
        if option == 0:
            for kl in key_list:
                if kl['ID'] == v_id:
                    key = kl
                    option = 3
                    break

        if option == 1:
            aes_key = key['AES']
            aes_iv = sec_image[aes_data_offset:aes_data_offset+16]
        if option in [2, 3]:
            rsa_key_length = alg_data.signature_num_bytes
            _key_obj = sec_image[aes_data_offset:aes_data_offset + rsa_key_length]
            key_obj = int.from_bytes(
                _key_obj, byteorder='little', signed=False)
            M = int.from_bytes(key['M'], byteorder='little', signed=False)
            E = int.from_bytes(key['E'], byteorder='little', signed=False)
            D = pow(key_obj, E, M)
            aes_object = D.to_bytes(
                alg_data.signature_num_bytes, byteorder='little', signed=False)
            aes_key = bytes(aes_object[0:0x20])
            aes_iv = bytes(aes_object[0x20:0x30])

        ctr = Counter.new(128, initial_value=int.from_bytes(
            aes_iv, byteorder='big'))
        aes = AES.new(aes_key, AES.MODE_CTR, counter=ctr)
        dec_image = bytearray(aes.decrypt(
            bytes(sec_image[enc_offset:sign_image_size])))

        return sec_image[:enc_offset] + dec_image

    def modeGCM_verify_n_decrypt(self, sec_image, sign_image_size,
                                 signature_offset, enc_offset, aes_data_offset,
                                 key_list):
        v_id = 0

        image_signature = sec_image[signature_offset:signature_offset+16]
        rev_signature = bytearray(16)
        for i in range(0, 15, 4):
            rev_signature[i] = image_signature[12-i]
            rev_signature[i+1] = image_signature[13-i]
            rev_signature[i+2] = image_signature[14-i]
            rev_signature[i+3] = image_signature[15-i]

        aes_iv = sec_image[aes_data_offset:aes_data_offset+12]

        for kl in key_list:
            if kl['TYPE'] != AES_OEM:
                continue
            verify_pass = 1
            try:
                aes = AES.new(kl['AES'], AES.MODE_GCM, nonce=aes_iv[0:12])
                aes.update(sec_image[:enc_offset])
                plaintext = aes.decrypt_and_verify(
                    sec_image[enc_offset:sign_image_size], rev_signature)
            except (ValueError, KeyError):
                verify_pass = 0
            if verify_pass == 1:
                v_id = kl['ID']
                break

        if verify_pass == 0:
            raise SecError("Mode GCM verify failed")

        return sec_image[:enc_offset] + plaintext, v_id

    def verify_rot_image(self, sec_image, header_offset, alg_data, key_list):
        dec_image = None

        header = sec_image[header_offset:header_offset +
                           self.sec.ROT_HEADER_SIZE]
        (aes_data_offset, enc_offset, sign_image_size, signature_offset,
         _, _, _, _) = struct.unpack(self.sec.ROT_HEADER_FORMAT, header)

        if alg_data.algorithm_type == RSA_SHA and enc_offset != 0:
            raise SecError(
                "The secure image is encrypted, the OTP image is not match")
        elif alg_data.algorithm_type in [AES_RSA_SHA, AES_GCM] and enc_offset == 0:
            raise SecError(
                "The OTP config is enable encryption mode, but the secure image is not encrypted")

        if alg_data.algorithm_type in [RSA_SHA, AES_RSA_SHA]:
            v_id = self.mode2_verify(sec_image, sign_image_size,
                                     signature_offset, alg_data, key_list)
            for kl in key_list:
                if kl['TYPE'] == RSA_OEM and kl['ID'] == v_id:
                    print('Verify key ...')
                    print('Key Type: OEM DSS public keys')
                    print('ID: {}'.format(v_id))
                    print('M:')
                    hexdump.hexdump(kl['M'])
                    print('E:')
                    hexdump.hexdump(kl['E'])
                    break

            if alg_data.algorithm_type == AES_RSA_SHA:
                dec_image = self.mode2_decrypt(sec_image, sign_image_size,
                                               enc_offset, aes_data_offset,
                                               v_id, alg_data, key_list)
        else:
            dec_image, v_id = \
                self.modeGCM_verify_n_decrypt(sec_image, sign_image_size,
                                              signature_offset, enc_offset,
                                              aes_data_offset, key_list)
        return dec_image

    # def verify_cot_image(self, sec_image, header_offset):
    #     rot_cot_header = sec_image[header_offset:header_offset+0x8]
    #     (cot_info, data_offset) = struct.unpack(
    #         self.sec.COT_INFO_FORMAT, rot_cot_header)

    def verify_secure_image(self, soc_version, sec_image_fd, output_fd, otp_image_fd, cot_offset):
        otp_image = otp_image_fd.read()
        sec_image = sec_image_fd.read()
        data_region, config_region = self.parse_otp(otp_image)

        alg_data, header_offset, retire_list = self.parse_config(soc_version,
                                                                 config_region)

        key_list = self.parse_data(alg_data, data_region, retire_list)
        header = sec_image[header_offset:header_offset +
                           self.sec.ROT_HEADER_SIZE]
        (aes_data_offset, enc_offset, sign_image_size, signature_offset,
         revision_low, revision_high, reserved, bl1_header_checksum) = \
            struct.unpack(self.sec.ROT_HEADER_FORMAT, header)

        if((aes_data_offset + enc_offset + sign_image_size +
            signature_offset + revision_low + revision_high +
                reserved + bl1_header_checksum) & 0xffffffff) != 0:
            raise SecError("header checksum verify failed")
        if sign_image_size % 512:
            raise SecError("The sign_image_size should be 512 bytes aligned")

        print("check RoT header PASS")
        dec_image = self.verify_rot_image(
            sec_image[:sign_image_size+1024], header_offset, alg_data, key_list)
        print("check RoT integrity PASS")

        if dec_image != None and output_fd != None:
            print("write decrypt image to {}".format(output_fd.name))
            output_fd.write(bytes(dec_image + sec_image[sign_image_size:]))
            output_fd.flush()
            output_fd.close()

        if cot_offset == None:
            return

        # offset_list = cot_offset.split(':')


class secTool(object):
    """Object for sectool command-line tool."""

    def __init__(self):
        """Initializer method."""
        self.sec = Sec()
        self.verify = SecureBootVerify()

    def run(self, argv):
        """Command-line processor.

        Arguments:
            argv: Pass sys.argv from main.
        """
        parser = argparse.ArgumentParser()
        subparsers = parser.add_subparsers(title='subcommands')

        sub_parser = subparsers.add_parser('make_secure_bl1_image',
                                           help='Makes a signed bl1 image.')
        sub_parser.add_argument('--soc',
                                help='soc id (e.g. 2600, 1030)',
                                metavar='SOC',
                                default='2600')
        sub_parser.add_argument('--bl1_image',
                                help='Bootloader 1 Image (e.g. u-boot-spl.bin), which will be verified by soc',
                                type=argparse.FileType('rb'),
                                required=False)
        sub_parser.add_argument('--header_offset',
                                help='RoT header offsest',
                                type=parse_number,
                                default=None)
        sub_parser.add_argument('--rsa_sign_key',
                                help='Path to RSA private key file, which will use to sign BL1_IMAGE')
        sub_parser.add_argument('--gcm_aes_key',
                                type=argparse.FileType('rb'),
                                help='Path to aes private key file, which will use to sign BL1_IMAGE')
        sub_parser.add_argument('--output',
                                help='Output file name',
                                type=argparse.FileType('w+b'),
                                required=False)
        sub_parser.add_argument('--algorithm',
                                help='Algorithm to use (default: NONE e.g. AES_GCM, AES_RSA2048_SHA256, RSA2048_SHA256, ...), \
                                      RSA algo support RSA1024, RSA2048, RSA3072 and RSA4096, HASH algo support SHA224, SHA256, SHA384 and SHA512',
                                metavar='ALGORITHM',
                                default='NONE')
        sub_parser.add_argument('--rollback_index',
                                help='Rollback Index',
                                type=parse_number,
                                default=0)
        sub_parser.add_argument('--signing_helper',
                                help='Path to helper used for signing',
                                metavar='APP',
                                nargs='?',
                                default=None,
                                required=False)
        sub_parser.add_argument('--signing_helper_with_files',
                                help='Path to helper used for signing using files',
                                metavar='APP',
                                nargs='?',
                                default=None,
                                required=False)

        enc_group = sub_parser.add_argument_group(
            'enc_group', 'Enable aes encryption in mode 2')

        enc_group.add_argument('--enc_offset',
                               help='Offset where encryption start',
                               type=parse_number,
                               default=None)
        enc_group.add_argument('--aes_key',
                               help='Path to aes key file',
                               type=argparse.FileType('rb'),
                               nargs='?',
                               required=False)
        enc_group.add_argument('--key_in_otp',
                               help='aes key is storing in otp',
                               action='store_true',
                               required=False)
        enc_group.add_argument('--rsa_aes',
                               help='Path to RSA public key file, which is used to encrypt aes key',
                               nargs='?')

        cot_group = sub_parser.add_argument_group(
            'cot_group', 'Chain of trust argument')

        cot_group.add_argument('--cot_algorithm',
                               help='Algorithm to use (default: NONE e.g. RSA2048_SHA256), \
                                      RSA algo support RSA1024, RSA2048, RSA3072 and RSA4096, HASH algo support SHA224, SHA256, SHA384 and SHA512',
                               metavar='ALGORITHM',
                               nargs='?')
        cot_mut = cot_group.add_mutually_exclusive_group()
        cot_mut.add_argument('--cot_verify_key',
                             help='Path to RSA public key file, which will use to verify next chain image (BL2)',
                             nargs='?')
        cot_mut.add_argument('--cot_digest',
                             type=argparse.FileType('rb'),
                             help='Path to digest result of next chain image')

        sub_parser.set_defaults(func=self.make_secure_bl1_image)

        sub_parser = subparsers.add_parser('make_sv_chain_image',
                                           help='Makes a signature verified cot image.')
        sub_parser.add_argument('--algorithm',
                                help='Algorithm to use (default: NONE e.g. RSA2048_SHA256, RSA3072_SHA384, ...), \
                                      RSA algo support RSA1024, RSA2048, RSA3072 and RSA4096, HASH algo support SHA224, SHA256, SHA384 and SHA512',
                                metavar='ALGORITHM',
                                default='NONE')
        sub_parser.add_argument('--cot_part',
                                help='',
                                nargs='+',
                                metavar='BL2_IMAGE:BL2_OUT:BL2_SIGN_KEY:BL2_VERIFY_KEY BL3_IMAGE:BL3_OUT:BL3_SIGN_KEY:BL3_VERIFY_KEY')
        sub_parser.add_argument('--rollback_index',
                                help='Rollback Index',
                                type=parse_number,
                                default=0)
        sub_parser.add_argument('--image_relative_path',
                                help='Image relative path',
                                type=parse_path,
                                default='')
        sub_parser.add_argument('--signing_helper',
                                help='Path to helper used for signing',
                                metavar='APP',
                                nargs='?',
                                default=None,
                                required=False)
        sub_parser.add_argument('--signing_helper_with_files',
                                help='Path to helper used for signing using files',
                                metavar='APP',
                                nargs='?',
                                default=None,
                                required=False)
        sub_parser.set_defaults(func=self.make_sv_chain_image)

        sub_parser = subparsers.add_parser('verify',
                                           help='verify the image')
        sub_parser.add_argument('--soc',
                                help='soc id (e.g. 2600, 1030)',
                                metavar='SOC',
                                default='2600')
        sub_parser.add_argument('--sec_image',
                                help='Path to secure image',
                                type=argparse.FileType('rb'),
                                metavar='IMAGE',
                                required=True)
        sub_parser.add_argument('--output',
                                help='Output non-secure image',
                                type=argparse.FileType('w+b'),
                                metavar='IMAGE',
                                required=False)
        sub_parser.add_argument('--otp_image',
                                help='Path to otp image',
                                type=argparse.FileType('rb'),
                                metavar='IMAGE',
                                required=True)
        sub_parser.add_argument('--cot_offset',
                                help='Offset for every image, e.g. 0x10000:0x100000',
                                metavar='IMAGE',
                                default=None,
                                required=False)
        sub_parser.set_defaults(func=self.verify_secure_image)

        args = parser.parse_args(argv[1:])

        if(argv.__len__() == 1):
            parser.print_usage()
            sys.exit(1)
        args.func(args)

    def make_secure_bl1_image(self, args):
        """Implements the 'make_secure_bl1_image' sub-command."""
        self.sec.make_secure_bl1_image(args.soc,
                                       args.bl1_image, args.rsa_sign_key,
                                       args.gcm_aes_key,
                                       args.output, args.algorithm,
                                       args.header_offset,
                                       args.rollback_index,
                                       args.enc_offset,
                                       args.aes_key,
                                       args.rsa_aes,
                                       args.key_in_otp,
                                       args.cot_algorithm,
                                       args.cot_verify_key,
                                       args.cot_digest,
                                       args.signing_helper,
                                       args.signing_helper_with_files)

    def make_sv_chain_image(self, args):
        """Implements the 'make_sv_chain_image' sub-command."""
        self.sec.make_sv_chain_image(args.algorithm,
                                     args.cot_part,
                                     args.rollback_index,
                                     args.image_relative_path,
                                     args.signing_helper,
                                     args.signing_helper_with_files)

    def verify_secure_image(self, args):
        """Implements the 'verify' sub-command."""
        self.verify.verify_secure_image(args.soc,
                                        args.sec_image,
                                        args.output,
                                        args.otp_image,
                                        args.cot_offset)


if __name__ == '__main__':

    tool = secTool()
    tool.run(sys.argv)
