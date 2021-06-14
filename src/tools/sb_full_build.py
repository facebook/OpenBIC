#!/usr/bin/env python3

import sys
import os
import subprocess

MAIN_CONFIG = "\
CONFIG_OTP_MEM = y\n\
CONFIG_SB_ENABLE = y\n\
CONFIG_OTP_KEY_FOLDER = \"configs/key\"\n\
CONFIG_OTP_USER_DATA_FOLDER = \"configs/data\"\n\
"

OTP_CONFIG = "CONFIG_OTP_CONFIG = \"configs/otp_config/{}\"\n"

MODE_2 = "CONFIG_SB_MODE_2 = y\n"
MODE_GCM = "CONFIG_SB_MODE_GCM = y\n"

MODE_2_AES = "CONFIG_SB_MODE_2_AES = y\n"
MODE_2_OPT = "CONFIG_SB_MODE_2_O{} = y\n"
SB_AES_KEY = "CONFIG_SB_AES_KEY = \"configs/key/test_aes_key.bin\"\n"
SB_RSA_AES_KEY = "CONFIG_SB_RSA_AES_KEY = \"configs/key/{}\"\n"

RSA_SET = "CONFIG_SB_RSA_{} = y\n"
SHA_SET = "CONFIG_SB_SHA_{} = y\n"

SB_GCM_AES_KEY0 = ["CONFIG_SB_GCM_AES_KEY = \"configs/key/test_gcm_aes_key_0.pem\"\n",
                   "CONFIG_SB_GCM_AES_KEY = \"configs/key/test_gcm_aes_key_2.pem\"\n",
                   "CONFIG_SB_GCM_AES_KEY = \"configs/key/test_gcm_aes_key_1.pem\"\n"]

SB_RSA_KEY = ["CONFIG_SB_RSA_KEY = \"configs/key/test_oem_dss_private_key_{}_0.pem\"\n",
              "CONFIG_SB_RSA_KEY = \"configs/key/test_oem_dss_private_key_{}_1.pem\"\n",
              "CONFIG_SB_RSA_KEY = \"configs/key/test_oem_dss_private_key_{}_2.pem\"\n"]

MODE_2_LIST = [
    {
        "name": "RSA2048_SHA256",
        "encrypt": False,
        "rsa": 2048,
        "sha": 256,
        "otp": "mini_RSA2048_SHA256.json"
    },
    {
        "name": "RSA2048_SHA256_o1",
        "encrypt": True,
        "option": 1,
        "rsa": 2048,
        "sha": 256,
        "otp": "mini_RSA2048_SHA256_o1.json"
    },
    {
        "name": "RSA2048_SHA256_o2_pub",
        "encrypt": True,
        "option": 2,
        "rsa_aes": "test_soc_private_key_2048.pem",
        "rsa": 2048,
        "sha": 256,
        "otp": "mini_RSA2048_SHA256_o2_pub.json"
    },
    {
        "name": "RSA2048_SHA256_o2_priv",
        "encrypt": True,
        "option": 2,
        "rsa_aes": "test_soc_public_key_2048.pem",
        "rsa": 2048,
        "sha": 256,
        "otp": "mini_RSA2048_SHA256_o2_priv.json"
    },
    {
        "name": "RSA3072_SHA384",
        "encrypt": False,
        "rsa": 3072,
        "sha": 384,
        "otp": "mini_RSA3072_SHA384.json"
    },
    {
        "name": "RSA3072_SHA384_o1",
        "encrypt": True,
        "option": 1,
        "rsa": 3072,
        "sha": 384,
        "otp": "mini_RSA3072_SHA384_o1.json"
    },
    {
        "name": "RSA3072_SHA384_o2_pub",
        "encrypt": True,
        "option": 2,
        "rsa_aes": "test_soc_private_key_3072.pem",
        "rsa": 3072,
        "sha": 384,
        "otp": "mini_RSA3072_SHA384_o2_pub.json"
    },
    {
        "name": "RSA3072_SHA384_o2_priv",
        "encrypt": True,
        "option": 2,
        "rsa_aes": "test_soc_public_key_3072.pem",
        "rsa": 3072,
        "sha": 384,
        "otp": "mini_RSA3072_SHA384_o2_priv.json"
    },
    {
        "name": "RSA4096_SHA512",
        "encrypt": False,
        "rsa": 4096,
        "sha": 512,
        "otp": "mini_RSA4096_SHA512.json"
    },
    {
        "name": "RSA4096_SHA512_o1",
        "encrypt": True,
        "option": 1,
        "rsa": 4096,
        "sha": 512,
        "otp": "mini_RSA4096_SHA512_o1.json"
    },
    {
        "name": "RSA4096_SHA512_o2_pub",
        "encrypt": True,
        "option": 2,
        "rsa_aes": "test_soc_private_key_4096.pem",
        "rsa": 4096,
        "sha": 512,
        "otp": "mini_RSA4096_SHA512_o2_pub.json"
    },
    {
        "name": "RSA4096_SHA512_o2_priv",
        "encrypt": True,
        "option": 2,
        "rsa_aes": "test_soc_public_key_4096.pem",
        "rsa": 4096,
        "sha": 512,
        "otp": "mini_RSA4096_SHA512_o2_priv.json"
    },
]

root_dir = ""


def run_shell(cmd):
    print(cmd)
    my_env = os.environ.copy()
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, cwd=root_dir, env=my_env)
    p.communicate()
    return p.returncode


if __name__ == "__main__":
    # defconfig = "fpga-ast1030_defconfig"
    root_dir = sys.argv[1]

    # check root dir
    print('checking root dir ...')
    ret = run_shell('make check')
    if ret != 0:
        print('root dir is wrong')
        raise TypeError
    print('make ...')
    run_shell('mkdir -p ./sb_bin')
    # run_shell('make {}'.format(defconfig))
    run_shell('make')
    print('make done')
    run_shell('cp .config .config_backup')

    with open('{}/.config'.format(root_dir), 'r+') as conf:
        defconfig_list = conf.readlines()

    for mode2 in MODE_2_LIST:
        defconfig_case = defconfig_list.copy()
        otp_conf = '\"configs/otp_config/{}\"'.format(mode2['otp'])
        dest_folder = './sb_bin/{}'.format(mode2['name'])

        run_shell('mkdir -p ./sb_bin/{}'.format(mode2['name']))
        run_shell('cp {} {}'.format(otp_conf, dest_folder))
        defconfig_case.append(MAIN_CONFIG)
        defconfig_case.append(OTP_CONFIG.format(mode2['otp']))
        defconfig_case.append(MODE_2)
        defconfig_case.append(RSA_SET.format(mode2['rsa']))
        defconfig_case.append(SHA_SET.format(mode2['sha']))

        if mode2['encrypt']:
            defconfig_case.append(MODE_2_AES)
            defconfig_case.append(SB_AES_KEY)
            defconfig_case.append(MODE_2_OPT.format(mode2['option']))
            if mode2['option'] == 2:
                defconfig_case.append(SB_RSA_AES_KEY.format(mode2['rsa_aes']))

        i = 0
        print('start ...')
        for kid in SB_RSA_KEY:

            sec_res = 'sec_ast1030_bic{}.bin'.format(i)
            uart_res = 'sec_uart_ast1030_bic{}.bin'.format(i)
            i = i + 1
            d = defconfig_case.copy()
            d.append(kid.format(mode2['rsa']))
            with open('{}/.config'.format(root_dir), 'w') as conf:
                conf.writelines(d)
            if run_shell('make secure_boot') != 0:
                raise TypeError
            run_shell(
                'cp ./bin/sec_ast1030_bic.bin {}/{}'.format(dest_folder, sec_res))
            run_shell(
                'cp ./bin/uart_sec_ast1030_bic.bin {}/{}'.format(dest_folder, uart_res))
            run_shell('cp .config {}/'.format(dest_folder))
            run_shell('cp ./bin/otp/otp-all.image {}/'.format(dest_folder))

    run_shell('mv .config_backup .config')
