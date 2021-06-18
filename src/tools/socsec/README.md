# AST2600 Secure Boot

AST2600 support two root of trust (RoT) measurement modes: `Mode_2` & `Mode_GCM` (`Mode_1` has been eliminated). The following chapter will introduce the hardware RoT and the software chain of trust (CoT)

## SOCSEC

This tool is used to generate ast2600 secure boot RoT and CoT image.

### Usage

#### Setting Up

```bash
sudo apt-get install python3 python3-pip python3-virtualenv
virtualenv .venv
source .venv/bin/activate
pip3 install -r requirements.txt
python3 setup.py install
```

#### Key Generation

* RSA private key

```bash
openssl genrsa -out rsa_key.pem 4096
```

* RSA public key

```bash
openssl rsa -in rsa_key.pem -pubout -out rsa_key-public.pem
```

* AES key

```bash
openssl rand 32 > aes_key.bin
```

#### How to use

* RoT image generating command

```bash
usage: socsec make_secure_bl1_image [-h] [--soc SOC] [--bl1_image BL1_IMAGE]
                                    [--header_offset HEADER_OFFSET]
                                    [--rsa_sign_key RSA_SIGN_KEY]
                                    [--rsa_key_order ORDER]
                                    [--gcm_aes_key GCM_AES_KEY]
                                    [--output OUTPUT] [--algorithm ALGORITHM]
                                    [--rollback_index ROLLBACK_INDEX]
                                    [--signing_helper [APP]]
                                    [--signing_helper_with_files [APP]]
                                    [--enc_offset ENC_OFFSET]
                                    [--aes_key [AES_KEY]] [--key_in_otp]
                                    [--rsa_aes [RSA_AES]]
                                    [--cot_algorithm [ALGORITHM]]
                                    [--cot_verify_key [COT_VERIFY_KEY] |
                                    --cot_digest COT_DIGEST]

optional arguments:
  -h, --help            show this help message and exit
  --soc SOC             soc id (e.g. 2600, 1030)
  --bl1_image BL1_IMAGE
                        Bootloader 1 Image (e.g. u-boot-spl.bin), which will
                        be verified by soc
  --header_offset HEADER_OFFSET
                        RoT header offsest
  --rsa_sign_key RSA_SIGN_KEY
                        Path to RSA private key file, which will use to sign
                        BL1_IMAGE
  --rsa_key_order ORDER
                        This value the OTP setting(e.g. little, big), default
                        value is "little"
  --gcm_aes_key GCM_AES_KEY
                        Path to aes private key file, which will use to sign
                        BL1_IMAGE
  --output OUTPUT       Output file name
  --algorithm ALGORITHM
                        Algorithm to use (default: NONE e.g. AES_GCM,
                        AES_RSA2048_SHA256, RSA2048_SHA256, ...), RSA algo
                        support RSA1024, RSA2048, RSA3072 and RSA4096, HASH
                        algo support SHA224, SHA256, SHA384 and SHA512
  --rollback_index ROLLBACK_INDEX
                        Rollback Index
  --signing_helper [APP]
                        Path to helper used for signing
  --signing_helper_with_files [APP]
                        Path to helper used for signing using files

enc_group:
  Enable aes encryption in mode 2

  --enc_offset ENC_OFFSET
                        Offset where encryption start
  --aes_key [AES_KEY]   Path to aes key file
  --key_in_otp          aes key is storing in otp
  --rsa_aes [RSA_AES]   Path to RSA public key file, which is used to encrypt
                        aes key

cot_group:
  Chain of trust argument

  --cot_algorithm [ALGORITHM]
                        Algorithm to use (default: NONE e.g. RSA2048_SHA256),
                        RSA algo support RSA1024, RSA2048, RSA3072 and
                        RSA4096, HASH algo support SHA224, SHA256, SHA384 and
                        SHA512
  --cot_verify_key [COT_VERIFY_KEY]
                        Path to RSA public key file, which will use to verify
                        next chain image (BL2)
  --cot_digest COT_DIGEST
                        Path to digest result of next chain image
```

* CoT image generating command

```bash
usage: socsec make_sv_chain_image [-h] [--algorithm ALGORITHM]
                                  [--rsa_key_order ORDER]
                                  [--cot_part BL2_IMAGE:BL2_OUT:BL2_SIGN_KEY:BL2_VERIFY_KEY BL3_IMAGE:BL3_OUT:BL3_SIGN_KEY:BL3_VERIFY_KEY [BL2_IMAGE:BL2_OUT:BL2_SIGN_KEY:BL2_VERIFY_KEY BL3_IMAGE:BL3_OUT:BL3_SIGN_KEY:BL3_VERIFY_KEY ...]]
                                  [--rollback_index ROLLBACK_INDEX]
                                  [--image_relative_path IMAGE_RELATIVE_PATH]
                                  [--signing_helper [APP]]
                                  [--signing_helper_with_files [APP]]

optional arguments:
  -h, --help            show this help message and exit
  --algorithm ALGORITHM
                        Algorithm to use (default: NONE e.g. RSA2048_SHA256,
                        RSA3072_SHA384, ...), RSA algo support RSA1024,
                        RSA2048, RSA3072 and RSA4096, HASH algo support
                        SHA224, SHA256, SHA384 and SHA512
  --rsa_key_order ORDER
                        This value the OTP setting(e.g. little, big), default
                        value is "little"
  --cot_part BL2_IMAGE:BL2_OUT:BL2_SIGN_KEY:BL2_VERIFY_KEY BL3_IMAGE:BL3_OUT:BL3_SIGN_KEY:BL3_VERIFY_KEY [BL2_IMAGE:BL2_OUT:BL2_SIGN_KEY:BL2_VERIFY_KEY BL3_IMAGE:BL3_OUT:BL3_SIGN_KEY:BL3_VERIFY_KEY ...]
  --rollback_index ROLLBACK_INDEX
                        Rollback Index
  --image_relative_path IMAGE_RELATIVE_PATH
                        Image relative path
  --signing_helper [APP]
                        Path to helper used for signing
  --signing_helper_with_files [APP]
                        Path to helper used for signing using files
```

Here is an example.
There are two stages to make the whole secure image, make RoT image and make CoT image.

* RoT secure image with mode 2 RSA4096_SHA512

```bash
socsec make_secure_bl1_image \
    --algorithm RSA4096_SHA512 \
    --bl1_image path/to/u-boot-spl.bin \
    --output path/to/s_u-boot-spl.bin \
    --rsa_sign_key path/to/test_oem_dss_private_key_4096_1.pem \
    --cot_algorithm RSA4096_SHA512 \
    --cot_verify_key path/to/test_bl2_public_4096.pem \
```

* CoT secure image

```bash
socsec make_sv_chain_image \
--algorithm RSA4096_SHA512 \
--image_relative_path path/to/all_image/ \
--cot_part u-boot.bin:s_u-boot.bin:test_bl2_private_4096.pem:test_bl2_public_4096.pem \
ast2600-ramfs.itb:s_ast2600-ramfs.itb: test_bl3_private_4096.pem: test_bl3_public_4096.pem \
```

The format of `--cot_part` option is `image_input:secure_image_output:sign_key:verification_key`

The `--signing_helper` option can be used to specify any external program for signing hashes. The data to sign (including padding e.g. PKCS1-v1.5) is fed via `STDIN` and the signed data is returned via STDOUT. Arguments for a signing helper is `--rsa_sign_key` in `make_secure_bl1_image` or `sign_key` in `make_sv_chain_image --cot_part`. If the signing helper exits with a non-zero exit code, it means failure.
Here's an example invocation:

```bash
/path/to/my_signing_program /path/to/privatekey.pem
```

`--signing_helper_with_files` is similar to `--signing_helper` except that a temporary file is used to communicate with the helper instead of STDIN and STDOUT. This is useful in situations where the signing helper is using code which is outputting diagnostics on STDOUT instead of STDERR. Here's an example invocation

```bash
/path/to/my_signing_program_with_files \
/path/to/privatekey.pem /tmp/path/to/communication_file
```

## OTP Tool

AST2600 built-in 64Kbit one time programmable (OTP) memory for configuration, strap, key storage, patch and user data. Each memory bit cell inside the OTP memory is capable to be programmed once. Typically, the data stored the OTP memory are non-volatile and can preserve permanently, but to improve the FIT (failure in time) of the OTP memory, ECC is recommended to enable.

### Usage

Using this tool to generate the otp image, and using OTP Utility to program that image into OTP memory.

```bash
usage: otptool [-h] [--key_folder KEY_FOLDER]
               [--user_data_folder USER_DATA_FOLDER]
               [--output_folder OUTPUT_FOLDER]
               config

positional arguments:
  config                configuration file

optional arguments:
  -h, --help            show this help message and exit
  --key_folder KEY_FOLDER
                        key folder
  --user_data_folder USER_DATA_FOLDER
                        user data folder
  --output_folder OUTPUT_FOLDER
                        output folder
```

#### Argument

* `config`: the config file is a json format document, which content otp data region, otp config region and otp strap description. Below is an example.

* `key_folder`: put all key file into key folder

* `output_folder`: the generated otp image will put into this folder.

#### Output

* `otp-all.image`: a programmable image, use u-boot otp utility to program this image, which contain all region of otp.

* `*.image`: a programmable image, use u-boot otp utility to program this image
configs/ast2600/security/otp/sample.json is an example for all otp config and otp strap.

`data_region` object is to describe the otp data region, which contain key, user data, otp patch, and ecc code. The figure below is data region layout. When `ecc_region` enable, otp tool will generate the ECC code. The otp patch should put inside user data region (non-secure region).
