"""
Created on Tue Nov 22 09:42:43 2022

@author: Mouchen
@discription: common code
@note: none
"""

import sys, os, time, hashlib, platform

class System_ctrl:
    def __init__(self):
        # OS
        self.os_name = platform.system()

    def get_os(self):
        return self.os_name

class Common_msg:
    def __init__(self):
        # OS
        self.os_name = platform.system()

        # Color
        self.color_purple = '\033[95m'
        self.color_blue = '\033[94m'
        self.color_cyan = '\033[96m'
        self.color_lightgray = '\033[97m'
        self.color_green = '\033[92m'
        self.color_yellow = '\033[93m'
        self.color_red = '\033[91m'
        self.ENDC = '\033[0m'
        self.BOLD = '\033[1m'
        self.UNDERLINE = '\033[4m'

        # Header
        self.hdr_sys = "<system> "
        self.hdr_wrn = "<warn>   "
        self.hdr_err = "<error>  "
        self.hdr_wdt = "<wdt>    "

    def msg_hdr_print(self, hdr_type, msg, pre_msg="", end_msg="\n"):
        if hdr_type == "s":     #system
            header = self.hdr_sys
        elif hdr_type == "w":   #warning
            header = self.hdr_wrn
        elif hdr_type == "e":   #error
            header = self.hdr_err
        elif hdr_type == "wdt": #watchdog
            header = self.hdr_wdt
        elif hdr_type == "n": #none
            header = ""
        elif hdr_type == "c": #none with space front
            header = ""
            pre_msg = "         "
        print(pre_msg + header + msg, end=end_msg)

    def msg_color_print(self, msg, color, end_msg="\n"):
        if self.os_name == "linux":
            print(color + msg + self.ENDC, end=end_msg)
        else:
            print(msg, end=end_msg)

class Common_file:
    def __init__(self):
        self.comm_time = Common_time()
        pass

    def resource_path(self, relative_path):
        """ Get absolute path to resource, works for dev and for PyInstaller """
        try:
            # PyInstaller creates a temp folder and stores path in _MEIPASS
            base_path = sys._MEIPASS
        except Exception:
            base_path = os.path.abspath(".")

        return os.path.join(base_path, relative_path)

    def list_dump(self, list, pre_msg=""):
        print(pre_msg + "[", end="")
        for i in range(len(list)):
            if i == len(list)-1:
                print(list[i].replace("0x", ""), end="")
            else:
                print(list[i].replace("0x", ""), end=", ")
        print("]")

    def is_file_exist(self, file_path):
        if not os.path.exists(file_path):
            return False
        else:
            return True

    def is_binary(self, file_path):
        try:
            with open(file_path, 'tr') as check_file:
                check_file.read()
                return 0
        except:
            return os.path.getsize(file_path)

    def byte_to_k(self, byte_num):
        return round(byte_num/1024)

    def gen_bytes(self, num, pad_size, pad_val_str, mode):
        hex_str = str(hex(num))
        hex_str = hex_str.replace("0x", "")

        output = []
        if not pad_size:
            return output

        if ( len(hex_str) ) > pad_size*2:
            return output

        # padding
        if len(hex_str) != pad_size*2:
            hex_str = pad_val_str*(pad_size*2 - len(hex_str)) + hex_str

        if mode == "msb":
            for i in range( 0, len(hex_str), 2 ):
                output.append( int(hex_str[i]+hex_str[i+1], 16) )
        elif mode == "lsb":
            for i in range( len(hex_str)-1, 0, -2 ):
                output.append( int(hex_str[i-1]+hex_str[i], 16) )

        return output

    def check_hex_valid(self, hex_str):
        if int(hex_str, 16) > 0xff or int(hex_str, 16) < 0:
            return 1
        return 0

    def list_rm(self, list, idx=-1):
        if (len(list) == 0):
            return list

        if idx == -1:
            rm_idx = len(list) - 1
        else:
            if idx < 0 or idx > len(list)-1:
                print("Invalid index given")
                return list
            rm_idx = idx

        new_lst = []
        for i in range(len(list)):
            if i == rm_idx:
                continue
            new_lst.append(list[i])

        return new_lst

    def get_md5_str_from_file(self, file_path) -> str:
        hash_md5 = hashlib.md5()

        with open(file_path, "rb") as image:
            for chunk in iter(lambda: image.read(4096), b""):
                hash_md5.update(chunk)

        return hash_md5.hexdigest()

    def get_md5_str_from_file_list(self, file_path_list) -> str:
        hash_md5 = hashlib.md5()

        for file_path in file_path_list:
            with open(file_path, "rb") as image:
                for chunk in iter(lambda: image.read(4096), b""):
                    hash_md5.update(chunk)

        return hash_md5.hexdigest()

    def get_md5_hex_from_file(self, file_path_list):
        comb_data = []
        md5str = self.get_md5_str_from_file_list(file_path_list)
        for i in range( 0, 32, 2 ):
            comb_data.append( int(md5str[i] + md5str[i+1], 16) )

        return comb_data

    def add_md5_to_end_of_file(self, output_img_path):
        comb_data = []
        md5str = self.get_md5_str_from_file(output_img_path)
        for i in range( 0, 32, 2 ):
            comb_data.append( int(md5str[i] + md5str[i+1], 16) )

        with open(output_img_path, 'ab') as f:
            f.write(bytearray(comb_data))

    def file_wr(self, file_path, file_type, flag, contents):
        if file_type == "txt":
            with open(file_path, flag) as f:
                f.write(contents)
        else:
            print("flag ", flag, " not support yet!")

    def log_record(self, file_path, op, contents=""):
        if op == "init":
            self.file_wr(file_path, "txt", 'w', contents)
        elif op == "append":
            self.file_wr(file_path, "txt", 'a', "\n[" + self.comm_time.get_time(0) + "]\n")
            self.file_wr(file_path, "txt", 'a', contents)
        else:
            print("op ", op, " not support yet!")

class Common_time:
    def __init__(self):
        pass
        
    def get_time(self, mode):
        TIME = ""
        if mode==0:
            TIME = time.strftime("%Y/%m/%d %H:%M:%S", time.localtime())
        elif mode==1:
            TIME = time.strftime("%d/%m/%Y %H:%M:%S", time.localtime())
        elif mode==2:
            TIME = time.strftime("%Y%m%d", time.localtime())
        elif mode ==3:
            TIME = time.strftime("%H%M%S", time.localtime())
        elif mode ==4:
            TIME = time.strftime("%a", time.localtime())
            if TIME == "一":
                TIME = "Mon"
            elif TIME == "二":
                TIME = "Tue"
            elif TIME == "三":
                TIME = "Wed"
            elif TIME == "四":
                TIME = "Tur"
            elif TIME == "五":
                TIME = "Fri"
            elif TIME == "六":
                TIME = "Sat"
            elif TIME == "日":
                TIME = "Sun"
        else:
            TIME = "--> Can't mach with given time mode!"
        return TIME
