import os, sys, json, serial, readline, glob, time
import platform
import traceback
import progressbar

def serial_ports():
    """ Lists serial port names
 
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
 
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

readline.set_completer_delims(' \t\n=')
readline.parse_and_bind('tab: complete')
readline.parse_and_bind('set editing-mode vi')

ports_list = serial_ports()
serial_num = len(ports_list)

def read_until(ser, end_char):
    msg = ""
    while True:
        c = ser.read(1)
        if c:
            msg += c
            if msg[-1:] == end_char:
                break
    if len(msg) > 1:
        print msg
    if "ERROR" in msg:
        sys.exit()
    return msg

while True:
    print "-----------------------"
    for index, port in enumerate(ports_list):
        print "[{}]: {}".format(index, port)
    print "[>={}]: exit".format(len(ports_list))
    print "-----------------------"
    try:
        port_num = int(raw_input("Please select the interface connects with BMC:"))
    except ValueError:
        print("Not an integer! Try again.")
        continue
    if port_num < serial_num:
        dev = ports_list[port_num]
        baudrate = 115200
        ast_ser = serial.Serial(dev, 115200, timeout=0.2)
        break
    else:
        sys.exit()


while True:
    try:
        file_name = raw_input("Please enter the svf file name:")
        open_file = open(file_name, "r")
        break
    except KeyboardInterrupt:
        sys.exit()
    except Exception as e:
        print(e)
    
svf = open_file.readlines()
open_file.close()

progress = progressbar.ProgressBar(widgets=[
    ' [', progressbar.Timer(), '] ',
    progressbar.Percentage(),
    progressbar.Bar(),
    ' (', progressbar.ETA(), ') ',
])
try:
    for i in progress(range(len(svf))):
        ast_ser.write(svf[i])
        ast_ser.flush()
        if ';' in svf[i]:
            read_until(ast_ser, '>')
        else:
            time.sleep(0.2)
except KeyboardInterrupt:
    sys.exit()