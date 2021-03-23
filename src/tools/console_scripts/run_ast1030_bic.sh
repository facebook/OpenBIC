#!/bin/bash
# ------------------------------------------------------------------------
# usage: 
# 1. download bin-xfer from https://gist.github.com/cstrahan/5796653
# 2. sudo chmod +x /usr/bin/bin-xfer.sh
# 3. modify variables 'DEV' and 'FILE' according to yout local envrionment
# 4. run 'sh run_ast1030_bic.sh'
# ------------------------------------------------------------------------
DEV=/dev/ttyS12
FILE=/mnt/d/tftpboot/uart_ast1030_bic.bin

stty -F $DEV 115200 raw cs8 -ixoff -cstopb -parenb
bin-xfer.sh -i $FILE -o $DEV
