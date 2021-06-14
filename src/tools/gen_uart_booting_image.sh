#!/bin/sh

print_usage() {
    echo "$0 <src_img> <dst_img>"
    exit 1
}

# check parameter
if [ $# -ne 2 ]
then
    print_usage
fi

src="$1"
dst="$2"

# check src existence
if [ -t "$src" ]
then
    echo "$src: No such file"
    exit 1
fi

# get src size and round up to 4-byte align
src_sz=`wc -c < $src`
src_sz_align=$(( ((${src_sz} + 3) / 4) * 4 ))

# output size header
printf "0: %.8x" $src_sz_align | sed -E 's/0: (..)(..)(..)(..)/0: \4\3\2\1/' | xxd -r -g0 > $dst

# output src img
dd if=$src of=$dst bs=1 seek=4

# output zero padding
dd if=/dev/zero bs=1 count=$(( ${src_sz_align} - ${src_sz} )) >> $dst
