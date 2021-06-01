#!/bin/bash
top="$( pwd )"
src=$top/release
if [ ! -d $src ] 
then
	git clone ssh://git@192.168.10.30:7999/bmc/minibmc.git $src
fi
cd $src
git checkout master
git submodule init
git submodule update
if git rev-parse v$TAG_ID >/dev/null 2>&1
then
    git checkout v$TAG_ID
else
    git tag v$TAG_ID
fi
revision="$(date '+%Y%m%d')"_v$TAG_ID
cd $top
dest=$top/release_$revision

cp -r $src $dest
rm -rf $dest/app
rm -rf $dest/drivers

app_list=('main.c' 'cmd_clk.c' 'cmd_mem.c' 'demo_i2c.c' 'demo_i3c.c' 'demo_adc.c' 'demo_pwm_tach.c' 'demo_espi.c' 'Makefile' 'Kconfig')
mkdir -p $dest/app

for app in $app_list; do
	if [ -f "$app" ];then
		cp $src/app/$app $dest/app/$app
	else
		cp -r $src/app/$app $dest/app/$app
	fi
done

drv_list=('cache' 'clk' 'i2c' 'i3c' 'irq_aspeed.c' 'reset_aspeed.c' 'serial' 'timer' 'adc' 'pwm_tach' 'espi' 'Makefile' 'Kconfig')
mkdir -p $dest/drivers

for drv in $drv_list; do
	if [ -f "$drv" ];then
		cp $src/drivers/$drv $dest/drivers/$drv
	else
		cp -r $src/drivers/$drv $dest/drivers/$drv
	fi
done

inc_list=('buffer.h' 'cache_aspeed.h' 'clk_aspeed.h' 'dma_api.h' 'hal_def.h' 
'i2c_aspeed.h' 'i3c_api.h' 'i3c_aspeed.h' 'irq_aspeed.h' 'objects.h' 'pinmap.h' 
'reset_aspeed.h' 'serial.h' 
'serial_api.h' 'timer_aspeed.h' 'uart_aspeed.h' 'wait.h' 'watchdog_api.h' 'ipi_aspeed.h' 
'pwm_tach_aspeed.h' 'adc_aspeed.h' 'espi_aspeed.h')
mkdir -p $dest/drivers/include

for drv in $inc_list; do
	if [ -f "$drv" ];then
		cp $src/drivers/include/$drv $dest/drivers/include/$drv
	else
		cp -r $src/drivers/include/$drv $dest/drivers/include/$drv
	fi
done

rm -rf .vscode
rm -f .tags

rm -f $dest/arch/arm/aspeed/ast1030/*.c
rm -f $dest/arch/arm/aspeed/ast1030/Makefile
rm -rf $dest/arch/arm/aspeed/ast1030/gcc
rm -rf $dest/arch/arm/aspeed/ast1030/include

rm -f $dest/board/fpga-ast1030/Makefile
rm -f $dest/board/fpga-ast1030/*.*
rm -f $dest/board/fpga-ast2600/Makefile
rm -f $dest/board/fpga-ast2600/*.*

rm -f $dest/board/evb-ast1030/Makefile
rm -f $dest/board/evb-ast1030/*.*
rm -f $dest/board/evb-ast2600/Makefile
rm -f $dest/board/evb-ast2600/*.*

rm -f $dest/configs/evb-ast1030_defconfig
rm -f $dest/configs/fpga-ast1030_defconfig
rm -f $dest/debug_1030.gdb


rm -rf $dest/tools/socsec
rm -f $dest/tools/sb_full_build.sh
rm -f $dest/tools/release.sh

#
#make evb-ast2600a1_defconfig  &> ${revision}.log
#make &>> ${revision}.log
#make clean
#cd ..
#zip -r ${revision}.zip release
#
find $dest -type f -iname '.git*' | xargs rm -f
find $dest -type d -iname '.git*' | xargs rm -rf
