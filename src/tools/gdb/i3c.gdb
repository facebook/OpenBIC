set $I3C_BASE = 0x7e7a3000

define i3c_init
	i3c_reset
	shell sleep 1
	set *0x7e6e2010=0x1688a8a8
	x/xw 0x7e6e2f10
	x/xw 0x7e6e2010
	set *0x7e6e2054=0x00003f80
	set *0x7e6e2094=0x00003f00
	x/xw 0x7e6e2050
	x/xw 0x7e6e2090
	set *0x7e6e2438=0x000f3830
	x/xw 0x7e6e2438
	set *0x7e7a0014=0x000474c4
	set *0x7e7a0024=0x000474c4
	set *0x7e7a0034=0x000474c4
	set *($I3C_BASE + 0x000)=0x0000000
	set *($I3C_BASE + 0x004)=0x80778050
	set *($I3C_BASE + 0x008)=0x00000000
	set *($I3C_BASE + 0x010)=0x00000000
	set *($I3C_BASE + 0x018)=0x00000000
	set *($I3C_BASE + 0x01c)=0x04040200
	set *($I3C_BASE + 0x020)=0x00000101
	set *($I3C_BASE + 0x024)=0x00000001
	set *($I3C_BASE + 0x030)=0x00000000
	set *($I3C_BASE + 0x034)=0x00000000
	set *($I3C_BASE + 0x038)=0x00000002
	set *($I3C_BASE + 0x04c)=0x00000000
	set *($I3C_BASE + 0x050)=0x00000000
	set *($I3C_BASE + 0x054)=0x00000000
	set *($I3C_BASE + 0x058)=0x00000000
	set *($I3C_BASE + 0x05c)=0x00000000
	set *($I3C_BASE + 0x090)=0x00000000
	x/xw ($I3C_BASE + 0x050)
	set *($I3C_BASE + 0x0b0)=0x00000000
	set *($I3C_BASE + 0x0b4)=0x003d003d
	set *($I3C_BASE + 0x0b8)=0x003d003d
	set *($I3C_BASE + 0x0bc)=0x003d003d
	set *($I3C_BASE + 0x0c0)=0x003d003d
	set *($I3C_BASE + 0x0c8)=0x5b071611
	set *($I3C_BASE + 0x0cc)=0x0000000c
	set *($I3C_BASE + 0x0d4)=0x01f4003d
	set *($I3C_BASE + 0x0d8)=0x00000020
	set *($I3C_BASE + 0x0e0)=0x00000000
	set *($I3C_BASE + 0x0e4)=0x00000000
	set *($I3C_BASE + 0x0e8)=0x00000000
	set *($I3C_BASE + 0x040)=0x00003b5b
	set *($I3C_BASE + 0x044)=0x00003b5b
	x/xw ($I3C_BASE + 0x040)
	x/xw ($I3C_BASE + 0x044)
	set *($I3C_BASE + 0x040)=0x0000333c
	x/xw ($I3C_BASE + 0x040)
	set *($I3C_BASE + 0x000)=0x00000000
	set *($I3C_BASE + 0x000)=0x90000000
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	x/xw ($I3C_BASE + 0x060)
	set *($I3C_BASE + 0x280)=0x00510051
	x/xw ($I3C_BASE + 0x280)
	#printf "set 0xc\n"
	#set *($I3C_BASE + 0x00c)=0x4420c383
	#x/xw ($I3C_BASE + 0x010)
	set *($I3C_BASE + 0x000)=0x40000000
	set *($I3C_BASE + 0x000)=0x80000000
end

define rstdaa
	set $ccc = 0x06
	#set $value = 0x4420c380 & 0xffff807f
	set $value = 0x4400c380 & 0xffff807f
	set $value = $value | ($ccc << 7)
	set *($I3C_BASE + 0x00c)=1
	set *($I3C_BASE + 0x00c)=$value
	printf "set value %08x, ccc=%02x\n", $value, $ccc
	x/xw ($I3C_BASE + 0x010)
	x/xw ($I3C_BASE + 0x03c)
end

define setaasa
	set $ccc = 0x29
	#set $value = 0x4420c380 & 0xffff807f
	set $value = 0x4400c380 & 0xffff807f
	set $value = $value | ($ccc << 7)
	set *($I3C_BASE + 0x00c)=1
	set *($I3C_BASE + 0x00c)=$value
	printf "set value %08x, ccc=%02x\n", $value, $ccc
	x/xw ($I3C_BASE + 0x010)
	x/xw ($I3C_BASE + 0x03c)
end

define i3c_resume
	set *($I3C_BASE + 0x000)=0xc0000000
	x/xw ($I3C_BASE + 0x000)
end

#set *($I3C_BASE + 0x014)=0xaabbccdd
#set *($I3C_BASE + 0x014)=0xaabbccdd
#set *($I3C_BASE + 0x014)=0x000000aa
#set *($I3C_BASE + 0x00c)=0x00090001
#set *($I3C_BASE + 0x014)=0x44008400
#x/xw ($I3C_BASE + 0x010)