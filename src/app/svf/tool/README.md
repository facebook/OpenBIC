## SVF player usage
- Setup 1030 to svf player status
```
minibmc>jtag init
minibmc>svf
svf>
```
- close com port
- Run svf player tool
```
$>./exe/svf_player
-----------------------
[0]: /dev/ttyS1
[1]: /dev/ttyS2
[2]: /dev/ttyS3
[3]: /dev/ttyS7
[>=4]: exit
-----------------------
Please select the interface connects with BMC: 0 [dev connect to minibmc uart]
> svf_example/program.svf
```
