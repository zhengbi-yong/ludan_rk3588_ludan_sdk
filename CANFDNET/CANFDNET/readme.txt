1. 安装驱动
	sudo apt-get update
	sudo apt-get install can-utils
	
	sudo modprobe can-dev
	sudo insmod usbcanfd.ko

2. 检查硬件是否枚举成功
	lsusb
	...04cc:1240...
	...3068:0009...

3. 检查硬件通道节点
	sudo ls /sys/class/net | grep can
	sudo ls /sys/class/net | grep lin

4. 设置LIN通道配置
	sudo echo '{"LIN0":{"Enable":1,"IsMaster":1,"Baudrate":19200,"Feature":1,"TrEnable":1,"DataLen":8}}' > `ls /sys/bus/usb/drivers/usbcanfd/*/lin0_cfg`
	sudo echo '{"LIN1":{"Enable":1,"IsMaster":0,"Baudrate":19200,"Feature":1,"TrEnable":0,"DataLen":8}}' > `ls /sys/bus/usb/drivers/usbcanfd/*/lin1_cfg`

5. 设置通道，并启动通道
	设置通道参数说明
	sudo ip link set can0 up type can help
	
	设置通道参数
	sudo ip link set can0 type can fd on bitrate 500000 dbitrate 2000000 sample-point 0.8 dsample-point 0.8
	sudo ip link set can1 type can fd on bitrate 500000 dbitrate 2000000 sample-point 0.8 dsample-point 0.8
	sudo ip link set lin0 up type can fd on bitrate 1000000 dbitrate 1000000
	sudo ip link set lin1 up type can fd on bitrate 1000000 dbitrate 1000000
	
	启动通道
	sudo ifconfig can0 up
	sudo ifconfig can1 up
	sudo ifconfig lin0 up
	sudo ifconfig lin1 up
	
	获取通道配置
	sudo ip -details link show can0

6. 接收报文（单独开一个终端控制台）
	candump any						接收所有通道 -c
	candump can0					接收单一通道
	candump -e any,0:0,#FFFFFFFF 	接收错误帧和正常报文

7. 发送
	CAN报文
	cansend can0 123#11.22.33.44.55.66.77.88   			CAN0口发送ID为123的8字节CAN标准帧；
	cansend can0 00000123#11.22.33.44.55.66.77.88  		CAN0口发送ID为123的8字节CAN扩展帧
	cansend can0 123##2.00.11.22.33.44.55.66.77.88 		CAN0口发送ID为123的12字节CANFD标准帧(这里构造9字节，实际根据CANFD协议需要填充至12字节)
	cansend can0 123##3.00.11.22.33.44.55.66.77.88		CAN0口发送ID为123的12字节CANFD加速标准帧
	
	LIN报文
	sudo cansend lin0 b12#00.11.22.33.44.55.66.77

	配置帮助
	sudo ip link set can0 up type can help
	
8. 卸载驱动
	sudo rmmod usbcanfd

9. 接收问题
	socketcan的驱动，受限于系统socketcan的设置，驱动本身接收不会丢帧（可以通过ifconfig验证）。
	如遇到candump丢帧，而ifconfig不丢帧，可以通过下面两条命令修改内核关于socketcan的接收缓冲区大小。
	   
	设置接收缓冲区最大值（单位：字节，例如设为 25MB）
	 sudo sysctl -w net.core.rmem_max=26214400

	设置默认接收缓冲区大小（建议与 rmem_max 保持一致）
	 sudo sysctl -w net.core.rmem_default=26214400
	
