ittiam code STA/AP 模式修改记录：



bug fix说明：
===========
2013-02-27
1.sta模式，修复将wpa/wpa2加密显示为wep bug。
2.sta模式，切换power type不会重新关联，但暂时拿掉driver 层ps策略。
3.sta/ap模式，调整open/close，exit/init接口中的操作。

2013-02-20
1.修正ps开启关闭不重新关联。
2.增加驱动层休眠策略，2s一个统计周期，若忙保持active状态，若闲进入ps。
3.可以接收ap发来的A-ASDU。
4.解决AP模式下，STA连上后切换模式后，STA连接不上AP，AP不发Beacon帧问题。
注意：此版中默认支持以下802.11n功能：
      (1). A-MSDU聚合功能(包括发送和接收A-MSDU聚合功能，接收AP回应的A-MSDU时，需要使用支持AMSDU聚合功能的AP，且需在AP中开启AMSDU聚合功能).
      (2). 支持RX Block-Ack，TX Block-Ack功能未开启。
      (3). 支持接收AMPDU聚合包，发送AMPDU功能未开启。

2013-02-15:
1.sta模式，增加android应用对wifi流量的获取接口。
2.sta模式，增加scan ap列表最大数，20个ap info。
3.sta模式，支持802.1x认证方式。
4.ap模式，修复有网卡进入ps后，tim值没有更新bug。
5.sta模式，修复ui下切换ap连接，出现“已停用”。
6.sta/ap模式，修复打开/关闭模块时会出现“灰显”或“重启”bug。
7.sta模式，优化对ui命令的响应和逻辑判断。
8.sta模式，增加判断：当前收发包忙时，scan命令不会执行。
9.sta模式，修复将wpa/wpa2加密显示为wep bug。

2013-02-05:
1.整合trout2 rf 55 增加bt共存测试代码。
2.解决优化后rx dscr只清除了状态位，未清除dscr中前6Word可能导致出问题的风险，已经优化为block write方式清状态，但是通过
block write方式清dscr中前6个word发现比采用ram write方式还慢，因此，为了提高吞吐，屏蔽掉block write方式操作。


2013-02-04:
1.增加trout2 fpga rf55 和 rf2829 以及 MF EVB的宏，用于rf配置。
2.config.mk中增加宏，方便根据具体的硬件环境进行测试。

2013-01-31:
1. 新增STA下的TX Block-Ack功能，即：STA下的Block-Ack TX/RX都可以正常工作，但TX的block-ack仍然有很多RTS，影响吞吐。
   另外，AP模式下的TX Block-Ack功能还有问题。
2. 此版中可以测试硬件在802.11n模式下的几项功能是否正常：A-MSDU/A-MPDU/Block-Ack，使用中只需在amsdu_aggr.c中打开对应
   的宏即可，另外需要说明的是：block-ack与A-MPDU共存的时候还有一些问题，暂时不能同时开启。

2013-01-23:
1.修复rx本地buffer使用中，可能因为用完后只清除状态并没有清除使用计数，导致后续有包来时，没有本地buffer可使用问题。

2013-01-16:
1. 添加系统启动时，mem初始化(rx,tx)失败时保护机制，防止失败时，程序继续运行导致重启问题。


2013-01-12: chwg
1. 优化并修改reset_mac()处理流程，解决reset_mac时，其它操作也在进行导致系统挂掉问题。

2013-01-12: chenq
1.修改复位时事件统计出现异常而导致死循环。（现象多为wlan框一直灰显）
2.增加连接尝试次数，超过后sta不再发起连接。（现象多为ui显示不准确）
3.修改scan时的配置参数，每个信道时间由20ms改为10ms。
4.增加连接时的mac地址匹配，用于区分同名essid的出现。
5.配合修改wpa_supplicant，先配置mac地址在配置essid，优化了用户体验。

2013-01-10：
1. 去掉rx处理中的自旋锁，改用互斥锁，并对rx处理进行保护，防止reset_mac时，两边不同步导致异常的问题。

2.增加配置文件用于修改soft ap模式下的mac地址和信道。
使用方法：
*自行创建trout_wifi.cfg，文件内容格式
mac=xx:xx:xx:xx:xx:xx
channel=x

*导入
push 到/system/lib/modules/
chmod 777 trout_wifi.cfg

3.sta模式在连接一个不存在的AP或者是因为信号参数等原因被过滤掉的AP时，
因为不能成功连接而影响的UI界面ap列表无法更新的问题。

2013-01-9:
1. 在init_mac_mem_pools()中加入了判断返回值，防止当初始化时因分配内存不成功导致程序继续往后走导致异常重启的问题。

2013-01-07:
1.修正sta速率显示有误。
2.修正关闭wlan或者soft ap是UI卡住。
3.修正sta在连接上wpa/wpa2加密方式的ap后，因为长时间不发NULL frame被提掉。
4.修正设置soft ap加密方式为WPA2-AES后连不上。
5.sta模式下切换连接ap后，将原来连接的ap显示“已停用”修正为“已保存”
6.修复了sta模式下扫描AP过程中长时间（大于30秒）无法扫描到近距离且高信号强度AP的BUG。

另：增加ap模式的信道以及mac地址切换
iwconfig wlap0 rate 1～9（mac的最后一位就是1～9，信道也在1～9）

2013-01-06:
1. 出现在跑iperf测试时，进入UI扫描界面，扫描AP一段时间以后，返回上一级，提示设备断开连接，经分析确认，是由于软件中设置link_lose阀值太小导致，
建议至少设置为50次，程序中修改为50次。

2013-01-05：
1. 修复BA rx异常导致系统重启问题，具体位置为process_ba_rx_buff_frame()中wlan_rx.sa/wlan_rx.da为空导致空指针导致重启。

2012-12-31:
1. AP/STA下都可以正常工作。

2012-12-29：
1. 解决收包处理中由于挂包时未先清除子队列尾部导致硬件next寄存器更新出错的bug.

2012-12-29:
1. 解决收包处理中由于收包中断处理rx_complete_isr()与rx_complete_work()不同步导致同一个dscr被shedule处理两次导致trout收包队列错乱问题。

2012-12-24:
1. AP模式下，STA连上AP后，在block-ack交互协商中，有时因为STA因某种原因断开后，AP不知道，然后STA重新进行block-ack协商认证，
   AP认为STA已经建立过交互过程并且没有用addba结束交互过程，因此对STA发出的block-ack协商不予进行响应。
   
   解决方法：AP在接收到block-ack请求之后，首先查看是否已经与该STA建立了block-ack协商信息，如果有，则清除与该STA相关的block-ack信息，
   然后再进行block-ack回应。
   
   代码修改位置：文件management_11n.c中函数handle_wlan_addba_req()中1590行。
-------------------------------------------------------------------------------------------------------------
2012-12-21:
1. 修改脚本，方便调试。

2012-12-20:
1.陈沁将此2.2.2版本上级为2.3.5版本。
在config.mk中加入宏开关TROUT_WIFI_FPGA
若在fpga上跑请打开此宏。

2012-12-19:
1. 合并陈沁关于Android2.2.2和部分Android2.3.5及解决部分bug的版本和程文刚优化后的版本。
2. 初步调试STA及AP模式都可以正常工作。

2012-12-17:
1. 修复了rx处理中do_rx_preprocess()中出错处理不完善的问题。
2. 此版为合并陈沁修改的版本之前的trout优化版本最新版本备份。

2012-12-11:
1. 合并胡哥关于tx，将tx share memory划分成两块，分时复用，提高发送效率。
2. 之前添加的trout tx/rx buffer动态自适应机制暂时未打开，准备重新修改。

2012-12-10:
1. 加入了trout tx/rx buffer动态自适应机制。
2. 解决了sta在802.11n only模式下收包测试时，会因为软件死锁导致系统重启的bug，具体原因为：在11n模式下，process_all_event检测到有待处理
   事件，但实际去处理时，却因为event刚alloc但还没有post，所以导致buffer状态为2的event被状态为1的event打断，get_event始终获取不到，导致
   在process_all_event()的while()中一直循环造成软件超时，同时修改send_data_frame_to_host_if()中不必要的post event方式，另外需要注意：
   process_all_event()中g_event_cnt的次数会由于程序在其它地方主动调用process_event_queues()中的函数，导致g_event_cnt自动减一，有可能与
   此处预想的不一致。
3. 在802.11n only模式下，频繁打印send delBA情况，追其原因是由于在n模式下，收到了AMPDU聚合包，而聚合包中不含802.11n包头，但是代码中还是
   按照802.11 MAC头去解析，导致解析出的MAC地址在entry表中找不到，软件认为此MAC地址没有协商是否采用Block-Ack传输就开始了块传输，因此发出
   结束块传输命令(暂未解决，需要讨论处理方式，当前暂时屏蔽掉)。

2012-11-29:
1.  多次设置essid导致adb shell没有响应问题已经没有了。

2012-11-26:
1. 去掉tx_pkt_process处理时，修改trout中tx_dscr状态以及判断is_trout_tx_mem_empty()中读tx dscr的状态，降低IO次数。
2. 去掉qmu_cpy_to_trout中启动发送过程中检测tx pointer中lock_bit是否被占用，降低IO，以上修改没有出现问题。

2012-11-21:
1. 将mac_isr_work中针对normal-rx, high-rx, tx中断分别单独清中断方式改为一次清除。
2. 将mac_isr_work中每次读int mask reg获取int mask值改为通过全局变量g_int_mask来获取。
3. 将qmu_cpy_to_trout中读TSF reg改为第一次的初值由tsf reg获取，后续的值通过初值加上本地时间累积值来更新，一定时间间隔(10s)之后
   再次读tsf reg的值作为初值以矫正tsf值，降低IO次数。
2012-11-10:
1. 去掉协议栈调用mac_xmit()时，由于txq full导致无效的pkt memory copy.
2. 去掉spi_interface.c中读写reg/ram接口多余的互斥锁。
3. 为了提高tx吞吐，将trout tx share memory size由8KB增大到32KB，rx相应的减小。

2012-11-02：
1. 由于发包时，更新tx descr中TSF域调用update_tx_dscr_tsf_ts()很耗时，超过80us的几率比较大，所以尽量避免读该寄存器。
   修改方式为：当采用一次DMA处理qmu_cpy_to_trout()时，只读一次tsf register，然后其它几次tx descr的值就用此值，测试
   不会出问题。

2012-10-30：
1. 调整部分中断操作部分I/O，发包通过中断方式触发，但还存在丢中断问题。

2012-10-23:
优化记录：
1. 将tx_pkt_process()中I/O方式回读tx dscr信息改为通过DMA方式读取，从而降低I/O操作次数。
2. 在原来qmu_cpy_to_trout()中分多次搬移数据到trout的方式的基础上添加先将多次搬移的数据搬移到一整块buffer中，然后再通过1次DMA搬移
   到trout中，降低了DMA搬移小块数据的次数(通过config.mk中宏TX_PKT_USE_DMA控制)。
3. 将前一次qmu_cpy_to_trout中数据是否被硬件真正发送成功判断由原来的判断每个tx dscr状态改为只判断最后一个tx dscr，降低了I/O次数。
4. 将mac_xmit触发的qmu_cpy_to_trout改为只有txq_handle中tx dscr <= 1时才通过mac_xmit触发方式，其它的qmu_cpy_to_trout全部由tx isr
   来触发，这样就避免了查询txq是否为null而引起的多读I/O，但由于tx存在丢中断现象，因此，此方法在正式代码中不可行。

2012-10-12:
1. 将is_all_machw_q_null()函数调用改为每隔5次读取一次以便降低读I/O次数，仅仅用于测试(最好的方法是在event处理中调用此函数时再判断每隔5次调用一次，这样不会
影响到tx isr中调用此函数)。
2. 优化tx_pkt_process()处理，通过DMA的方式降低I/O次数。

2012-10-10:
1. 在trace中新添加irq及event_wq中I/O操作次数统计等信息，方便定位CPU占用统计。
通过在adb shell中输入：
echo print_count_info > /sys/devices/platform/sprd-sdhci.1/mmc_host/mmc1/mmc1:0001/mmc1:0001:1/trout;dmesg -c
即可查看统计信息。
另外还可以通过：
echo clean_count_info > /sys/devices/platform/sprd-sdhci.1/mmc_host/mmc1/mmc1:0001/mmc1:0001:1/trout;dmesg -c
清除统计信息。
2012-10-08:
1. 在trace的基础上合并蔡水发最新版的固件，udp发包测试中，802.11bgn模式下未出现发包速率只有几百KB问题，但
receive_from_host()中event分配失败问题依然存在。
2. 修正STA/AP模式下，spi write ram时，Beacon帧前没有预留8bytes的Bug。

2012-09-29:
1. trace接口已经加好，通过adv_config.mk中宏"TROUT_TRACE_DBG"来开关，它依赖于宏"DEBUG_MODE"。
另外，测试中发现trout udp发送时，g模式下发包正常，但802.11bgn模式时，发包速率只有几百KB，分析
发现receive_from_host()中event分配失败导致，但蔡水发正常发布版本却没有此问题，还需对比确认。

2012-09-04:
1. 整合ittiam_mac_v1.3版。

2012-08-23
modify wifi direct (P2P) feature
1. P2P功能使用到了share ram尾部的空间，所以如果在其他部分空间增大的情况下（比如rx buf大于32个队列），
   将导致share ram使用空间超出范围。
2. 在config.mk文件中: 已关掉DEFAULT_SME宏；已打开MAC_P2P宏，使能P2P功能。
3. 配置P2P功能的详细文档在svn上的地址：http://10.6.2.16/svn/trout/6820/6820_doc/wifi_direct/wifi_direct_readme.txt

2012-08-06:
1. 优化并精简trout rx部分代码，尽量避免不必要的读trout造成系统资源消耗。

2012-08-03:
1. 增加了Counter寄存器。
//ACK/CTS/RTS/FCS fail/etc(detail in Received Frame Filter Register-0x80) Filter Counter when RX Frame Done
#define rMAC_RX_FRAME_FILTER_COUNTER        (PA_BASE + 0x0050)

//Address etc filter counter(detail in Received Frame Filter Register) after RX MAC Header done
#define rMAC_RX_MAC_HEADER_FILTER_COUNTER   (PA_BASE + 0x0054)

//the counter for the losted frames when the Rx Q is Full.
#define rMAC_RXQ_FULL_COUNTER          (PA_BASE + 0x005C)

//the counter for the rx frames into the rx q.
#define rMAC_RX_RAM_PACKET_COUNTER          (PA_BASE + 0x00C4)

2. 根据吴冉青建议，修改一些寄存器参数：  
       			     PHY Bank 1：  address      	data
						0x70		0x29
						0x71		0x30
				9861:		address 	data
						0xd 		0x80
						0x10 		0x80
						0x11 		0xff
	我们去掉了DC offset之后发包情况很好，CRC error几乎没有

3. 增加了iwconfig iwlan0 rate auto命令调用调试接口。
4，解决了Tx队列无法堆积的bug。


2012-08-01:
1. 调整了trout tx部分代码，避免了因发包中断中num_dscr不准确带来的发包发不出去等问题，可能未完全解决。


2012-07-23：
1. 合并了陈沁最新的配置程序，解决了因"validate_buffer"导致的系统Reset现象。
2. 其它功能同0721版，均正常。
3. spi接口中，使能了DMA，spi时钟改为24MHZ，需要更换boot_image为boot_cs24_gap11.img。

2012-07-21：
1. AP模式下，修复了多个sta连接上Ap之后，互ping及发长包不通的问题。
2. 合并了SDIO接口。

2012-07-20:
1. STA/AP模式下，修复了系统重启的bug。

2012-07-18：
1. AP/STA模式均能正常工作。
2. AP/STA模式都支持802.11b/g/n模式，且在n模式下，都支持A-MSDU聚合功能，能ping长包。
3. 修复了部分导致内核Oop及invalid_buffer导致系统reset的问题，提高了系统稳定性。

2012-07-16：
1. AP模式下，在基于7.15版本上，调整RF参数，对应FPGA版本：trout_fpage_spi_u2_syn_0710_ada.bin;
主要参数调整：
(1) TX/RX时钟：
   system reg 0x62【WORD地址】，配置为0x800000, 即BIT23设置为1
   system reg 0x63【WORD地址】，配置为0x20, 即BIT5设置为1
对应程序位置：csl_linux.c: line296~299;
(2) 修改9863的配置:
	MFE_TX7_VAL 	0xFF 修改为0x60
    MFE_IOC1_VLAN	0xC0 修改为0x00
对应程序位置：phy_ittiam.h: line583, line540;
(3) 使能函数write_mxfe_reg():
对应程序位置：phy_ittiam.h: line620~640;    
(4) 修改phy_max2829_ittiam.h参数：
#define TXPOWER11A    0x3F
#define TXPOWER11B    0x3F
对应程序位置：phy_max2829_ittiam.h: line429~430;	


2012-07-15:
1. AP模式，802.11b/g/n模式下，支持n模式下，A-MSDU聚合功能(仅限于验证FPGA芯片功能)。

2012-07-12:
1. AP模式，802.11b/g/n模式下，可以ping通长数据包(大于1518)，调整部分代码及bug。
2. 合并AP/STA模式代码，但STA模式下还未验证。

2012-07-04:
1. 实现了reset_phy_ittiam()的功能，并调整spi读写寄存器接口(主要修改了宏SPI_RD_ERG_CMD、SPI_WR_ERG_CMD)。
2. 已实现连接在同一个AP下的多台PC互相ping通功能。