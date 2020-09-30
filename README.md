## 在乐鑫 ESP32 上用 RT-Thread 实现语音唤醒和控制功能

### 一、项目背景及介绍

**背景：**乐鑫 ESP32 系列芯片提供了一个或两个可以单独控制的 CPU 内核，时钟频率可调，最高可达240 MHz。同时还包括 WIFI/BLE 双模，支持多种低功耗运行模式，具备低噪声放大器，I2S，高速 SPI，以太网等外设接口。是一款理想的嵌入式物联网芯片，可用于各类物联网应用场景。

**目标：**在乐鑫 ESP32 上用 RT-Thread 实现语音唤醒和控制功能。可以通过语音唤醒，并识别外部语音指令进行相关操作，如统计 RT-Thread 内部运行任务资源，控制外设 LED 灯，播报语音等。项目需要在 esp-idf 框架下完成 RT-Thread 系统移植适配，确保任务调度，内存分配，进程通信等功能运行正常。并结合 esp-adf 语音开发框架，完成语音唤醒和控制相关功能要求。

### 二、工程目录介绍

整个项目目前分为两个部分实现：

* idf部分：整个工程位于`idf-projects/get-started/hello_world`下，为esp-idf框架的测试工程，用于测试rtthread在esp-idf框架上的各种功能。

	> 目前所采取的方案是将rtthread变成为一个esp-idf的组件，该组件的具体位置为`idf-projects/get-started/hello_world/components/freertos`，目前由于esp-idf框架的限制（文件夹名称即为组件名称），文件夹名字只能取freertos。目前已经在ESP32论坛中提出[该问题](https://esp32.com/viewtopic.php?f=25&t=17496&sid=072cda22047fe9d4f2e5a0cfceb5a382)，但是尚未给出解决方案

* adf部分：该部分是整个项目最后的工程，位于`adf-projects/speech_recognition/asr`下，需要提前搭建好esp-idf+esp-adf框架的环境。

其余两部分分别为esp-idf框架和esp-adf框架：

* esp-idf框架：即文件夹`esp-idf-4.0.1`，本项目所采用的esp-idf框架源码（4.0.1版本），该部分为[乐鑫官方idf源码](https://github.com/espressif/esp-idf/releases/tag/v4.0.1)，未曾改动

* esp-adf框架：即文件夹`esp-adf-2.0`，本项目所采用的esp-adf框架源码（2.0版本），该部分由[乐鑫官方adf源码](https://github.com/espressif/esp-adf/releases/tag/v2.0)修改而来（只增加了目前使用的[安信可SEP32 Audio Kit板卡](https://docs.ai-thinker.com/esp32-audio-kit)）

	> 具体修改文件为：
	>
	> * `esp-adf-2.0/components/audio_board/ai_thinker_audio_kit_v2_2/*`
	> * `esp-adf-2.0/components/audio_board/CMakeLists.txt`
	> * `esp-adf-2.0/components/audio_board/component.mk`
	> * `esp-adf-2.0/components/audio_board/Kconfig.projbuild`
	> * `esp-adf-2.0/components/audio_hal/driver/ac101/*`
	> * `esp-adf-2.0/components/audio_hal/CMakeLists.txt`
	> * `esp-adf-2.0/components/audio_hal/component.mk`
	>
	> 之后可能会单独提出该组件，从而不修改esp-adf源码

### 三、硬件配置

该项目所采用的具体硬件为[安信可SEP32 Audio Kit板卡](https://docs.ai-thinker.com/esp32-audio-kit)，是Ai-Thinker基于ESP32-A1S模组开发的小型音频开发板，大多数音频外设分布在开发板两侧，支持TF卡，耳机输出，两路麦克风输入和两路喇叭输出。

![esp32-audio-kit2](img/esp32-audio-kit2.png)

开发环境为Ubuntu18.04 + Visual Studio Code

### 四、使用方式

按照乐鑫官方的[ESP-IDF指南](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/get-started/index.html)和[ESP-ADF指南](https://docs.espressif.com/projects/esp-adf/zh_CN/latest/get-started/index.html)配置对应平台（Windows、linux、macOS）上的开发环境后，到本项目对应工程目录下，依次执行以下命令即可编译、烧录、运行并查看结果：

```bash
# 清理工程
idf.py clean
# 如果挪动了工程位置，可能需要如下命令来清除
idf.py fullclean
# 编译
idf.py build
# 烧录
idf.py -p /dev/ttyUSB0 flash
# 监视器
idf.py -p /dev/ttyUSB0 monitor
```

* 其中烧录和监视器命令中的端口需要根据具体的串口号进行具体配置，详情请见ESP-IDF指南中的[第六步：连接设备](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/get-started/index.html#get-started-connect)。
* adf工程中，唤醒词为`嗨，乐鑫`，关键词支持以下几种：
  * `打开电灯`：打开板载LED灯（IO22）
  * `关闭电灯`：关闭板载LED灯（IO22）
  * `版本号`：播报当前系统版本号
  * `软件包数量`：播报rtthread软件包数量

adf工程系统启动日志如下(分别说出`嗨，乐鑫。版本号`和`嗨，乐鑫。软件包数量`)：

```bash
ets Jun  8 2016 00:22:57

rst:0xc (SW_CPU_RESET),boot:0x1f (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0018,len:4
load:0x3fff001c,len:7392
ho 0 tail 12 room 4
load:0x40078000,len:14944
load:0x40080400,len:4988
entry 0x40080708
I (70) boot: Chip Revision: 1
I (70) boot_comm: chip revision: 1, min. bootloader chip revision: 0
I (41) boot: ESP-IDF v4.0.1-dirty 2nd stage bootloader
I (41) boot: compile time 21:41:33
I (50) boot: Enabling RNG early entropy source...
I (50) qio_mode: Enabling QIO for flash chip WinBond
I (52) boot: SPI Speed      : 80MHz
I (56) boot: SPI Mode       : QIO
I (60) boot: SPI Flash Size : 4MB
I (64) boot: Partition Table:
I (68) boot: ## Label            Usage          Type ST Offset   Length
I (75) boot:  0 factory          factory app      00 00 00010000 00300000
I (83) boot:  1 nvs              WiFi data        01 02 003d0000 00004000
I (90) boot: End of partition table
I (94) boot_comm: chip revision: 1, min. application chip revision: 0
I (102) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x165498 (1463448) map
I (508) esp_image: segment 1: paddr=0x001754c0 vaddr=0x3ffb0000 size=0x017e4 (  6116) load
I (510) esp_image: segment 2: paddr=0x00176cac vaddr=0x40080000 size=0x00400 (  1024) load
0x40080000: _WindowOverflow4 at /home/null/Code/ESP32/adf-examples/speech_recognition/asr_rtt/build/../components/freertos/libcpu/xtensa/common/xtensa_vectors.S:1781

I (515) esp_image: segment 3: paddr=0x001770b4 vaddr=0x40080400 size=0x08f5c ( 36700) load
I (536) esp_image: segment 4: paddr=0x00180018 vaddr=0x400d0018 size=0x2fef0 (196336) map
0x400d0018: _stext at ??:?

I (589) esp_image: segment 5: paddr=0x001aff10 vaddr=0x4008935c size=0x01f84 (  8068) load
0x4008935c: rt_timer_delete at /home/null/Code/ESP32/adf-examples/speech_recognition/asr_rtt/build/../components/freertos/src/timer.c:271 (discriminator 3)

I (599) boot: Loaded app from partition at offset 0x10000
I (599) boot: Disabling RNG early entropy source...
I (601) cpu_start: Pro cpu up.
I (604) cpu_start: Application information:
I (609) cpu_start: Project name:     speech_recognition_example
I (616) cpu_start: App version:      1
I (620) cpu_start: Compile time:     Sep 30 2020 17:05:51
I (626) cpu_start: ELF file SHA256:  69ff9af4584ed0d8...
I (632) cpu_start: ESP-IDF:          v4.0.1-dirty
I (638) cpu_start: Single core mode
I (642) heap_init: Initializing. RAM available for dynamic allocation:
I (649) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (655) heap_init: At 3FFB4960 len 0002B6A0 (173 KiB): DRAM
I (661) heap_init: At 3FFE0440 len 0001FBC0 (126 KiB): D/IRAM
I (668) heap_init: At 40078000 len 00008000 (32 KiB): IRAM
I (674) heap_init: At 4008B2E0 len 00014D20 (83 KiB): IRAM
I (680) cpu_start: Pro cpu start user code
initialize rti_board_start:0 done

 \ | /
- RT -     Thread Operating System
 / | \     3.1.3 build Sep 30 2020
 2006 - 2019 Copyright by rt-thread team
I (701) asr_keywords: Initialize SR wn handle
Quantized wakeNet5: wakeNet5_v1_hilexin_5_0.95_0.90, mode:0 (Mar 30 2020 20:16:38)
I (711) asr_keywords: keywords: hilexin (index = 1)
Fail: index is out of range, the min index is 1, the max index is 1I (721) asr_keywords: keywords_num = 1, threshold = 0.000000, sample_rate = 16000, chunksize = 480, sizeof_uint16 = 2
I (731) asr_keywords: create
WARNING: Sample length must less than 5000ms in the SINGLE_RECOGNITION mode
SINGLE_RECOGNITION: 2_0 MN1_4; core: 0; (May 15 2020 14:50:27)
SHIFT: 8, 12, 17, 17, 19, 17, 6, 16, 15, 14,
I (761) asr_keywords: get_samp_chunksize
I (761) asr_keywords: get_samp_chunknum
I (761) asr_keywords: get_samp_rate
I (771) asr_keywords: keywords_num = 166 , sample_rate = 16000, chunksize = 480, sizeof_uint16 = 2
I (781) asr_keywords: [ 1 ] Start codec chip
W (1781) AC101: reset succeed
I (1901) asr_keywords: [ 2.0 ] Create audio pipeline for recording
I (1901) asr_keywords: [ 2.1 ] Create i2s stream to read audio data from codec chip
I (1901) asr_keywords: [ 2.2 ] Create filter to resample audio data
I (1911) asr_keywords: [ 2.3 ] Create raw to receive data
I (1911) asr_keywords: [ 3 ] Register all elements to audio pipeline
I (1921) asr_keywords: [ 4 ] Link elements together [codec_chip]-->i2s_stream-->filter-->raw-->[SR]
I (1931) asr_keywords: [ 5 ] Start audio_pipeline
I (10161) asr_keywords: wake up
I (13081) asr_keywords: get RT-Thread version
I (13081) asr_keywords: [ 1 ] Start audio codec chip
W (13081) AUDIO_BOARD: The board has already been initialized!
I (13201) asr_keywords: [ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event
I (13201) asr_keywords: [2.1] Create mp3 decoder to decode mp3 file and set custom read callback
I (13211) asr_keywords: [2.2] Create i2s stream to write data to codec chip
W (13211) I2S: I2S driver already installed
I (13221) asr_keywords: [2.3] Register all elements to audio pipeline
I (13231) asr_keywords: [2.4] Link it together [mp3_music_read_cb]-->mp3_decoder-->i2s_stream-->[codec_chip]
I (13241) asr_keywords: [ 3 ] Set up  event listener
I (13241) asr_keywords: [3.1] Listening event from all elements of pipeline
I (13351) asr_keywords: [ 4 ] Start audio_pipeline
I (17351) asr_keywords: [ * ] Receive music info from mp3 decoder, sample_rates=44100, bits=16, ch=2
I (20161) asr_keywords: Wakeup time is out
W (20181) AUDIO_PIPELINE: There are no listener registered
I (25081) asr_keywords: wake up
I (33271) asr_keywords: get RT-Thread softpack number
I (33271) asr_keywords: [ 1 ] Start audio codec chip
W (33271) AUDIO_BOARD: The board has already been initialized!
I (33391) asr_keywords: [ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event
I (33391) asr_keywords: [2.1] Create mp3 decoder to decode mp3 file and set custom read callback
I (33401) asr_keywords: [2.2] Create i2s stream to write data to codec chip
W (33401) I2S: I2S driver already installed
I (33411) asr_keywords: [2.3] Register all elements to audio pipeline
I (33421) asr_keywords: [2.4] Link it together [mp3_music_read_cb]-->mp3_decoder-->i2s_stream-->[codec_chip]
I (33431) asr_keywords: [ 3 ] Set up  event listener
I (33431) asr_keywords: [3.1] Listening event from all elements of pipeline
I (33541) asr_keywords: [ 4 ] Start audio_pipeline
I (35081) asr_keywords: Wakeup time is out
I (37541) asr_keywords: [ * ] Receive music info from mp3 decoder, sample_rates=44100, bits=16, ch=2
W (40671) AUDIO_PIPELINE: There are no listener registered
```

### 五、注意事项

* 由于时间有限，兼容层有少许函数暂未实现，其中包括`TLS`类函数（`vTaskSetThreadLocalStoragePointer`、`pvTaskGetThreadLocalStoragePointer`、`vTaskSetThreadLocalStoragePointerAndDelCallback`），该部分负责实现pthread中用户申请变量的垃圾回收，在诸如音频管道和wifi中会用到，可能导致的后果是随着调用次数过多而导致内存泄露从而使系统崩溃。

    > 移植该部分需要对线程结构体中加入垃圾回收回调函数成员，需要对原本的内核进行较大的改动，综合考虑后决定先暂时不实现该部分

* 由于线程栈中exit成员必须为IDF框架提供的函数`_xt_user_exit`，所以在目前的代码中线程创建后不能退出。出于项目兼容性的考虑，目前的退出方法是通过利用无限延时来暂时解决的。

    > 因为中断退出后或还原上文时，会调用栈帧中的exit成员`_xt_user_exit`函数还原部分关键寄存器。解决方法：可以在汇编中相应调用exit成员函数的地方改为直接调用`_xt_user_exit`，从而腾出exit的位置让给rtthread的`rt_thread_exit`函数接管线程退出的后续部分，从而解决函数退出的问题。由于该项目时间的考虑该部分暂时未解决。

* 原计划部分语音的播报采用wifi+百度语音在线合成（ESP32不支持本地语音合成）的方式，但是发现百度该项服务已经改成收费模式遂改变策略为调用本地音频，但是原本访问百度语音在线合成的代码仍保留在adf工程中（已经注释）

    > 由于上述问题，WIFI处于STA模式时不能够自动重连，如若第一次未连接成功，后续和WIFI有关的操作有可能失败

### 六、个人总结及感悟

在整个项目过程中我收获良多，尤其是涉及到内核底层的一些知识。这里为本人在开发过程中的一些调试总结与感悟，提供给对该项目感兴趣的朋友：

* [FreeRTOS内存管理](https://nu-ll.github.io/2020/08/27/FreeRTOS%E5%86%85%E5%AD%98%E7%AE%A1%E7%90%86/)
* [异常和中断](https://nu-ll.github.io/2020/08/25/%E5%BC%82%E5%B8%B8%E5%92%8C%E4%B8%AD%E6%96%AD/)
* [Xtensa基础](https://nu-ll.github.io/2020/07/06/Xtensa%E5%9F%BA%E7%A1%80/)
* [ESP32](https://nu-ll.github.io/2020/05/16/ESP32/)
* [ESP32上移植Rt-thread](https://nu-ll.github.io/2020/07/06/ESP32%E4%B8%8A%E7%A7%BB%E6%A4%8DRt-thread/)
