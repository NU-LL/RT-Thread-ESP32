## 在乐鑫 ESP32 上用 RT-Thread 实现语音唤醒和控制功能

### 一、项目背景及介绍

**背景：**乐鑫 ESP32 系列芯片提供了一个或两个可以单独控制的 CPU 内核，时钟频率可调，最高可达240 MHz。同时还包括 WIFI/BLE 双模，支持多种低功耗运行模式，具备低噪声放大器，I2S，高速 SPI，以太网等外设接口。是一款理想的嵌入式物联网芯片，可用于各类物联网应用场景。

**目标：**在乐鑫 ESP32 上用 RT-Thread 实现语音唤醒和控制功能。可以通过语音唤醒，并识别外部语音指令进行相关操作，如统计 RT-Thread 内部运行任务资源，控制外设 LED 灯，播报语音等。项目需要在 esp-idf 框架下完成 RT-Thread 系统移植适配，确保任务调度，内存分配，进程通信等功能运行正常。并结合 esp-adf 语音开发框架，完成语音唤醒和控制相关功能要求。

### 二、工程目录介绍

整个项目目前分为两个部分实现：

* idf部分：整个工程位于`idf-projects/get-started/hello_world`下，目前为esp-idf框架的测试工程，用于测试rtthread在esp-idf框架上的各种功能。

	> 目前rtthread充当了esp-idf的一个组件，具体位置为`idf-projects/get-started/hello_world/components/freertos`，目前由于esp-idf框架的限制（文件夹名称即为组件名称），文件夹名字只能取freertos，后期会尝试将文件名称改为rtthread

* adf部分：该部分是整个项目最后的工程，位于`adf-projects/speech_recognition/asr`下，需要esp-idf+esp-adf框架的支持，最终功能均在该工程下实现。目前该工程暂时只能成功实现语音唤醒和识别。

	> 该工程目前仍然使用FreeRTOS

其余两部分分别为esp-idf框架和esp-adf框架：

* esp-idf框架：即文件夹`esp-idf-4.0.1`，采用esp-idf框架4.0.1版本，该部分为[乐鑫官方idf源码](https://github.com/espressif/esp-idf/releases/tag/v4.0.1)，未曾改动

* esp-adf框架：即文件夹`esp-adf-2.0`，采用esp-adf框架2.0版本，该部分由[乐鑫官方adf源码](https://github.com/espressif/esp-adf/releases/tag/v2.0)修改而来（只增加了目前使用的[安信可SEP32 Audio Kit板卡](https://docs.ai-thinker.com/esp32-audio-kit)）

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

开发环境为Ubuntu18.04+Visual Studio Code

### 四、使用方式

按照乐鑫官方的[ESP-IDF指南](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/get-started/index.html)和[ESP-ADF指南](https://docs.espressif.com/projects/esp-adf/zh_CN/latest/get-started/index.html)配置对应平台（Windows、linux、macOS）上的开发环境后，到本项目对应工程目录下，依次执行以下命令即可编译、烧录、运行并查看结果：

```bash
# 清理工程
idf.py clean
# 编译
idf.py build
# 烧录
idf.py -p /dev/ttyUSB0 flash
# 监视器
idf.py -p /dev/ttyUSB0 monitor
```

* 其中烧录和监视器命令中的端口需要根据具体的串口号进行具体配置，详情请见ESP-IDF指南中的[第六步：连接设备](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/get-started/index.html#get-started-connect)。