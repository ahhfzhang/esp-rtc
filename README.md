# ESP-RTC example


## 例程简介

ESP RTC 是一个基于标准 SIP 协议的视频通话客户端，可以用于点对点音频/视频通话等场景。

## 环境配置

### 硬件要求

本例程目前仅支持`ESP32-S3-Korvo-2`开发板。

### 其他要求

- 您可以使用 Example 乐鑫自建的服务器来测试，具体测试账号可以联系乐鑫来获取。

- 或者您也可以搭建如下 SIP PBX 服务器中的一个：

  - [Asterisk FreePBX](https://www.freepbx.org/downloads/)

  - [Asterisk for Raspberry Pi](http://www.raspberry-asterisk.org/)

  - [Freeswitch](https://freeswitch.org/confluence/display/FREESWITCH/Installation)
      - 建议关闭服务器事件通知 `NOTIFY`，可以通过在 `conf/sip_profiles/internal.xml` 中设置 `<param name="send-message-query-on-register" value="false"/>` 关闭通知。

      - 建议关闭服务器 timer，可以通过在 `conf/sip_profiles/internal.xml` 中设置 `<param name="enable-timer" value="false"/>` 来关闭。

      - 建议在 `conf/vars.xml` 中打开 PCMA、PCMU、VP8。

- 我们建议搭建 Freeswitch 服务器来测试。

## 编译和下载

### IDF 默认分支

本例程默认 IDF 为 https://github.com/espressif/esp-idf/tree/release/v4.4

本例程默认 ADF 为 https://github.com/espressif/esp-adf/tree/master

本例程还需给 IDF 合入1个patch ，合入命令如下：

```
cd $IDF_PATH
git apply $ADF_PATH/idf_patches/idf_v4.4_freertos.patch
```

### 编译和下载

请先编译版本并烧录到开发板上，然后运行 monitor 工具来查看串口输出 (替换 PORT 为端口名称)：

```
idf.py -p PORT flash monitor
```

退出调试界面使用 ``Ctrl-]``

有关配置和使用 ESP-IDF 生成项目的完整步骤，请参阅 [《ESP-IDF 编程指南》](https://docs.espressif.com/projects/esp-idf/zh_CN/release-v4.4/get-started/index.html)。

## 如何使用例程

### 功能和用法

- 例程开始运行后，连接默认Wifi网络或者串口输入 "join" 来配置网络。
- 设备开机联网完成并成功连接服务器后，串口输入 "call <num>" 拨打对方号码 如 1010/1011
- 或者按下 "PLAY" 键来进行拨号
- 串口输入 "bye" 或者 按下 "MUTE" 键来进行挂断或者拒接
- 串口输入 "answer" 或者 按下 "REC" 键来接听
- 串口输入 "tasks" "stat" "mem" 查看系统状态
- "Vol+" 和 "Vol-" 键可以调节开发板通话音量
- 如果使用自建服务器，可以在 `LOGIN_URL` 中配置服务器域名和用户账号及密码 (Transport://user:password@server:port)
  - 例如：tcp://100:100@192.168.1.123:5060

## 技术支持
请按照下面的链接获取技术支持：

- 技术支持参见 [esp32.com](https://esp32.com/viewforum.php?f=20) 论坛
- 故障和新功能需求，请创建 [GitHub issue](https://github.com/espressif/esp-adf/issues)

我们会尽快回复。
