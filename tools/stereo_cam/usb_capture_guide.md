# USB 抓包操作指南 — 捕获 YLX 陀螺仪激活命令

## 目标
捕获 Windows 下 YLX 厂商驱动发送给摄像头的 UVC Extension Unit 控制命令（特别是 XU#4 的激活命令），以便在 Linux uvcvideo 驱动中复现。

## 前置条件
- 摄像头 **不传给虚拟机**，保持连在 Windows 主机上
- Windows 上装好 YLX 厂商驱动/SDK，摄像头能正常工作（含陀螺仪）

---

## Step 1：安装 USBPcap

1. 运行 `USBPcapSetup-1.5.4.0.exe`
2. 安装过程中勾选 **"Install USBPcapCMD"** 和驱动
3. 安装后 **重启电脑**（驱动需要重启生效）

## Step 2：安装 Wireshark

1. 运行 `Wireshark-4.6.6-x64.exe`
2. 安装过程中勾选 **USBPcap** 组件（如果出现的话）
3. 如果 Wireshark 没有 USBPcap 组件也没关系 — USBPcap 在 Step 1 已装好

## Step 3：列出 USB 设备

以管理员身份打开 CMD：
```
cd "C:\Program Files\USBPcap"
USBPcapCMD.exe
```

会列出所有 USB Root Hub，找到 YLX 摄像头 (1bcf:0b15) 所在的 Root Hub 编号。

## Step 4：开始抓包

```cmd
USBPcapCMD.exe -d \\.\USBPcap1 -o D:\ylx_gyro_capture.pcapng
```

> 把 `USBPcap1` 替换为 YLX 摄像头所在的 Root Hub 编号

## Step 5：操作摄像头

在 USBPcap 正在捕获的同时：
1. 打开 YLX 摄像头软件/SDK demo
2. 启动视频预览（确保陀螺仪数据在软件中可见）
3. 等待 5-10 秒
4. 关闭摄像头软件

## Step 6：停止抓包

在 USBPcapCMD 窗口按 `Ctrl+C`。

## Step 7：分析抓包文件

用 Wireshark 打开 `.pcapng`：

**关键过滤条件：**

1. UVC VideoControl 接口请求：
   ```
   usb.src == "host" && usb.dst == "1.5.0" && usb.setup.bmRequestType == 0x21
   ```

2. UVC Extension Unit 请求（XU#4）：
   ```
   usb.setup.bmRequestType == 0x21 && usb.setup.bRequest == 0x01
   ```
   (SET_CUR = 0x01, GET_CUR = 0x81, GET_MIN = 0x82, GET_MAX = 0x83, GET_RES = 0x84, GET_LEN = 0x85, GET_INFO = 0x86, GET_DEF = 0x87)

3. 只看控制传输（非批量/等时）：
   ```
   usb.transfer_type == 0x02
   ```

**重点关注：**
- `wValue` 高位是 control selector，低位是 0
- `wIndex` 高位是 XU 编号 (4)，低位是接口号
- 数据阶段（Data Stage）的内容 = 激活命令参数
