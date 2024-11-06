# 用意：OpenMV入门学习 - 颜色识别和坐标获取并通过串口发送数据
# 坐标系：
#（原点）--------X轴
#  |
#  |
#  |
#  |
#  |
#  Y
#  轴
# 在进行颜色追踪时，需要控制环境光线稳定，避免识别标志物的色彩阈值发生改变

import sensor, image, time, math
import ustruct  # 用于打包数据
from pyb import UART  # UART用于串口通信

# 定义颜色阈值
blue_threshold = (43, 52, -1, 33, -78, -46)   # 设置蓝色阈值
green_threshold = (67, 51, -14, -41, -38, -6) # 设置绿色阈值
red_threshold = (30, 61, 7, 51, -7, 30)       # 设置红色阈值(45, 58, 21, 46, -11, 20)

# 初始化摄像头设置
sensor.reset()  # 初始化摄像头
sensor.set_pixformat(sensor.RGB565)  # 设置摄像头图像格式为RGB565
sensor.set_framesize(sensor.VGA)     # 设置图像分辨率为VGA(640*480)
sensor.set_hmirror(True)    # 水平镜像翻转
sensor.set_vflip(True)      # 垂直翻转
sensor.skip_frames(n=2000)  # 跳过2000帧，等待摄像头传感器稳定
sensor.set_auto_gain(True)  # 设置自动增益为True（使用颜色识别时应关闭）
sensor.set_auto_whitebal(True)  # 设置自动白平衡为True（使用颜色识别时应关闭）

# 初始化时钟对象用于追踪帧率
clock = time.clock()

# 初始化UART串口通信
uart = UART(3, 115200)  # 设置UART3，波特率为115200
uart.init(115200, bits=8, parity=None, stop=1)  # 初始化UART配置：8个数据位，无校验位，1个停止位

# 定义通过串口发送数据的函数
def sending_data(color, cx_h, cx_l, cy_h, cy_l):
    global uart
    # 使用ustruct将数据打包成指定格式，<BBBBBB表示6个字节，分别是颜色信息和坐标数据
    data = ustruct.pack("<BBBBBBBB",
                        0x2c,  # 起始字节
                        0x12,  # 数据类型（自定义）
                        color,  # 颜色编码
                        cx_h,   # 中心X坐标高8位
                        cx_l,   # 中心X坐标低8位
                        cy_h,   # 中心Y坐标高8位
                        cy_l,   # 中心Y坐标低8位
                        0x5b)   # 结束字节
    uart.write(data)  # 通过UART发送数据
    for i in data:  # 打印每个发送的数据
        print("data的内容是：", hex(i))  # 以十六进制打印

# 主循环：图像捕捉和颜色识别
while(True):
    # 获取当前帧图像，并进行镜头畸变校正
    img = sensor.snapshot().lens_corr(strength=1.8, zoom=1.0)

    # 查找符合颜色阈值的颜色块
    blue_blobs = img.find_blobs([blue_threshold], x_stride=30, y_stride=25, pixels_threshold=100)
    green_blobs = img.find_blobs([green_threshold], x_stride=30, y_stride=25, pixels_threshold=100)
    red_blobs = img.find_blobs([red_threshold], x_stride=30, y_stride=25, pixels_threshold=100)

    # 如果找到蓝色物体
    if blue_blobs:
        color_status = ord('B')  # 设置颜色状态为'B'
        for b in blue_blobs:  # 遍历找到的每个蓝色块
            img.draw_rectangle((b[0], b[1], b[2], b[3]), color=(0, 0, 255))
            img.draw_cross(b[5], b[6], size=2, color=(0, 255, 0))
            img.draw_string(b[0], (b[1]-10), "blue", color=(0, 0, 255), scale=3)
            print("中心X坐标", b[5], "中心Y坐标", b[6], "识别颜色类型", "蓝色")

            # 计算高低字节
            cx_h = (b[5] >> 8) & 0xFF  # 中心X坐标高8位
            cx_l = b[5] & 0xFF         # 中心X坐标低8位
            cy_h = (b[6] >> 8) & 0xFF  # 中心Y坐标高8位
            cy_l = b[6] & 0xFF         # 中心Y坐标低8位

            # 调用函数发送数据
            sending_data(color_status, cx_h, cx_l, cy_h, cy_l)

    # 如果找到绿色物体
    elif green_blobs:
        color_status = ord('G')  # 设置颜色状态为'G'
        for g in green_blobs:  # 遍历找到的每个绿色块
            img.draw_rectangle((g[0], g[1], g[2], g[3]), color=(255, 255, 255))
            img.draw_cross(g[5], g[6], size=2, color=(255, 255, 255))
            img.draw_string(g[0], (g[1]-10), "green", color=(0, 255, 0), scale=3)
            print("中心X坐标", g[5], "中心Y坐标", g[6], "识别颜色类型", "绿色")

            # 计算高低字节
            cx_h = (g[5] >> 8) & 0xFF
            cx_l = g[5] & 0xFF
            cy_h = (g[6] >> 8) & 0xFF
            cy_l = g[6] & 0xFF

            # 调用函数发送数据
            sending_data(color_status, cx_h, cx_l, cy_h, cy_l)

    # 如果找到红色物体
    elif red_blobs:
        color_status = ord('R')  # 设置颜色状态为'R'
        for r in red_blobs:  # 遍历找到的每个红色块
            img.draw_rectangle((r[0], r[1], r[2], r[3]), color=(255, 0, 0))
            img.draw_cross(r[5], r[6], size=2, color=(0, 255, 0))
            img.draw_string(r[0], (r[1]-10), "red", color=(255, 0, 0), scale=4)
            print("中心X坐标", r[5], "中心Y坐标", r[6], "识别颜色类型", "红色")

            # 计算高低字节
            cx_h = (r[5] >> 8) & 0xFF
            cx_l = r[5] & 0xFF
            cy_h = (r[6] >> 8) & 0xFF
            cy_l = r[6] & 0xFF

            # 调用函数发送数据
            sending_data(color_status, cx_h, cx_l, cy_h, cy_l)

    else:
        color_status = ord('A')  # 设置默认颜色状态为'A'
        # 发送默认数据，表示未识别到颜色块
        sending_data(color_status, 0, 0, 0, 0)  # 发送默认值
