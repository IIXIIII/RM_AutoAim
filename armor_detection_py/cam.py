import time
import struct
import serial
import numpy as np
import cv2

from det import ArmorDetector
from MvCameraControl_class import *
from CameraParams_header import *

# ========== 初始化海康相机 ==========
device_list = MV_CC_DEVICE_INFO_LIST()
ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
if device_list.nDeviceNum == 0:
    raise RuntimeError("No camera found.")

cam = MvCamera()
cam.CreateHandle(device_list.pDeviceInfo[0])
cam.OpenDevice()
cam.SetEnumValue("TriggerMode", 0)  # 连续取流模式
cam.SetEnumValue("PixelFormat", PixelType_Gvsp_BGR8_Packed)

# 设置分辨率和帧率（可选）
cam.SetFloatValue("AcquisitionFrameRate", 60)

# 启动取流
cam.StartGrabbing()

# ========== 初始化串口 ==========
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.01)

# ========== 初始化装甲板检测器 ==========
detector = ArmorDetector(binary_threshold=220)

# ========== 主循环 ==========
try:
    while True:
        data_buf = None
        st_frame_info = MV_FRAME_OUT_INFO_EX()

        ret, data_buf, frame_info = cam.GetOneFrameTimeout(1024 * 1024, st_frame_info, 100)
        if ret == 0:
            # 转换为 OpenCV 图像
            img = np.frombuffer(data_buf, dtype=np.uint8).reshape((st_frame_info.nHeight, st_frame_info.nWidth, 3))

            results = detector.detect(img)

            if results:
                # 提取第一个目标的中心点
                x1, y1, x2, y2 = results[0].armor_xyxy
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # 打包为2个float32小端发送
                packet = struct.pack('<2f', cx, cy)
                ser.write(packet)

                print(f"[TX] x={cx:.2f}, y={cy:.2f}")

        time.sleep(0.01)  # 稍微限制CPU占用

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    cam.StopGrabbing()
    cam.CloseDevice()
    cam.DestroyHandle()
    ser.close()
