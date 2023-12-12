import math
import cv2 as cv
import numpy as np
import serial
import struct
import threading

ESC = 27

COMPUTER_SCREEN = 0
VIDEO_SCREEN = 1

CAMERA = 0

# 串口相关
usart = '/dev/serial0'
bitrate = 9600
ser = serial.Serial(usart, bitrate, timeout=None)

# 模式选择
MODE_SCAN_LINE = 0  # 扫线模式
MODE_DETECT_CIRCLE = 1 # 找圆模式

MODE_DETECT_CIRCLE_NOCOCLR = 0 # 没有颜色
MODE_DETECT_CIRCLE_RED = 1  # 找红色的圆
MODE_DETECT_CIRCLE_BLUE = 2  # 找蓝色的圆
MODE_DETECT_CIRCLE_GREEN = 3  # 找绿色的圆
CIRCLE_DEBUG = 5
MAIN_MODE = MODE_SCAN_LINE
CIRCLE_MODE = MODE_DETECT_CIRCLE_BLUE
# 阈值部分
lg = np.array([75, 82, 58], dtype=np.uint8)  # 绿色阈值下界
hg = np.array([94, 255, 255], dtype=np.uint8)  # 绿色阈值上界

lb = np.array([108, 58, 45], dtype=np.uint8) # 蓝色阈值下界
hb = np.array([157, 228, 255], dtype=np.uint8)# 蓝色阈值上界

lr1 = np.array([0, 43, 46], dtype=np.uint8)  # 红色阈值下界1
hr1 = np.array([10, 255, 255], dtype=np.uint8)  # 蓝色阈值上界1
lr2 = np.array([156, 43, 46], dtype=np.uint8)  # 红色阈值下界2
hr2 = np.array([180, 255, 255], dtype=np.uint8)  # 红色阈值上界2

# 颜色部分
blueTarget = (255, 0, 0)
greenTarget = (0, 255, 0)
redTarget = (0, 0, 255)

NOCOLOR = 0
RED = 1
GREEN = 2
BLUE = 3

# 文本部分
greenText = 'Green'
blueText = 'Blue'
redText = 'Red'

# 阈值
INF = 100000

# 屏幕相关
screenwidth = 320
screenheight = 240

# 串口的长度
usartbuf = [0 for _ in range(2)]

# 锁
seriallock = threading.Lock()

def serial_thread():
    # 引入串口对象
    global ser
    global MAIN_MODE
    global CIRCLE_MODE
    global usartbuf
    global seriallock
    # 接收数据
    while True:
        # 读取1字节数据
        seriallock.acquire()
        
        serialdata = ser.read(4)
        serialdata = str(serialdata.decode())
        # print(f'{serialdata}')
        
        if (len(serialdata) == 4) and (serialdata[0] == 'E') and (serialdata[3] == 'F'):
            usartbuf[0], usartbuf[1] = serialdata[1], serialdata[2]
            
        seriallock.release()
            
        MAIN_MODE = int(usartbuf[0])
        CIRCLE_MODE = int(usartbuf[1])
        
        print(f'{MAIN_MODE} {CIRCLE_MODE}')

def LookForCircle(frame, color, colorTarget, min_area, max_area, low, high, elower=None, eupper=None):
    global screenwidth, screenheight
    state = NOCOLOR
    # 颜色阈值分割
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # 二值化
    if (elower is not None and eupper is not None) or (elower is not None and eupper is not None):
        rect_roi1 = cv.inRange(hsv_frame, low, high)
        rect_roi2 = cv.inRange(hsv_frame, elower, eupper)
        rect_roi = rect_roi1 + rect_roi2
    else:
        rect_roi = cv.inRange(hsv_frame, low, high)
        
    # cv.imshow('color_mask', rect_roi)

    # 均值滤波
    # rect_roi = cv.blur(rect_roi, (5, 5))
    # 高斯滤波
    # rect_roi = cv.GaussianBlur(rect_roi, (5, 5), 0)
    # 中值滤波
    # rect_roi = cv.medianBlur(rect_roi, 5)

    kernel = np.ones((5, 5), np.uint8)

    # 执行开运算操作
    rect_roi = cv.erode(rect_roi, (3, 3), iterations=1)
    # rect_roi = cv.morphologyEx(rect_roi, cv.MORPH_OPEN, (5, 5), iterations=1)
    # 执行膨胀操作
    rect_roi = cv.dilate(rect_roi, kernel, iterations=2)

    # cv.imshow('dilated_image', rect_roi)

    # rect_roi = cv.bitwise_and(binary_img, binary_img, mask=rect_roi)
    # 寻找边缘上的轮廓
    _, contours, _ = cv.findContours(rect_roi, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # 遍历轮廓
    if contours is not None:
        contours = sorted(contours, key=cv.contourArea, reverse=True)

        for contour in contours:
            areas = cv.contourArea(contour)
            if min_area <= areas <= max_area:
                # 计算图像矩
                M = cv.moments(contour)
                # 找圆心
                if M["m00"] != 0:
                    # 计算圆心坐标
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    state = color

                    # 使用拟合椭圆更精准
                    # if len(contour) >= 300:
                    #     ellipse = cv.fitEllipse(contour)
                    #     (center_fit, axes_fit, angle_fit) = ellipse
                            
                    #     # 坐标
                    #     x = center_fit[0] - screenwidth / 2
                    #     y = screenheight / 2 - center_fit[1]
                        
                    #     # 画圆心
                    #     cv.circle(frame, (np.int(center_fit[0]), np.int(center_fit[1])), 5, colorTarget, -1)
                    #     # 画圆环
                    #     cv.circle(frame, (np.int(center_fit[0]), np.int(center_fit[1])), np.int(min(axes_fit)), colorTarget, 2)
                    #     print("Fitting ellipse")
                        
                    #     return x, y, state
                    # 如果没有用拟合椭圆的方法，那么就直接找重心
                    # 求出坐标系中的位置
                    x = cX - screenwidth / 2
                    y = screenheight / 2 - cY
                    # 绘制质心
                    cv.circle(frame, (np.int(cX), np.int(cY)), 5, colorTarget, -1)
                    # 绘制轮廓
                    cv.drawContours(frame, [contour], -1, colorTarget, 2)
                    return x, y, state
    return 255, 255, state

def empty():
    pass

def find_circle(frame):
    # 串口
    global ser
    # 模式
    global MAIN_MODE, CIRCLE_MODE
    
    x, y, color = 0, 0, 0

    if (CIRCLE_MODE == CIRCLE_DEBUG): # 调试模式
        h_min = cv.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv.getTrackbarPos("Val Max", "TrackBars")
        
        LookForCircle(frame, NOCOLOR, greenTarget, (h_min, s_min, v_min), (h_max, s_max, v_max), 0, 400000)
    elif (CIRCLE_MODE == MODE_DETECT_CIRCLE_RED): # 判断红色的圆
        x, y, color = LookForCircle(frame, RED, redTarget, 2000, 40000, lr1, hr1, lr2, hr2)
    elif (CIRCLE_MODE == MODE_DETECT_CIRCLE_BLUE): # 判断蓝色的圆
        x, y, color = LookForCircle(frame, BLUE, blueTarget, 2000, 40000, lb, hb)
    elif (CIRCLE_MODE == MODE_DETECT_CIRCLE_GREEN): # 判断绿色的圆
        x, y, color = LookForCircle(frame, GREEN, greenTarget, 2000, 40000, lg, hg)
    else:
        print('go to a bad result')

    cv.circle(frame, (np.int(screenwidth / 2), np.int(screenheight / 2)), 5, (0, 0, 255), -1)
    print(f'x:{x} y:{y} colormode:{CIRCLE_MODE} color:{color}')
    outdata = struct.pack('<bbbffbb', 0x21, 0, 0, x, y, color, 0x41)
    ser.write(outdata)

def main():
    global MAIN_MODE, CIRCLE_MODE
    
    # 创建摄像头的对象
    video = cv.VideoCapture(CAMERA)
    global screenwidth, screenheight
    video.set(cv.CAP_PROP_FRAME_WIDTH, screenwidth)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, screenheight)
    
    # 创建线程
    thread = threading.Thread(target=serial_thread)
    thread.start()
    
    if (CIRCLE_MODE == CIRCLE_DEBUG):
        cv.namedWindow("TrackBars")
        cv.resizeWindow("TrackBars", 640, 240)
        cv.createTrackbar("Hue Min", "TrackBars", 74, 179, empty)
        cv.createTrackbar("Hue Max", "TrackBars", 112, 179, empty)
        cv.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
        cv.createTrackbar("Sat Max", "TrackBars", 64, 255, empty)
        cv.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
        cv.createTrackbar("Val Max", "TrackBars", 34, 255, empty)
    
    while (True):
        print(f'{CIRCLE_MODE}')
        if (CIRCLE_MODE == MODE_DETECT_CIRCLE_NOCOCLR):
            continue

        ret, frame = video.read()

        if (ret == True):            
            find_circle(frame)
            
            cv.imshow('frame', frame)
            
            if cv.waitKey(1) & 0xff == ord('q'):
                break


    thread.join()  # 等待串口接收线程结束
    video.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
