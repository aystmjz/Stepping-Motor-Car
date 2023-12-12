import math
import cv2 as cv
import numpy as np
import serial
import struct
import threading
import time
import VL53L0X

ESC = 27

VIDEO_LINE = 2
VIDEO_CIRCLE = 0

CAMERA = VIDEO_LINE

# 串口相关
usart = '/dev/serial0'
bitrate = 9600
ser = serial.Serial(usart, bitrate, timeout=None)

# 模式选择
MODE_SCAN_LINE = 0  # 扫线模式
MODE_DETECT_CIRCLE = 1 # 找圆模式

MODE_DETECT_CIRCLE_NOCOCLR = 0 # 没有颜色
MODE_DETECT_CIRCLE_RED = 1  # 找红色的圆
MODE_DETECT_CIRCLE_BLUE = 3  # 找蓝色的圆
MODE_DETECT_CIRCLE_GREEN = 2  # 找绿色的圆
CIRCLE_DEBUG = 5
MAIN_MODE = MODE_SCAN_LINE
CIRCLE_MODE = MODE_DETECT_CIRCLE_NOCOCLR

#----------------------------------------寻圆部分---------------------------------------------------
# 阈值部分
lg = np.array([63, 56, 58], dtype=np.uint8)  # 绿色阈值下界
hg = np.array([94, 255, 255], dtype=np.uint8)  # 绿色阈值上界

lb = np.array([100, 56, 51], dtype=np.uint8) # 蓝色阈值下界
hb = np.array([133, 255, 255], dtype=np.uint8)# 蓝色阈值上界

# lb = np.array([86, 68, 117], dtype=np.uint8) # 蓝色阈值下界
# hb = np.array([148, 255, 255], dtype=np.uint8)# 蓝色阈值上界

# H改为0防止识别到肤色
lr1 = np.array([0, 43, 46], dtype=np.uint8)  # 红色阈值下界1
hr1 = np.array([0, 255, 255], dtype=np.uint8)  # 蓝色阈值上界1
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

#----------------------------------------扫线部分---------------------------------------------------
lgr = np.array([0, 121, 136], dtype=np.uint8)
hgr = np.array([179, 255, 255], dtype=np.uint8)

# 霍夫变换
HoughThreshold = 50

min_angle = 50
max_angle = 130

def nothing(x):
    pass

def calculate_intersection_point(lx1, ly1, rx1, ry1, lx2, ly2, rx2, ry2):
    # 计算直线1和直线2的方向向量
    d1x, d1y = rx1 - lx1, ry1 - ly1
    d2x, d2y = rx2 - lx2, ry2 - ly2

    # 计算向量叉积
    cross_product = d1x * d2y - d1y * d2x

    if cross_product == 0:
        # 直线平行或重合，没有交点
        return None
    else:
        # 计算参数 t 和 s
        t = ((lx2 - lx1) * d2y - (ly2 - ly1) * d2x) / cross_product

        # 计算交点坐标
        x = lx1 + t * d1x
        y = ly1 + t * d1y

        return x, y

def check_region(image, region):
    # 提取指定区域的图像块
    x, y, w, h = region
    region_image = image[y:y+h, x:x+w]

    # 计算图像块中像素值等于指定值的数量
    count = cv.countNonZero(region_image)
    ratio = 0.4
    threshold = np.around(w * h * ratio)
    # 判断是否所有像素都满足条件
    if count > threshold:
        # print("1 ", end='')
        return 1
    else:
        # print("0 ", end='')
        return 0

def checkAllregions(image, object):
    res = int(0)
    for i in range(0, object.num):
        res |= check_region(image, object.getRect(i)) << i
    return res

# 图像坐标部分
class RectArray:

    def __init__(self, start_x, start_y, w, h, num):
        self.start_x = start_x
        self.start_y = start_y
        self.w = w
        self.h = h
        self.num = num
        self.object = [[0, 0] for i in range(num)]

    # 取得rect[index]的信息
    def getRect(self, index):
        return (self.object[index][0], self.object[index][1], self.w, self.h)


    # 随意画
    def drawRect(self, img, wgap, hgap, color=(0, 0, 255), thinkness=None, linetype=None, shift=None):
        for i in range(0, self.num):
            self.object[i][0] = self.start_x + i * (self.w + wgap)
            self.object[i][1] = self.start_y + i * (self.h + hgap)

            spt = (self.object[i][0], self.object[i][1])
            ept = (self.object[i][0] + self.w, self.object[i][1] + self.h)
            color = (color[0], color[1], color[2])
            cv.rectangle(img, spt, ept, color, thinkness, linetype, shift)

    # 竖直
    def vertical_drawRect(self, img, gap, color=(0, 0, 255), thinkness=None, linetype=None, shift=None):
        for i in range(0, self.num):
            self.object[i][0] = self.start_x
            self.object[i][1] = self.start_y + i * (self.h + gap)

            spt = (self.object[i][0], self.object[i][1])
            ept = (self.object[i][0] + self.w, self.object[i][1] + self.h)
            color = (color[0], color[1], color[2])
            cv.rectangle(img, spt, ept, color, thinkness, linetype, shift)

    # 水平
    def horizontal_drawRect(self, img, gap, color=(0, 0, 255), thinkness=None, linetype=None, shift=None):
        for i in range(0, self.num):
            self.object[i][0] = self.start_x + i * (self.h + gap)
            self.object[i][1] = self.start_y

            spt = (self.object[i][0], self.object[i][1])
            ept = (self.object[i][0] + self.w, self.object[i][1] + self.h)
            color = (color[0], color[1], color[2])
            cv.rectangle(img, spt, ept, color, thinkness, linetype, shift)


def adjust_color_balance(image, hue_shift, sat_scale, val_scale):
    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # 调整色调、饱和度和亮度
    hsv_image[:, :, 0] = (hsv_image[:, :, 0] + hue_shift) % 180
    hsv_image[:, :, 1] = np.clip(hsv_image[:, :, 1] * sat_scale, 0, 255)
    hsv_image[:, :, 2] = np.clip(hsv_image[:, :, 2] * val_scale, 0, 255)

    # 将图像从HSV颜色空间转换回BGR颜色空间
    result_image = cv.cvtColor(hsv_image, cv.COLOR_HSV2BGR)

    return result_image

def Image_equalization(hsv_image):
    h, s, v = cv.split(hsv_image)
    equ_v = cv.equalizeHist(v)
    image_hsv_equalized = cv.merge((h, s, equ_v))
    return image_hsv_equalized

def getDist_P2L(PointP, Pointa, Pointb):
    """计算点到直线的距离
        PointP：定点坐标
        Pointa：直线a点坐标
        Pointb：直线b点坐标
    """
    # 求直线方程
    A = 0
    B = 0
    C = 0
    A = Pointa[1] - Pointb[1]
    B = Pointb[0] - Pointa[0]
    C = Pointa[0] * Pointb[1] - Pointa[1] * Pointb[0]
    # 代入点到直线距离公式
    distance = 0
    distance = (A * PointP[0] + B * PointP[1] + C) / math.sqrt(A * A + B * B)

    return distance

# 线性映射
def linear_mapping(x):
    x_min = 0
    x_max = 240
    y_min = 0
    y_max = 200

    y = (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min
    return y

open_kernel = np.ones((3, 3), np.uint8)
Gauss_kernel = (5, 5)

# 调整色彩平衡，这里示例中的参数值可以根据需要进行调整
hue_shift = 30  # 色调偏移（取值范围：0-180，正数表示顺时针偏移，负数表示逆时针偏移）
sat_scale = 1.5  # 饱和度缩放因子（大于1增加饱和度，小于1减少饱和度）
val_scale = 1.2  # 亮度缩放因子（大于1增加亮度，小于1减少亮度）

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
        ser.flushInput()
        serialdata = str(serialdata.decode())
        
        if (len(serialdata) == 4) and (serialdata[0] == 'E') and (serialdata[3] == 'F'):
            usartbuf[0], usartbuf[1] = serialdata[1], serialdata[2]
            
        seriallock.release()
            
        MAIN_MODE = int(usartbuf[0])
        CIRCLE_MODE = int(usartbuf[1])
        
        print(f'{MAIN_MODE} {CIRCLE_MODE}')
        
def swap_num(x1, x2):
    x1, x2 = x2, x1
    return x1, x2
        
def LookForCircle(frame, color, colorTarget, min_area, max_area, low, high, elower=None, eupper=None):
    global screenwidth, screenheight
    state = NOCOLOR
    
    # 确定圆的坐标
    cx_up, cx_up, x_up, y_up, state_up = -1, -1, 255, 255, NOCOLOR
    cx_down, cy_down, x_down, y_down, state_down = -1, -1, 255, 255, NOCOLOR
    
    # 颜色阈值分割
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # 二值化
    if (elower is not None and eupper is not None) or (elower is not None and eupper is not None):
        rect_roi1 = cv.inRange(hsv_frame, low, high)
        rect_roi2 = cv.inRange(hsv_frame, elower, eupper)
        rect_roi = rect_roi1 + rect_roi2
    else:
        rect_roi = cv.inRange(hsv_frame, low, high)
        
    cv.imshow('color_mask', rect_roi)

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
        # 记录找到的轮廓数量

        cnt = 0
        state = NOCOLOR
        
        for contour in contours:
            areas = cv.contourArea(contour)
            if min_area <= areas <= max_area:
                # 计算图像矩
                M = cv.moments(contour)
                # 找圆心
                cnt += 1
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
                    
                    # 超出阈值范围则代表数据传输错误
                    if (np.fabs(x) > screenwidth / 2) or (np.fabs(y) > screenheight / 2):
                        x, y = 255, 255
                        state = NOCOLOR
                    
                    if (cnt == 1):
                        cx_down = cX
                        cy_down = cY
                        x_down = x
                        y_down = y
                        state_down = state
                    
                    elif (cnt == 2):
                        cx_up = cX
                        cy_up = cY
                        x_up = x
                        y_up = y
                        state_up = state
                        
                        if (y_up < y_down):
                            cx_up, cx_down = swap_num(cx_up, cx_down)
                            cy_up, cy_down = swap_num(cy_up, cy_down)
                            x_up, x_down = swap_num(x_up, x_down)
                            y_up, y_down = swap_num(y_up, y_down)
                            state_up, state_down = swap_num(state_up, state_down)
                        
                        cv.putText(frame, "up", (cx_up, cy_up), cv.FONT_HERSHEY_COMPLEX, 2.0, (100, 200, 200), 5)
                        cv.putText(frame, "down", (cx_down, cy_down), cv.FONT_HERSHEY_COMPLEX, 2.0, (100, 200, 200), 5)
                        # print(2)
                        return x_up, y_up, state_up, x_down, y_down, state_down 
        if (cnt == 1):
            # print(1)
            return 255, 255, NOCOLOR, x_down, y_down, state_down
    
    # print(3)
    return 255, 255, NOCOLOR, 255, 255, NOCOLOR

def empty():
    pass


# 寻圆模式
def scanCircle():
    # 开启测距
    # tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
    # tof.open()
    # tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

    # 串口
    global ser
    # 模式
    global MAIN_MODE, CIRCLE_MODE
    global CAMERA
    # 创建摄像头的对象
    video = cv.VideoCapture(CAMERA)
    global screenwidth, screenheight
    video.set(cv.CAP_PROP_FRAME_WIDTH, screenwidth)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, screenheight)
    # 创建窗口
    cv.namedWindow("Circle", cv.WINDOW_AUTOSIZE)
    cv.resizeWindow("Circle", 320, 240)
    cv.moveWindow('Circle', 0, 0)
    
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
        if (MAIN_MODE == MODE_DETECT_CIRCLE) and (CIRCLE_MODE == MODE_DETECT_CIRCLE_NOCOCLR):
            continue

        ret, frame = video.read()

        if (ret == True):            
            # 确定圆的坐标
            cx_up, cx_up, x_up, y_up, color_up = -1, -1, 255, 255, NOCOLOR
            cx_down, cy_down, x_down, y_down, color_down = -1, -1, 255, 255, NOCOLOR

            if (CIRCLE_MODE == CIRCLE_DEBUG): # 调试模式
                h_min = cv.getTrackbarPos("Hue Min", "TrackBars")
                h_max = cv.getTrackbarPos("Hue Max", "TrackBars")
                s_min = cv.getTrackbarPos("Sat Min", "TrackBars")
                s_max = cv.getTrackbarPos("Sat Max", "TrackBars")
                v_min = cv.getTrackbarPos("Val Min", "TrackBars")
                v_max = cv.getTrackbarPos("Val Max", "TrackBars")
                
                LookForCircle(frame, NOCOLOR, greenTarget, (h_min, s_min, v_min), (h_max, s_max, v_max), 0, 400000)
            elif (CIRCLE_MODE == MODE_DETECT_CIRCLE_RED): # 判断红色的圆
                x_up, y_up, color_up, x_down, y_down, color_down = LookForCircle(frame, RED, redTarget, 3000, 80000, lr1, hr1, lr2, hr2)
                
                # h = tof.get_distance()
                
                # if h > 0:
                #     # 736
                #     print("%dmm, %dp, %.2fmm" % (h, x_down, x_down * h / 423.25))
            elif (CIRCLE_MODE == MODE_DETECT_CIRCLE_BLUE): # 判断蓝色的圆
                x_up, y_up, color_up, x_down, y_down, color_down = LookForCircle(frame, BLUE, blueTarget, 3000, 80000, lb, hb)
            elif (CIRCLE_MODE == MODE_DETECT_CIRCLE_GREEN): # 判断绿色的圆
                x_up, y_up, color_up, x_down, y_down, color_down = LookForCircle(frame, GREEN, greenTarget, 3000, 80000, lg, hg)
            else: # 切换了模式
                if (MAIN_MODE == MODE_SCAN_LINE):
                    break

            cv.circle(frame, (np.int(screenwidth / 2), np.int(screenheight / 2)), 5, (0, 0, 255), -1)
            
            # cv.circle(frame, (np.int(screenwidth / 2), np.int(screenheight / 2)), 5, (0, 0, 255), -1)
            # cv.line(frame, (0, 120), (320, 120), (0, 0, 255))
            # cv.line(frame, (160, 0), (160, 240), (0, 0, 255))
            # print(f'x:{x} y:{y} colormode:{CIRCLE_MODE} color:{color}')
            outdata = struct.pack('<bbbffbb', 0x21, 0, 0, x_down, y_down, color_down, 0x41)
            ser.write(outdata)
            
            cv.imshow('Circle', frame)
            
            if (cv.waitKey(1) == 27):
                break
            elif (MAIN_MODE == MODE_SCAN_LINE):
                break
                

    # tof.stop_ranging()
    # tof.close()
    video.release()
    cv.destroyAllWindows()
    
    
# 巡线模式
def scanLine():
    global CAMERA
    # 创建摄像头的对象
    video = cv.VideoCapture(CAMERA)
    global screenwidth, screenheight
    video.set(cv.CAP_PROP_FRAME_WIDTH, screenwidth)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, screenheight)
    # 创建窗口
    cv.namedWindow("Line", cv.WINDOW_AUTOSIZE)
    cv.resizeWindow("Line", 320, 240)
    cv.moveWindow('Line', 0, 0)

    while (True):
        ret, frame = video.read()

        # Sobel算子
        # # 使用Sobel算子计算图像边界梯度
        # gradient_x = cv.Sobel(frame, cv.CV_64F, 1, 0, ksize=3)
        # gradient_y = cv.Sobel(frame, cv.CV_64F, 0, 1, ksize=3)
        # # 计算总的梯度
        # gradient_magnitude = np.sqrt(gradient_x ** 2 + gradient_y ** 2)
        # gradient_magnitude = cv.convertScaleAbs(gradient_magnitude)

        # Scharr算子
        # 使用Scharr算子计算图像边界梯度
        gradient_x = cv.Scharr(frame, cv.CV_64F, 1, 0)
        gradient_y = cv.Scharr(frame, cv.CV_64F, 0, 1)

        # 计算总的梯度
        gradient_magnitude = np.sqrt(gradient_x ** 2 + gradient_y ** 2)

        # 可选：将梯度图像转换为无符号8位整数类型并显示
        gradient_magnitude = cv.convertScaleAbs(gradient_magnitude)

        # 颜色空间转换为HSV
        hsv_image = cv.cvtColor(gradient_magnitude, cv.COLOR_BGR2HSV)


        if (ret == True):
            # 二值化
            binary_image = cv.inRange(hsv_image, lgr, hgr)
            # binary_image = cv.morphologyEx(binary_image, cv.MORPH_OPEN, (5, 5), iterations=2)

            # 霍夫变换函数
            Houghlines = cv.HoughLines(binary_image, 1, np.pi / 180, HoughThreshold)
            # Houghlines = cv.HoughLines(binary_image, 1, np.pi / 180, HoughThreshold, minLineLength=10, maxLineGap=5)

            vlx, vly, vrx, vry = -1, -1, -1, -1

            lx = 0
            ly = 0
            rx = 0
            ry = 0
            linedegreses = 0
            p2ldist = 0

            # 判断横竖直线
            vflag = False
            hflag = False
            if Houghlines is not None:
                for rho, theta in Houghlines[:, 0]:
                    # 线已经被找出来了，可以退出循环了
                    if (vflag == True) and (hflag == True):
                        break
                    # 此处貌似得出的结果是x轴向上，y轴向右的结果
                    angle_degrees = np.degrees(theta)
                    # 转换一下坐标系（左下角是原点，从左到右为x，从下到上为y）
                    if (angle_degrees >= 0) and (angle_degrees <= 90):
                        angle_degrees = 90 - angle_degrees
                    else:
                        angle_degrees = 270 - angle_degrees
                    # 指定角度的直线不画

                    # 竖向角度的线
                    if (vflag == False) and (angle_degrees > min_angle) and (angle_degrees < max_angle):
                        vflag = True
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))

                        # 计算斜率和截距
                        slope = 0
                        intercept = 0

                        # 绘制直线
                        cv.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # 赋值
                        vlx = x1
                        vly = y1
                        vrx = x2
                        vry = y2

                        linedegreses = angle_degrees
                        # p2ldist = getDist_P2L((cx, cy), (lx, ly), (rx, ry))
                        # print(f"({lx}, {linear_mapping(screenheight - np.uint8(ly))}), "
                        #       f"({rx}, {linear_mapping(screenheight - np.uint8(ry))}),"
                        #       f" {linedegreses}, "
                        #       f"{p2ldist}")


                        if (vly < 0):
                            vly = 0
                        if (vry < 0):
                            vry = 0

                        # print(f"({linear_mapping(screenheight - vly)}, {linear_mapping(screenheight - ry)})")
                        continue
                    elif (hflag == False):
                        hflag = True
                        # 横向角度的线
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))

                        # 计算斜率和截距
                        if (x2 == x1):
                            continue
                        slope = (y2 - y1) / (x2 - x1)
                        intercept = y1 - slope * x1

                        x1_screen = 0
                        y1_screen = int(slope * x1_screen + intercept)

                        x2_screen = screenwidth - 1
                        y2_screen = int(slope * x2_screen + intercept)

                        # cv.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # 绘制直线
                        cv.line(frame, (x1_screen, y1_screen), (x2_screen, y2_screen), (0, 0, 255), 2)
                        # 赋值
                        lx = x1_screen
                        ly = y1_screen
                        rx = x2_screen
                        ry = y2_screen
                        linedegreses = angle_degrees
                        # p2ldist = getDist_P2L((cx, cy), (lx, ly), (rx, ry))
                        # print(f"({lx}, {linear_mapping(screenheight - ly)}), "
                        #       f"({rx}, {linear_mapping(screenheight - ry)}),"
                        #       f" {linedegreses}, "
                        #       f"{p2ldist}")

                        if (ly < 0):
                            ly = 0
                        if (ry < 0):
                            ry = 0



                        # print(f"({linear_mapping(screenheight - ly)}, {linear_mapping(screenheight - ry)})")
                        continue

            # 查找是否发现拐角
            corner_state = 0
            clx, cly = -1, -1
            if (vlx != -1) and (vly != -1) and (vrx != -1) and (vry != -1):
                intersection = calculate_intersection_point(vlx, vly, vrx, vry, lx, ly, rx, ry)
                if intersection is not None:
                    clx, cly = intersection
                    corner_state = 1
                    # print('yes, I find a corner')
            else:
                corner_state = 0
                # print('I cannot find corner')

            print(f'cornerstate: {corner_state} outxly:{np.uint8(linear_mapping(screenheight - ly))} outry:{np.uint8(linear_mapping(screenheight - ry))}')
            outdata = struct.pack('<bbBBb', 0x21, corner_state, np.uint8(linear_mapping(screenheight - ly)), np.uint8(linear_mapping(screenheight - ry)), 0x41)
            ser.write(outdata)
            
            # cv.circle(frame, (np.int(screenwidth / 2), np.int(screenheight / 2)), 5, (0, 0, 255), -1)
            # cv.line(frame, (0, 120), (320, 120), (0, 0, 255))
            # cv.line(frame, (160, 0), (160, 240), (0, 0, 255))
        
        cv.imshow('Line', frame)
        cv.imshow('binary_image', binary_image)

        if (cv.waitKey(1) == 27):
            break
        elif MAIN_MODE != MODE_SCAN_LINE:
            break

    video.release()
    cv.destroyAllWindows()

def main():
    global CAMERA
    # 创建线程
    thread = threading.Thread(target=serial_thread)
    thread.start()
    
    while (True):
        if MAIN_MODE == MODE_SCAN_LINE:
            CAMERA = VIDEO_LINE
            # 扫线模式
            scanLine()
        elif MAIN_MODE == MODE_DETECT_CIRCLE:
            CAMERA = VIDEO_CIRCLE
            # 寻圆模式
            scanCircle()
    
    thread.join()  # 等待串口接收线程结束

if __name__ == "__main__":
    main()
