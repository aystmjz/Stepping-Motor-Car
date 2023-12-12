import cv2 as cv
import numpy as np
import serial
import struct
ESC = 27
# 串口
usart = '/dev/ttyS0'
bitrate = 9600
ser = serial.Serial(usart, bitrate, timeout=5)

COMPUTER_SCREEN = 0
VIDEO_SCREEN = 1

CAMERA = 0

# 模式选择
MODE_SCAN_LINE = 0              # 扫线模式
MODE_DETECT_CIRCLE_RED = 1      # 找红色的圆
MODE_DETECT_CIRCLE_BLUE = 2     # 找蓝色的圆
MODE_DETECT_CIRCLE_GREEN = 3    # 找绿色的圆


MODE = MODE_SCAN_LINE

# 阈值部分
lg = np.array([26, 50, 61], dtype=np.uint8) # 绿色阈值下界
hg = np.array([95, 255, 255], dtype=np.uint8) # 绿色阈值上界

lb = np.array([103, 29, 46], dtype=np.uint8) # 蓝色阈值下界
hb = np.array([156, 255, 255], dtype=np.uint8)# 蓝色阈值上界

lr1 = np.array([0, 43, 46], dtype=np.uint8) #红色阈值下界1
hr1 = np.array([10, 255, 255], dtype=np.uint8) #蓝色阈值上界1
lr2 = np.array([156, 43, 46], dtype=np.uint8) #红色阈值下界2
hr2 = np.array([180, 255, 255], dtype=np.uint8) #红色阈值上界2

# 颜色部分
NOCOLOR = 0
RED = 1
GREEN = 2
BLUE = 3

blueTarget = (255, 0, 0)
greenTarget = (0, 255, 0)
redTarget = (0, 0, 255)
# 文本部分
greenText = 'Green Target'
blueText = 'Blue Target'
redText = 'Red Target'

# 文本位置
greenpos = (0, 50)
bluepos = (0, 70)
redpos = (0, 90)

# 阈值
INF = 100000

# 屏幕相关
screenwidth = 320
screenheight = 240

def LookForCircle(frame, colorTarget, lower, upper, elower=None, eupper=None, minRadius=0, maxRadius=100):
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    rect_roi = 0
    global screenwidth, screenheight
    
    pointx, pointy = -1, -1

    # 二值化
    if (elower is not None and eupper is not None) or (elower is not None and eupper is not None):
        rect_roi1 = cv.inRange(hsv_frame, lower, upper)
        rect_roi2 = cv.inRange(hsv_frame, elower, eupper)
        rect_roi = rect_roi1 + rect_roi2
    else:
        rect_roi = cv.inRange(hsv_frame, lower, upper)

    # 霍夫圆
    circles = cv.HoughCircles(rect_roi, cv.HOUGH_GRADIENT, 1, 100, param1=100, param2=40, minRadius=minRadius,
                              maxRadius=maxRadius)
    # 如果找到了圆
    if circles is not None:
        # 先四舍五入找出所有圆的坐标
        circles = np.uint16(np.around(circles))
        # 画出圆
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            
            pointx = center[0] - screenwidth / 2
            pointy = screenheight / 2 - center[1]
            
            # 画圆的轮廓
            cv.circle(frame, (np.int(center[0]), np.int(center[1])), radius, colorTarget, 2)
            # 画圆心
            cv.circle(frame, (np.int(center[0]), np.int(center[1])), 1, colorTarget, 2)
            print(f"{pointx}, {pointy}")
            return pointx, pointy
    return pointx, pointy

def main():
    # 创建摄像头的对象
    video = cv.VideoCapture(CAMERA)
    
    global screenwidth, screenheight
    video.set(cv.CAP_PROP_FRAME_WIDTH, screenwidth)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, screenheight)
    while (True):
        ret, frame = video.read()

        if (ret == True):
            # LookForCircle(frame, greenTarget, lg, hg)
            # LookForCircle(frame, redTarget, lr1, hr1, lr2, hr2)
            datarecv = LookForCircle(frame, blueTarget, lb, hb)
            if (datarecv[0] == -1) and (datarecv[1] == -1):
                outdata = struct.pack('bbbffbb', 0x21, 0, 0, datarecv[0], datarecv[1], NOCOLOR, 0x41)
            else:
                outdata = struct.pack('bbbffbb', 0x21, 0, 0, datarecv[0], datarecv[1], BLUE, 0x41)
            ser.write(outdata)
        cv.imshow("frame", frame)

        if (cv.waitKey(1) == 27):
            break

    video.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
