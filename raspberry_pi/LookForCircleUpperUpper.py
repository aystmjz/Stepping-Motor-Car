import cv2 as cv
import numpy as np
import serial
import struct
ESC = 27

COMPUTER_SCREEN = 0
VIDEO_SCREEN = 1

CAMERA = 0

# 串口
usart = '/dev/ttyS0'
bitrate = 9600
ser = serial.Serial(usart, bitrate, timeout=5)

# 模式选择
MODE_SCAN_LINE = 0              # 扫线模式
MODE_DETECT_CIRCLE_RED = 1      # 找红色的圆
MODE_DETECT_CIRCLE_BLUE = 2     # 找蓝色的圆
MODE_DETECT_CIRCLE_GREEN = 3    # 找绿色的圆


MODE = MODE_SCAN_LINE

# 阈值部分
lg = np.array([74, 31, 78], dtype=np.uint8) # 绿色阈值下界
hg = np.array([95, 255, 255], dtype=np.uint8) # 绿色阈值上界

lb = np.array([102, 58, 27], dtype=np.uint8) # 蓝色阈值下界
hb = np.array([149, 228, 255], dtype=np.uint8)# 蓝色阈值上界

lr1 = np.array([0, 43, 46], dtype=np.uint8) #红色阈值下界1
hr1 = np.array([10, 255, 255], dtype=np.uint8) #蓝色阈值上界1
lr2 = np.array([156, 43, 46], dtype=np.uint8) #红色阈值下界2
hr2 = np.array([180, 255, 255], dtype=np.uint8) #红色阈值上界2

# 颜色部分
blueTarget = (255, 0, 0)
greenTarget = (0, 255, 0)
redTarget = (0, 0, 255)

NOCOLOR = 0
RED = 1
GREEN = 2
BLUE = 3

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

def LookForCircle(frame, color, colorTarget, lower, upper, elower=None, eupper=None, minRadius=0, maxRadius=100):
    global screenwidth, screenheight
    pointx, pointy = 0, 0
    state = NOCOLOR

    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    rect_roi = 0

    # 二值化
    if (elower is not None and eupper is not None) or (elower is not None and eupper is not None):
        rect_roi1 = cv.inRange(hsv_frame, lower, upper)
        rect_roi2 = cv.inRange(hsv_frame, elower, eupper)
        rect_roi = rect_roi1 + rect_roi2
    else:
        rect_roi = cv.inRange(hsv_frame, lower, upper)

    # rect_roi = cv.GaussianBlur(rect_roi, (5, 5), 0)
    # # # 定义核大小（腐蚀和膨胀的窗口大小）
    # # kernel = np.ones((5, 5), np.uint8)  # 定义5x5的矩形核
    # # # 执行开运算
    # # rect_roi = cv.morphologyEx(rect_roi, cv.MORPH_OPEN, kernel)

    # 找出轮廓
    _, circles, _ = cv.findContours(rect_roi, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    countx, county = 0, 0
    cnt = 0

    for circle in circles:
        if len(circle) >= 200:  # 添加检查，至少需要5个点才能拟合椭圆
            rrt = cv.fitEllipse(circle)
            x, y = rrt[0]

            pointx = x - screenwidth / 2
            pointy = screenheight / 2 - y
            
            # 积累差值
            countx += pointx
            county += pointy
            cnt += 1
    
    if cnt != 0:
        state = color
        pointx, pointy = countx / cnt, county / cnt
        
        # 画出圆
        # 画圆心
        cv.circle(frame, (np.int(screenwidth / 2 + countx / cnt), np.int(screenheight / 2 - county / cnt)), 1, colorTarget, 2)
    
    return pointx, pointy, state

def main():
    # 创建摄像头的对象
    video = cv.VideoCapture(CAMERA)
    global screenwidth, screenheight
    video.set(cv.CAP_PROP_FRAME_WIDTH, screenwidth)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, screenheight)
    while (True):
        ret, frame = video.read()

        if (ret == True):
            # data = LookForCircle(frame, BLUE, blueTarget, lb, hb)
            data = LookForCircle(frame, GREEN, greenTarget, lg, hg)
            cv.circle(frame, (160, 120), 1, (0, 0, 255), 2)
            
            outdata = struct.pack('<bbbffbb', 0x21, 0, 0, np.float32(data[0]), np.float32(data[1]), data[2], 0x41)
            print(f'x:{data[0]} y:{data[1]} color:{data[2]}')
            ser.write(outdata)
        
        cv.imshow("frame", frame)

        if (cv.waitKey(1) == 27):
            break

    video.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
