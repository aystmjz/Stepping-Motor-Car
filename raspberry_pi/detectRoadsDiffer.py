import cv2 as cv
import numpy as np
import math
ESC = 27
# 摄像头选择
CAMERA = 1

lgr = np.array([0, 105, 176], dtype=np.uint8)
hgr = np.array([179, 255, 255], dtype=np.uint8)

# 霍夫变换
HoughThreshold = 136

min_angle = 30
max_angle = 150

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
    x_max = 480
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

def main():
    # 导入全局变量
    global illum_lower, illum_upper, illum_alpha, illum_beta

    # 创建摄像头的对象
    video = cv.VideoCapture(CAMERA)

    while (True):
        ret, frame = video.read()
        screenheight, screenwidth = frame.shape[:2]  # 屏幕高度和宽度

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

            # 霍夫变换函数
            Houghlines = cv.HoughLines(binary_image, 1, np.pi / 180, HoughThreshold)

            vlx, vly, vrx, vry = -1, -1, -1, -1

            lx = 0
            ly = 0
            rx = 0
            ry = 0
            linedegreses = 0
            p2ldist = 0

            if Houghlines is not None:
                for rho, theta in Houghlines[:, 0]:
                    # 此处貌似得出的结果是x轴向上，y轴向右的结果
                    angle_degrees = np.degrees(theta)
                    # 转换一下坐标系（左下角是原点，从左到右为x，从下到上为y）
                    if (angle_degrees >= 0) and (angle_degrees <= 90):
                        angle_degrees = 90 - angle_degrees
                    else:
                        angle_degrees = 270 - angle_degrees
                    # 指定角度的直线不画

                    # 竖向角度的线
                    if (angle_degrees > min_angle) and (angle_degrees < max_angle):
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

                        # cv.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
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

                    # 横向角度的线
                    else:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))

                        # 计算斜率和截距
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
                        # print(f"({lx}, {linear_mapping(screenheight - np.uint8(ly))}), "
                        #       f"({rx}, {linear_mapping(screenheight - np.uint8(ry))}),"
                        #       f" {linedegreses}, "
                        #       f"{p2ldist}")

                        if (ly < 0):
                            ly = 0
                        if (ry < 0):
                            ry = 0

                        # print(f"({linear_mapping(screenheight - ly)}, {linear_mapping(screenheight - ry)})")
                        continue

            clx, cly = -1, -1
            if (vlx != -1) and (vly != -1) and (vrx != -1) and (vry != -1):
                intersection = calculate_intersection_point(vlx, vly, vrx, vry, lx, ly, rx, ry)
                if intersection is not None:
                    clx, cly = intersection
                    print(clx, cly)

            print(f'x:{x} y:{y} colormode:{CIRCLE_MODE} color:{color}')
            outdata = struct.pack('<bbbffbb', 0x21, ly, ry, 0, 0, 0, 0x41)
            ser.write(outdata)


        cv.imshow('original image', frame)
        cv.imshow('binary_image', binary_image)

        if (cv.waitKey(1) == 27):
            break

    video.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()