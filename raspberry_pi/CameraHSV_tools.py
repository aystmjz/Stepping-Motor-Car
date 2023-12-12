import cv2 as cv
import numpy as np

HoughThreshold = 110

def nothing(x):
    pass

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
    image_hsv_equalized  = cv.merge((h, s, equ_v))
    return image_hsv_equalized


# 调整色彩平衡，这里示例中的参数值可以根据需要进行调整
hue_shift = 30  # 色调偏移（取值范围：0-180，正数表示顺时针偏移，负数表示逆时针偏移）
sat_scale = 1.5  # 饱和度缩放因子（大于1增加饱和度，小于1减少饱和度）
val_scale = 1.2  # 亮度缩放因子（大于1增加亮度，小于1减少亮度）


def main():
    # 创建窗口
    cv.namedWindow("Line", cv.WINDOW_AUTOSIZE)
    cv.resizeWindow("Line", 320, 240)
    cv.moveWindow('Line', 0, 0)
    
    cv.namedWindow("hsv_adjust")

    cv.createTrackbar("low_H", "hsv_adjust", 0, 179, nothing)
    cv.createTrackbar("high_H", "hsv_adjust", 179, 179, nothing)
    cv.createTrackbar("low_S", "hsv_adjust", 121, 255, nothing)
    cv.createTrackbar("high_S", "hsv_adjust", 255, 255, nothing)
    cv.createTrackbar("low_V", "hsv_adjust", 136, 255, nothing)
    cv.createTrackbar("high_V", "hsv_adjust", 255, 255, nothing)
    cv.createTrackbar("HoughThreshold", "hsv_adjust", 255, 1024, nothing)

    # 创建摄像头的对象
    video = cv.VideoCapture(0)
    while True:
        low_h = cv.getTrackbarPos("low_H", "hsv_adjust")
        high_h = cv.getTrackbarPos("high_H", "hsv_adjust")
        low_s = cv.getTrackbarPos("low_S", "hsv_adjust")
        high_s = cv.getTrackbarPos("high_S", "hsv_adjust")
        low_v = cv.getTrackbarPos("low_V", "hsv_adjust")
        high_v = cv.getTrackbarPos("high_V", "hsv_adjust")
        HoughThreshold = cv.getTrackbarPos("HoughThreshold", "hsv_adjust")
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
        hsv_frame = cv.cvtColor(gradient_magnitude, cv.COLOR_BGR2HSV)

        # 颜色平衡调整
        # frame = adjust_color_balance(frame, hue_shift, sat_scale, val_scale)

        # 颜色空间转换
        # hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # 颜色均衡化处理
        # hsv_frame = Image_equalization(hsv_frame)

        inRange_frame = cv.inRange(hsv_frame, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

        # 霍夫变换函数
        Houghlines = cv.HoughLines(inRange_frame, 1, np.pi / 180, HoughThreshold)

        if Houghlines is not None:
            for rho, theta in Houghlines[:, 0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        dia_frame = cv.dilate(inRange_frame, (5, 5), iterations=1)
        cv.imshow('dia_frame', dia_frame)
        cv.imshow('Line', frame)
        if (cv.waitKey(1) == 27):
            break



if __name__ == "__main__":
    main()