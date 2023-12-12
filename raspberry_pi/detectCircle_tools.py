import cv2

import numpy as np



def stackImages(scale, imgArray):

  '''

  图像堆栈，可缩放，按列表排列，不受颜色通道限制

  '''

  rows = len(imgArray)

  cols = len(imgArray[0])

  rowsAvailable = isinstance(imgArray[0], list)

  width = imgArray[0][0].shape[1]

  height = imgArray[0][0].shape[0]

  if rowsAvailable:

    for x in range(0, rows):

      for y in range(0, cols):

        if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:

          imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)

        else:

          imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),

                        None, scale, scale)

        if len(imgArray[x][y].shape) == 2:

          imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)

    imageBlank = np.zeros((height, width, 3), np.uint8)

    hor = [imageBlank]*rows

    hor_con = [imageBlank]*rows

    for x in range(0, rows):

      hor[x] = np.hstack(imgArray[x])

    ver = np.vstack(hor)

  else:

    for x in range(0, rows):

      if imgArray[x].shape[:2] == imgArray[0].shape[:2]:

        imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)

      else:

        imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)

      if len(imgArray[x].shape) == 2:

        imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)

    hor = np.hstack(imgArray)

    ver = hor

  return ver



def empty(a):

  pass



cap = cv2.VideoCapture(0)
new_width = 320
new_height = 240
cap.set(cv2.CAP_PROP_FRAME_WIDTH, new_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, new_width)

cv2.namedWindow("TrackBars")

cv2.resizeWindow("TrackBars", 640, 240)

cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)

cv2.createTrackbar("Hue Max", "TrackBars", 0, 179, empty)

cv2.createTrackbar("Sat Min", "TrackBars", 43, 255, empty)

cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)

cv2.createTrackbar("Val Min", "TrackBars", 46, 255, empty)

cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

# 专门为红色准备
cv2.createTrackbar("eHue Min", "TrackBars", 156, 179, empty)

cv2.createTrackbar("eHue Max", "TrackBars", 180, 179, empty)

cv2.createTrackbar("eSat Min", "TrackBars", 43, 255, empty)

cv2.createTrackbar("eSat Max", "TrackBars", 255, 255, empty)

cv2.createTrackbar("eVal Min", "TrackBars", 46, 255, empty)

cv2.createTrackbar("eVal Max", "TrackBars", 255, 255, empty)



while True:

  success, img=cap.read()

  imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

  h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")

  h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")

  s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")

  s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")

  v_min = cv2.getTrackbarPos("Val Min", "TrackBars")

  v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
  # 专门为红色准备
  eh_min = cv2.getTrackbarPos("eHue Min", "TrackBars")

  eh_max = cv2.getTrackbarPos("eHue Max", "TrackBars")

  es_min = cv2.getTrackbarPos("eSat Min", "TrackBars")

  es_max = cv2.getTrackbarPos("eSat Max", "TrackBars")

  ev_min = cv2.getTrackbarPos("eVal Min", "TrackBars")

  ev_max = cv2.getTrackbarPos("eVal Max", "TrackBars")

  lower = np.array([h_min, s_min, v_min])
  upper = np.array([h_max, s_max, v_max])

  mask = cv2.inRange(imgHSV, lower, upper)
  # 转为红色准备的
  mask2 = cv2.inRange(imgHSV, np.array([eh_min, es_min, ev_min]), np.array([eh_max, es_max, ev_max]))
  mask = mask + mask2

  imgResult = cv2.bitwise_and(img, img, mask=mask)



  try:

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur_img = cv2.GaussianBlur(gray, (3, 3), 0)



    circles = cv2.HoughCircles(image=blur_img, method=cv2.HOUGH_GRADIENT,

                  dp=1,

                  minDist=200, # 两个圆之间圆心的最小距离.如果太小的，多个相邻的圆可能被错误地检测成了一个重合的圆。反之，这参数设置太大，某些圆就不能被检测出来。

                  param1=100,

                  param2=120,

                  minRadius=0, # 圆半径的最小值

                  maxRadius=20) # 圆半径的最大值

    circles = np.uint16(np.around(circles))



    for i in circles[0, :]:

      # 画圆

      cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 255), 5)

      # 画圆心

      cv2.circle(img, (i[0], i[1]), 2, (0, 255, 255), 3)

      print('圆心坐标为（%.2f,%.2f）' % (i[0], i[1]))

      a = str(i[0])

      b = str(i[1])



      img = cv2.putText(img, "(" + a + " ," + b + ")", (i[0], i[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, )



  except:

    print('无法识别到圆')

    imgStack = stackImages(0.6, ([gray, imgHSV, img], [mask, imgResult, img]))

  imgStack = stackImages(0.6, ([gray, imgHSV, img], [mask, imgResult, img]))

  cv2.imshow("Stack Images",imgStack)



  if cv2.waitKey(1) & 0xff == ord('q'):

    break
