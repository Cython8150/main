# -*- coding:utf-8 -*-

import cv2
import numpy as np
#import keyboard
import cv2

"""
功能：读取一张图片，显示出来，转化为HSV色彩空间
     并通过滑块调节HSV阈值，实时显示
"""

##image = cv2.imread("8888881711099485/888888"+str(i)+".jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
#image = cv2.imread("8888881800.jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
#cv2.imshow("BGR", image)  # 显示图片

hsv_low = np.array([0, 0, 0])
hsv_high = np.array([0, 0, 0])

def cv_show(name,img):
    cv2.imshow(name,img)
    cv2.waitKey(0)


# 下面几个函数，写得有点冗余

def h_low(value):
    hsv_low[0] = value


def h_high(value):
    hsv_high[0] = value


def s_low(value):
    hsv_low[1] = value


def s_high(value):
    hsv_high[1] = value


def v_low(value):
    hsv_low[2] = value


def v_high(value):
    hsv_high[2] = value


cv2.namedWindow('image', cv2.WINDOW_NORMAL)

# H low：
#    0：指向整数变量的可选指针，该变量的值反映滑块的初始位置。
#  179：表示滑块可以达到的最大位置的值为179，最小位置始终为0。
# h_low：指向每次滑块更改位置时要调用的函数的指针，指针指向h_low元组，有默认值0。
# （此函数的原型应为void
# XXX(int,
#     void *); ，其中第一个参数是轨迹栏位置，第二个参数是用户数据（请参阅下一个参数）。如果回调是NULL指针，则不调用任何回调，而仅更新值。）
cv2.createTrackbar('H low', 'image', 0, 179, h_low)
cv2.createTrackbar('H high', 'image', 0, 179, h_high)
cv2.createTrackbar('S low', 'image', 0, 255, s_low)
cv2.createTrackbar('S high', 'image', 0, 255, s_high)
cv2.createTrackbar('V low', 'image', 0, 255, v_low)
cv2.createTrackbar('V high', 'image', 0, 255, v_high)

dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))  # 矩形结构
erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # 矩形结构

i = 210
def increment(i):
    i += 1
    print(i)

def decrement(i):
    i -= 1

def one():
    global i
    while True:
        #image = cv2.imread("24ck_blue/"+'888888'+str(i)+".jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
        #image = cv2.imread("img9/"+'test_image'+str(i)+".jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
        image = cv2.imread("/home/eric/undergraduate_thesis/build/image/HSV/" + str(1)+ ".jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
        cv2.imshow("BGR", image)  # 显示图片
        #cv_show('BGR', image)

        red_mask = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # BGR转HSV
        red_mask = cv2.inRange(red_mask, hsv_low, hsv_high)  # 通过HSV的高低阈值，提取图像部分区域
        red_mask=cv2.medianBlur(red_mask,9)
        #red_mask=cv2.dilate(red_mask,dilate_kernel)
        #red_mask=cv2.erode(red_mask,erode_kernel)
        cv2.imshow('dst', red_mask)
        #cv_show('BGR', image)
        #cv_show('dst',red_mask)
        #i+=1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def three():
    global i
    while(True):
        image = cv2.imread("24ss/"+'888888'+str(i)+".jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
        #image = cv2.imread("img105.jpg")  # 根据路径读取一张图片，opencv读出来的是BGR模式
        cv2.imshow("BGR", image)  # 显示图片
        #cv_show('BGR', image)

        red_mask = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # BGR转HSV
        red_mask = cv2.inRange(red_mask, hsv_low, hsv_high)  # 通过HSV的高低阈值，提取图像部分区域
        red_mask=cv2.medianBlur(red_mask,5)
        #red_mask=cv2.dilate(red_mask,dilate_kernel)
        #red_mask=cv2.erode(red_mask,erode_kernel)
        #cv2.imshow('dst', red_mask)
        #cv_show('BGR', image)
        cv_show('dst',red_mask)
        i+=1
        contours,h=cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)   #contours为蓝色轮廓
        if(len(contours)==1):
            cnt=contours[0]
            mj=cv2.contourArea(cnt)
            x_y,y_y,w_y,h_y=cv2.boundingRect(cnt)
            #img_12=cv2.rectangle(img_12,(x_y,y_y),(x_y+w_y,y_y+h_y),(255,255,255),2)
            s_y=w_y*h_y
            print(f'蓝色面积为:{mj}')
            print(f'蓝色轮廓数量为:{len(contours)}')
            print(f'蓝色最小外接矩形面积为:{s_y}')
            print(f'蓝色最小外接矩形上边界Y坐标为:{y_y}')
        elif(len(contours)>=2):
            #mj=cv2.contourArea(contours[0])
            cnt=contours[0]
            mj=cv2.contourArea(cnt)
            for num_contours in range(len(contours)):
                mj_0=cv2.contourArea(contours[num_contours])
                if(mj<mj_0):
                    mj=mj_0
                    cnt=contours[num_contours]

            x_y,y_y,w_y,h_y=cv2.boundingRect(cnt)
            s_y=w_y*h_y
            print(f'蓝色面积为:{mj}')
            print(f'蓝色轮廓数量为:{len(contours)}')
            print(f'蓝色最小外接矩形面积为:{s_y}')
            print(f'蓝色最小外接矩形上边界Y坐标为:{y_y}')
        elif(len(contours)==0):
            mj=0
            x_y=0
            y_y=96
            w_y=0
            h_y=0
            s_y=0
            print(f'蓝色面积为:{mj}')
            print(f'蓝色轮廓数量为:{len(contours)}')
            print(f'蓝色最小外接矩形面积为:{s_y}')
            print(f'蓝色最小外接矩形上边界Y坐标为:{y_y}')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# cv2.imshow('hsv', hsv)
one()
#three()

cv2.destroyAllWindows()