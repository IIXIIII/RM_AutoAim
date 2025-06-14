'''
@Description : Armor Detection pipline
'''
import itertools

import argparse
import itertools
import pathlib
from typing import List, Tuple

import cv2
import numpy as np

class DetectResult:
    def __init__(self, armor_xyxy: list, number: int) -> None:
        #bounding box
        self.armor_xyxy = armor_xyxy
        # detected number
        self.number = number

class ArmorDetector:
    def __init__(self, binary_threshold: float = 220.0) -> None:
        self.binary_threshold = binary_threshold

    def __pre_process_blue(self, bgr: np.ndarray):
        #颜色通道分离
        b,g,r = cv2.split(bgr)

        #二值化 Thresholding
        '''
        Blue
        对蓝色通道 b 做二值化处理。
        self.binary_threshold 是设定的阈值。
        255 是最大值（像素值大于阈值时设置为255，否则为0）。
        0 是 cv2.THRESH_BINARY 的枚举值。
        '''
        _, binary = cv2.threshold(b, self.binary_threshold, 255, 0)

        #高斯滤波 Gaussian Blur
        '''
        使用高斯滤波对二值图像 binary 进行平滑处理。
        kernel size = (5, 5)，sigmaX = 0 自动计算。
        '''
        gauss_img = cv2.GaussianBlur(binary, (5, 5), 0)

        return gauss_img

    def __pre_process_gray_thresh(self, bgr: np.ndarray):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, int(self.binary_threshold), 255, cv2.THRESH_BINARY)
        gauss_img = cv2.GaussianBlur(binary, (5, 5), 0)

        return gauss_img

    def __pre_process_red(self, bgr: np.ndarray):
        red = bgr[:, :, 2]  # BGR 中 R 是第 2 个通道

        # 对红色通道进行阈值处理：R > threshold → 白色（255），否则黑色（0）
        _, binary = cv2.threshold(red, self.binary_threshold, 255, cv2.THRESH_BINARY)
        gauss_img = cv2.GaussianBlur(binary, (5, 5), 0)
        return gauss_img

    '''
    从高斯处理后的图像中提取出所有“可能是灯条”的区域。
    输入：高斯图像 gauss_img
     ↓
    cv2.findContours 提取轮廓
     ↓
    按面积排序
     ↓
    遍历每个轮廓：
    → 求最小外接矩形
    → 如果 h / w >= 2，则保留
     ↓
    输出：符合条件的 [x, y, w, h] 列表
    '''
    def __find_light_bars(self,gauss_img:np.ndarray) -> list:
        contours, hierarchy = cv2.findContours(image=gauss_img, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        contours = list(contours)
        #对轮廓按照面积进行排序，从大到小
        contours.sort(key=lambda c: cv2.contourArea(c), reverse=True)

        res = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            # area sieve
            if (w * h <= 200):
                continue
            '''
            light bar setting:
            w: 0.5
            h: 2.5
            '''
            if (h / w >= 2 and h/w <= 6):
                res.append([x, y, w, h])

        return res

    '''
    Matches pairs of detected light bars to form potential armor areas.
    Return: list: A list of matched armor regions, where each region is represented as 
              [left_x, left_y, right_x, right_y].
              [x1,y1,x2,y2]
    '''
    def __find_armors(self, light_bars: list) -> list:
        #灯条匹配，两两组合，根据armor特征筛选
        armors = []
        for i in range (len(light_bars)):
            for j in range (i+1,len(light_bars)):
                x1, y1, w1, h1 = light_bars[i]
                x2, y2, w2, h2 = light_bars[j]
                '''
                correlation selection
                vertical
                '''
                if (y1 <= y2):
                    if ((y1+h1) <y2):
                        continue
                else:
                    if(y1>(y2+h2)):
                        continue

                '''
                intersection selection
                horizontal 
                '''
                if (x1<x2):
                    if (x2<x1+w1):
                        continue
                else:
                    if (x1< x2+w2):
                        continue

                '''
                armor selection 
                armor setting: 
                w: 6.5
                h: 2.5
                '''
                # area selection
                s1 = w1*h1
                s2 = w2*h2
                if not(s1/s2<=1.5 or s2/s1<=1.5):
                    continue

                # slop selection

                slop1 = h1/w1
                slop2 = h2/w2
                if not(slop1/slop2<=1.5 or slop2/slop1<=1.5):
                    continue
                '''
                distance between = 2.5 * len
                '''
                dist = abs(x1-x2)
                if not (dist<((h1+h2)/2)*4):
                    continue
                if not(dist>((h1+h2)/2)*0.4):
                    continue


                armors.append([x1, y1, x2+w2, y2+h2])

        return armors

    def __number_identification(self, bgr: np.ndarray, armor_xyxy:list) -> list:
        pass

    #RETURN THE LENGTH OF THE LIGHT BAR
    def detection(self, bgr:np.ndarray) -> list:
        guess_img = self.__pre_process(bgr)
        light_bar = self.__find_light_bars(guess_img)
        armors = self.__find_armors(light_bar)

        res =[]
        for armor_xyxy in armors:
            num = self.__number_identification(bgr,armor_xyxy)
            res.append(DetectResult(armor_xyxy,num))
        return armors

