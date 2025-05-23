'''
@Description: Code for doing camera calibration, the parameters depend on the calibration board.
@Author: Leyan Chen
@Time: 2025/5/22
'''

import cv2
import numpy as np
import glob

from anaconda_project.conda_manager import new_conda_manager

'''
寻找亚像素角点的参数
stopping condition：循环大于30或者误差0.001
'''
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

'''
棋盘模板规格
长宽角点个数，棋盘的顶点不会被计算
'''
# TODO: change
w = 11 # how many columns
h = 8 # how many rows

'''
世界坐标系中的棋盘格点
ex: (0,0,0), (1,0,0)
去掉Z，变为二维矩阵
'''
objp = np.zeros((w*h, 3),np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
objp = objp * 18.1 #18.1mm为每个ceil的边长

'''
棋盘格子角点的世界坐标和图像坐标对
'''
objpoints = [] # 世界坐标系的三位点
imgpoints = [] # 图像平面中的二维点

images = glob.glob('./chessImg/*.jpg') # path to the chess img
cnt = 0
for fname in images:
    img = cv2.imread(fname)
    # get the center pt of the img
    # get the size of the img
    h1 ,w1 = img.shape[0],img.shape[1]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    u,v = img.shape[:2]
    # 找到棋盘格的角点
    ret, corners = cv2.findChessboardCorners(gray, (w,h), None )
    # store those pts when finding enough pts
    if ret == True:
        print("cnt: \t" , cnt)
        cnt = cnt + 1
        #在圆角点的基础上寻找亚像素角点
        cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        #追加进入世界三维和平面二维中
        objpoints.append(objp)
        imgpoints.append(corners)

        # show all the corner points on the img
        cv2. drawChessboardCorners(img,(w,h),corners,ret)
        cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('findCorners', 640,480)
        cv2.imshow('findCorners',img)
        cv2.waitKey(200)
        if cnt > 100:
            # TODO:现在设置的最大训练图片数量 100
            break

cv2.destroyAllWindows()
print("calculating........")

#标定
ret, mtx, dist, rvecs, tvecs = \
    cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None,None)


print ("ret:", ret)
print ("mtx: \n", mtx)  #内参矩阵

print("dist 畸变值: \n" , dist) #畸变系数 distortion coefficient
print ("rvecs 旋转向量外参: \n", rvecs) #旋转向量
print ("tvecs平移向量外参:\n", tvecs) #平移向量外参

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist,(u,v),0,(u,v))

print("newcameramtx外参", newcameramtx)




cv2.destroyAllWindows()
