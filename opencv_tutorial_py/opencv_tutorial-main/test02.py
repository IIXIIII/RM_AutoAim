import cv2

image01 = cv2.imread("opencv_logo.jpg")
cv2.imshow("blue", image01[:,:,1])

gray = cv2.cvtColor(image01, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray" , gray)
cv2.waitKey()