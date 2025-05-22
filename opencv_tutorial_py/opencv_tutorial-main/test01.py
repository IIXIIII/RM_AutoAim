import cv2

print(cv2.getVersionString())

image01 = cv2.imread("./opencv_logo.jpg")
print(image01.shape)

cv2.imshow("image0_t",image01)
cv2.waitKey()


