import cv2 as cv

image01 = cv.imread("198.jpg")


gray = cv.cvtColor(image01, cv.COLOR_BGR2GRAY)
cv.imshow("01",gray)

cv.waitKey()