import cv2

def main():
    img = cv2.imread("./selected_armor_img/466.jpg")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, binary = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)
    gauss_img = cv2.GaussianBlur(binary, (5, 5), 0)

    cv2.imwrite("processed.jpg",gauss_img)


if __name__ == "__main__":
    main()

