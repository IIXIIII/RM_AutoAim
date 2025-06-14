import cv2
from det import ArmorDetector, DetectResult

def main():
    # 读取测试图像
    img = cv2.imread("./selected_armor_img/issue_red_001.jpg")

    if img is None:
        print("Failed to load image.")
        return

    # 创建装甲板检测器
    detector = ArmorDetector(200)

    gauss_img = detector._ArmorDetector__pre_process_red(img)
    cv2.imwrite("1_preprocessed.jpg", gauss_img)  # ← 输出预处理图像


    # Step 2: 找到灯条框
    light_bars = detector._ArmorDetector__find_light_bars(gauss_img)
    print(f"[Light Bars] Found {len(light_bars)}")
    print(light_bars)

    # Step 3: 在图上画出每个灯条框（绿色）
    img_with_bars = img.copy()
    for (x, y, w, h) in light_bars:
        cv2.rectangle(img_with_bars, (x, y), (x+w, y+h), (0, 255, 0), 1)

    # Step 4: 保存画框后的图像
    cv2.imshow("1_light_bars_detected.jpg", img_with_bars)
    #cv2.waitKey()
    cv2.imwrite("1_light_bars_detected.jpg", img_with_bars)

    # Step 5: 找到装甲板框
    armors = detector._ArmorDetector__find_armors(light_bars)
    print(f"[Armors] Found {len(armors)}")
    print(armors)

    # Step 6: 在图上画出每个装甲板（红色框）
    img_with_armors = img_with_bars.copy()
    for (x1, y1, x2, y2) in armors:
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        cv2.rectangle(img_with_armors, (x1, y1), (x2,y2), (0, 0, 255), 2)

    # Step 7: 保存最终图像
    cv2.imwrite("1_armors_detected.jpg", img_with_armors)


if __name__ == "__main__":
    main()
