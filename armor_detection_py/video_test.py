import cv2
from Armor_Detection import ArmorDetector

def main():
    # 1. 打开输入 MP4 视频
    cap = cv2.VideoCapture("./video/vid_move_003.mp4")  # ← 换成你的视频路径
    if not cap.isOpened():
        print("Failed to open video.")
        return

    # 2. 获取视频参数
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS)

    # 3. 设置输出为 MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4 编码格式
    out = cv2.VideoWriter("output_processed.mp4", fourcc, fps, (width, height))

    # 4. 初始化检测器
    detector = ArmorDetector(150)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Step 1: 灰度+阈值处理
        gauss_img = detector._ArmorDetector__pre_process_gray_thresh(frame)

        # Step 2: 检测灯条
        light_bars = detector._ArmorDetector__find_light_bars(gauss_img)

        # Step 3: 检测装甲板
        armors = detector._ArmorDetector__find_armors(light_bars)

        # Step 4: 绘制绿色灯条、红色装甲
        for (x, y, w, h) in light_bars:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
        for (x1, y1, x2, y2) in armors:
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Step 5: 写入新帧到 MP4 视频
        out.write(frame)

        # 实时预览（可选）
        cv2.imshow("Armor Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 6. 清理资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print("✅ Done! Output saved to output_processed.mp4")

if __name__ == "__main__":
    main()
