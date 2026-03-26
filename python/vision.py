"""
vision.py — 板球平衡系统 PC 端视觉程序
功能：
  1. 启动后标定（点击板面四角）或加载已保存标定
  2. HSV 二值化 + 形态学滤波 + 最大轮廓检测
  3. 透视变换：像素坐标 → 物理坐标 (0~300mm)
  4. 串口发送（50Hz 限速）：
     - 找到球 → X:{x:.1f},Y:{y:.1f}\n
     - 连续丢失 ≥5帧 → LOST\n
  5. 按 's' 保存配置 | 按 'r' 重新标定 | 按 'q' 退出

用法：
  python vision.py --port COM3
  python vision.py                  (不带串口参数则仅显示画面)
"""

import cv2  # type: ignore
import numpy as np  # type: ignore
import time
import json
import os
import argparse
import serial  # type: ignore

# ---- 配置文件路径（与本脚本同目录） ----
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

# ---- 默认 HSV 阈值（橙色乒乓球的典型范围） ----
DEFAULT_HSV = {
    "H_min": 5,
    "H_max": 25,
    "S_min": 100,
    "S_max": 255,
    "V_min": 100,
    "V_max": 255,
}

# ---- 物理板面尺寸 (mm) ----
BOARD_SIZE_MM = 300

# ---- 形态学核大小 ----
MORPH_KERNEL_SIZE = 5

# ---- 轮廓最小面积阈值 ----
MIN_CONTOUR_AREA = 100

# ---- 连续丢帧阈值，达到后发 LOST ----
LOST_THRESHOLD = 5

# ---- 串口发送最小间隔 (秒)，50Hz = 20ms ----
SERIAL_INTERVAL = 0.020

# ---- 串口波特率（与 STM32 USART1 一致） ----
SERIAL_BAUDRATE = 115200

# ---- 标定点名称 ----
CORNER_NAMES = ["左上", "右上", "右下", "左下"]


# ======================= 配置读写 =======================

def load_config():
    """加载 config.json，返回 (hsv_dict, corners_list_or_None)"""
    hsv = DEFAULT_HSV.copy()
    corners = None
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            if "hsv" in data:
                valid = True
                for key in DEFAULT_HSV:
                    if key not in data["hsv"]:
                        print(f"[WARN] config.json hsv 缺少 '{key}'，使用默认值")
                        valid = False
                        break
                if valid:
                    hsv = data["hsv"]
                    print("[INFO] 已加载 HSV 阈值")
            if "corners" in data and len(data["corners"]) == 4:
                corners = data["corners"]
                print("[INFO] 已加载标定点")
        except (json.JSONDecodeError, IOError) as e:
            print(f"[WARN] 读取 config.json 失败: {e}，使用默认值")
    return hsv, corners


def save_config(hsv, corners):
    """保存 HSV 阈值和标定点到 config.json"""
    data: dict = {"hsv": hsv}
    if corners is not None:
        data["corners"] = corners
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[INFO] 配置已保存到 {CONFIG_PATH}")
    except IOError as e:
        print(f"[ERROR] 保存失败: {e}")


# ======================= 串口 =======================

def open_serial(port):
    """
    尝试打开串口，失败返回 None（程序不崩溃）。
    """
    if port is None:
        print("[INFO] 未指定串口，仅本地显示模式")
        return None
    try:
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=0.1)
        print(f"[INFO] 串口 {port} 已打开 ({SERIAL_BAUDRATE}bps)")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开串口 {port}: {e}")
        return None


def send_serial(ser, message):
    """
    安全发送串口数据，捕获异常防止程序崩溃。
    返回 True=发送成功，False=发送失败。
    """
    if ser is None:
        return False
    try:
        ser.write(message.encode("ascii"))
        return True
    except serial.SerialException as e:
        print(f"[WARN] 串口发送失败: {e}")
        return False
    except OSError as e:
        print(f"[WARN] 串口设备异常: {e}")
        return False


# ======================= UI =======================

def nothing(x):
    pass


def create_trackbars(hsv):
    cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Trackbars", 400, 300)
    cv2.createTrackbar("H_min", "Trackbars", hsv["H_min"], 179, nothing)
    cv2.createTrackbar("H_max", "Trackbars", hsv["H_max"], 179, nothing)
    cv2.createTrackbar("S_min", "Trackbars", hsv["S_min"], 255, nothing)
    cv2.createTrackbar("S_max", "Trackbars", hsv["S_max"], 255, nothing)
    cv2.createTrackbar("V_min", "Trackbars", hsv["V_min"], 255, nothing)
    cv2.createTrackbar("V_max", "Trackbars", hsv["V_max"], 255, nothing)


def read_trackbars():
    return {
        "H_min": cv2.getTrackbarPos("H_min", "Trackbars"),
        "H_max": cv2.getTrackbarPos("H_max", "Trackbars"),
        "S_min": cv2.getTrackbarPos("S_min", "Trackbars"),
        "S_max": cv2.getTrackbarPos("S_max", "Trackbars"),
        "V_min": cv2.getTrackbarPos("V_min", "Trackbars"),
        "V_max": cv2.getTrackbarPos("V_max", "Trackbars"),
    }


# ======================= 透视变换 =======================

def compute_perspective_matrix(corners_px):
    """四角像素坐标 → 透视变换矩阵"""
    src = np.float32(corners_px)
    dst = np.float32([
        [0, 0],
        [BOARD_SIZE_MM, 0],
        [BOARD_SIZE_MM, BOARD_SIZE_MM],
        [0, BOARD_SIZE_MM],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def pixel_to_physical(px, py, M):
    """像素坐标 → 物理毫米坐标"""
    point = np.float32([[[px, py]]])
    result = cv2.perspectiveTransform(point, M)
    return float(result[0][0][0]), float(result[0][0][1])


# ======================= 标定 =======================

def calibration_phase(cap):
    """冻结画面，用户点击四角，返回坐标列表或 None"""
    for _ in range(10):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] 标定时读取帧失败")
        return None

    calib_frame = frame.copy()
    corners = []

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(corners) < 4:
            corners.append([x, y])
            print(f"  [{CORNER_NAMES[len(corners)-1]}] 像素坐标: ({x}, {y})")

    cv2.namedWindow("Calibration", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("Calibration", mouse_callback)

    print("=" * 50)
    print("[标定模式] 请在画面上按顺序点击板面的四个角：")
    print("  1.左上  2.右上  3.右下  4.左下")
    print("  按 'c' 取消重来 | 按 'q' 退出程序")
    print("=" * 50)

    while True:
        display = calib_frame.copy()

        if len(corners) < 4:
            hint = f"Click: {CORNER_NAMES[len(corners)]} ({len(corners)+1}/4)"
        else:
            hint = "Done! Press any key to continue..."
        cv2.putText(display, hint, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

        for i, pt in enumerate(corners):
            cv2.circle(display, tuple(pt), 6, (0, 0, 255), -1)
            cv2.putText(display, CORNER_NAMES[i], (pt[0]+10, pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        if len(corners) > 1:
            for i in range(len(corners) - 1):
                cv2.line(display, tuple(corners[i]), tuple(corners[i+1]), (0, 255, 0), 2)
        if len(corners) == 4:
            cv2.line(display, tuple(corners[3]), tuple(corners[0]), (0, 255, 0), 2)

        cv2.imshow("Calibration", display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            cv2.destroyWindow("Calibration")
            return None
        elif key == ord("c"):
            corners.clear()
            print("[INFO] 已清除标定点，请重新点击")
        elif len(corners) == 4:
            if key != 255:
                cv2.destroyWindow("Calibration")
                return corners

    return None


# ======================= 主循环 =======================

def main():
    # ---- 命令行参数 ----
    parser = argparse.ArgumentParser(description="板球平衡系统 - PC视觉端")
    parser.add_argument("--port", type=str, default=None,
                        help="串口号，例如 COM3 或 /dev/ttyUSB0")
    args = parser.parse_args()

    # ---- 加载配置 ----
    hsv, saved_corners = load_config()

    # ---- 打开串口 ----
    ser = open_serial(args.port)

    # ---- 打开摄像头 ----
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] 无法打开摄像头，请检查设备连接")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] 摄像头分辨率: {actual_w}x{actual_h}")

    # ---- 标定 ----
    if saved_corners is not None:
        print("[INFO] 检测到已保存的标定点，跳过标定")
        print(f"  左上={saved_corners[0]}  右上={saved_corners[1]}")
        print(f"  右下={saved_corners[2]}  左下={saved_corners[3]}")
        print("  （按 'r' 可随时重新标定）")
        corners = saved_corners
    else:
        corners = calibration_phase(cap)
        if corners is None:
            cap.release()
            cv2.destroyAllWindows()
            if ser:
                ser.close()
            return

    M = compute_perspective_matrix(corners)
    print("[INFO] 透视变换矩阵已计算完成")

    # ---- Trackbar ----
    create_trackbars(hsv)

    # ---- 形态学核 ----
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
    )

    prev_time = time.time()
    frame_count: int = 0
    lost_count: int = 0          # 连续丢帧计数
    last_serial_time: float = 0.0  # 上次串口发送时间戳
    serial_ok = ser is not None  # 串口是否可用

    port_name = args.port if args.port else "无"
    print(f"[INFO] 串口: {port_name} | 按 'q' 退出 | 's' 保存 | 'r' 重新标定")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 读取帧失败")
            break

        frame_count += 1
        now = time.time()

        # ---- FPS ----
        fps = 1.0 / (now - prev_time) if (now - prev_time) > 0 else 0
        prev_time = now

        # ---- Trackbar ----
        hsv = read_trackbars()

        # ---- HSV + Mask ----
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (hsv["H_min"], hsv["S_min"], hsv["V_min"])
        upper = (hsv["H_max"], hsv["S_max"], hsv["V_max"])
        mask = cv2.inRange(hsv_frame, lower, upper)

        # ---- 形态学 ----
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ---- 轮廓检测 ----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_found = False
        real_x, real_y = 0.0, 0.0

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            if area >= MIN_CONTOUR_AREA:
                ball_found = True
                lost_count = 0

                (cx, cy), radius = cv2.minEnclosingCircle(max_contour)
                cx, cy, radius = int(cx), int(cy), int(radius)

                # 绘制标注
                cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, f"Px({cx},{cy})",
                            (cx + 10, cy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                # 透视变换
                real_x, real_y = pixel_to_physical(cx, cy, M)
                cv2.putText(frame, f"mm({real_x:.0f},{real_y:.0f})",
                            (cx + 10, cy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2, cv2.LINE_AA)

        if not ball_found:
            lost_count += 1
            # 闪烁 LOST
            if (frame_count // 15) % 2 == 0:
                text = "LOST"
                font = cv2.FONT_HERSHEY_SIMPLEX
                text_size = cv2.getTextSize(text, font, 2.0, 4)[0]
                text_x = (frame.shape[1] - text_size[0]) // 2
                text_y = (frame.shape[0] + text_size[1]) // 2
                cv2.putText(frame, text, (text_x, text_y),
                            font, 2.0, (0, 0, 255), 4, cv2.LINE_AA)

        # ---- 串口发送（50Hz 限速） ----
        if serial_ok and (now - last_serial_time) >= SERIAL_INTERVAL:
            if ball_found:
                msg = f"X:{real_x:.1f},Y:{real_y:.1f}\n"
                if not send_serial(ser, msg):
                    serial_ok = False
                    print("[WARN] 串口已断开，后续不再发送")
            elif lost_count >= LOST_THRESHOLD:
                msg = "LOST\n"
                if not send_serial(ser, msg):
                    serial_ok = False
                    print("[WARN] 串口已断开，后续不再发送")
            last_serial_time = now

        # ---- 画标定区域 ----
        pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 180, 0), thickness=1)

        # ---- 串口状态指示（左下角） ----
        if args.port:
            status_color = (0, 255, 0) if serial_ok else (0, 0, 255)
            status_text = f"UART: {args.port}" if serial_ok else f"UART: DISCONNECTED"
            cv2.putText(frame, status_text, (10, frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1, cv2.LINE_AA)

        # ---- FPS ----
        cv2.putText(frame, f"FPS: {fps:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255), 2, cv2.LINE_AA)

        # ---- 显示 ----
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        # ---- 按键 ----
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            save_config(hsv, corners)
        elif key == ord("r"):
            cv2.destroyWindow("Original")
            cv2.destroyWindow("Mask")
            cv2.destroyWindow("Trackbars")
            new_corners = calibration_phase(cap)
            if new_corners is not None:
                corners = new_corners
                M = compute_perspective_matrix(corners)
                print("[INFO] 已重新标定，透视矩阵已更新")
            create_trackbars(hsv)

    # ---- 释放资源 ----
    cap.release()
    if ser is not None:
        ser.close()
        print(f"[INFO] 串口 {args.port} 已关闭")
    cv2.destroyAllWindows()
    print("[INFO] 已安全退出")


if __name__ == "__main__":
    main()
