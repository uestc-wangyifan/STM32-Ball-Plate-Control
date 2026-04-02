"""
vision.py — 板球平衡系统 PC 端视觉程序 + 赛题任务状态机
功能：
  1. 摄像头 640x480 + HSV 调色盘 + 形态学滤波 + 球心检测
  2. 四点标定透视变换（像素 → 物理坐标 0~300mm）
  3. 串口发送 X 帧（50Hz）+ T 帧（目标变化时）+ LOST 帧
  4. 赛题任务状态机：按数字键 4-7 切换任务
  5. 鼠标左键点击设置目标点（Task 5 裁判指定）

用法：
  python vision.py --port COM3
  python vision.py                  (不带串口则仅显示)
"""

import cv2  # type: ignore
import numpy as np  # type: ignore
import time
import json
import os
import argparse
import serial  # type: ignore
from task_scheduler import TaskStateMachine  # type: ignore
from telemetry import Telemetry  # type: ignore

# ==================== 常量 ====================

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

DEFAULT_HSV = {
    "H_min": 5, "H_max": 25,
    "S_min": 100, "S_max": 255,
    "V_min": 100, "V_max": 255,
}

BOARD_SIZE_MM = 300
MORPH_KERNEL_SIZE = 5
MIN_CONTOUR_AREA = 100
LOST_THRESHOLD = 5
SERIAL_INTERVAL = 0.020
SERIAL_BAUDRATE = 115200
RECONNECT_INTERVAL = 1.0
CORNER_NAMES = ["左上", "右上", "右下", "左下"]


# ==================== 配置读写 ====================

def load_config():
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
    data: dict = {"hsv": hsv}
    if corners is not None:
        data["corners"] = corners
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[INFO] 配置已保存到 {CONFIG_PATH}")
    except IOError as e:
        print(f"[ERROR] 保存失败: {e}")


# ==================== 串口 ====================

def open_serial(port):
    if port is None:
        print("[INFO] 未指定串口，仅本地显示模式")
        return None
    try:
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=0.1, write_timeout=0.02)
        print(f"[INFO] 串口 {port} 已打开 ({SERIAL_BAUDRATE}bps)")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开串口 {port}: {e}")
        return None


def send_serial(ser, message):
    if ser is None:
        return False
    try:
        ser.write(message.encode("ascii"))
        return True
    except (serial.SerialException, OSError) as e:
        print(f"[WARN] 串口异常: {e}")
        return False


# ==================== UI ====================

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


# ==================== 透视变换 ====================

def compute_perspective_matrix(corners_px):
    src = np.float32(corners_px)
    dst = np.float32([
        [0, 0], [BOARD_SIZE_MM, 0],
        [BOARD_SIZE_MM, BOARD_SIZE_MM], [0, BOARD_SIZE_MM],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def compute_inverse_perspective(corners_px):
    """反向透视矩阵：物理坐标 → 像素坐标（用于在画面上标注目标点）"""
    src = np.float32(corners_px)
    dst = np.float32([
        [0, 0], [BOARD_SIZE_MM, 0],
        [BOARD_SIZE_MM, BOARD_SIZE_MM], [0, BOARD_SIZE_MM],
    ])
    return cv2.getPerspectiveTransform(dst, src)


def pixel_to_physical(px, py, M):
    point = np.float32([[[px, py]]])
    result = cv2.perspectiveTransform(point, M)
    return float(result[0][0][0]), float(result[0][0][1])


def physical_to_pixel(phys_x, phys_y, M_inv):
    """物理坐标转像素坐标"""
    point = np.float32([[[phys_x, phys_y]]])
    result = cv2.perspectiveTransform(point, M_inv)
    return int(result[0][0][0]), int(result[0][0][1])


# ==================== 标定 ====================

def calibration_phase(cap):
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
    print("[标定模式] 请按顺序点击板面四角：")
    print("  1.左上  2.右上  3.右下  4.左下")
    print("  按 'c' 重来 | 按 'q' 退出")
    print("=" * 50)

    while True:
        display = calib_frame.copy()
        if len(corners) < 4:
            hint = f"Click: {CORNER_NAMES[len(corners)]} ({len(corners)+1}/4)"
        else:
            hint = "Done! Press any key..."
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
        elif len(corners) == 4 and key != 255:
            cv2.destroyWindow("Calibration")
            return corners

    return None


# ==================== 绘制增强 GUI ====================

def draw_target_crosshair(frame, target_x, target_y, M_inv):
    """在画面上用黄色十字准星标注目标点"""
    try:
        tx, ty = physical_to_pixel(target_x, target_y, M_inv)
        size = 15
        cv2.line(frame, (tx - size, ty), (tx + size, ty), (0, 255, 255), 2)
        cv2.line(frame, (tx, ty - size), (tx, ty + size), (0, 255, 255), 2)
        cv2.circle(frame, (tx, ty), 4, (0, 255, 255), -1)
        cv2.putText(frame, f"T({target_x:.0f},{target_y:.0f})",
                    (tx + 10, ty + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    except Exception:
        pass  # 目标点在画面外时忽略


def draw_task_hud(frame, task_info, fps):
    """绘制任务 HUD（抬头显示）"""
    h, w = frame.shape[:2]

    # 左上：FPS
    cv2.putText(frame, f"FPS: {fps:.1f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0, 0, 255), 2, cv2.LINE_AA)

    # 左上第二行：任务名 + 状态
    task_name = task_info["task_name"]
    state = task_info["state"]
    state_colors = {"IDLE": (128,128,128), "READY": (0,255,255),
                    "RUNNING": (0,255,0), "DONE": (255,200,0)}
    color = state_colors.get(state, (255,255,255))
    cv2.putText(frame, f"Task: {task_name} [{state}]",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                color, 2, cv2.LINE_AA)

    # 右上：计时器
    if task_info["timer_text"]:
        timer = task_info["timer_text"]
        cv2.putText(frame, f"Timer: {timer}",
                    (w - 180, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)

    # 右上第二行：距离
    dist = task_info["distance"]
    dist_color = (0, 255, 0) if dist < 10 else (0, 165, 255) if dist < 30 else (0, 0, 255)
    cv2.putText(frame, f"Dist: {dist:.1f}mm",
                (w - 180, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                dist_color, 2, cv2.LINE_AA)

    # 底部：按键提示
    help_text = "[4-7]Task [Space]Start [R]Reset [S]Save [Q]Quit"
    cv2.putText(frame, help_text, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(description="板球平衡系统 - PC视觉端")
    parser.add_argument("--port", type=str, default=None,
                        help="串口号，例如 COM3")
    args = parser.parse_args()

    # ---- 加载配置 ----
    hsv, saved_corners = load_config()

    # ---- 打开串口 ----
    ser = open_serial(args.port)

    # ---- 打开摄像头 ----
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] 无法打开摄像头")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] 摄像头分辨率: {actual_w}x{actual_h}")

    # ---- 标定 ----
    if saved_corners is not None:
        print("[INFO] 已加载标定点（按 'r' 重新标定）")
        corners = saved_corners
    else:
        corners = calibration_phase(cap)
        if corners is None:
            cap.release()
            cv2.destroyAllWindows()
            if ser is not None:
                ser.close()
            return

    M = compute_perspective_matrix(corners)
    M_inv = compute_inverse_perspective(corners)
    print("[INFO] 透视变换矩阵已就绪")

    # ---- 状态机 ----
    sm = TaskStateMachine()

    # ---- 遥测波形 ----
    telem = Telemetry(max_points=500, update_interval=0.08)

    # ---- 鼠标回调：左键点击设置目标 / Task5 裁判指定 ----
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            phys_x, phys_y = pixel_to_physical(x, y, M)
            # 限制在板面范围内
            phys_x = max(0.0, min(float(BOARD_SIZE_MM), phys_x))
            phys_y = max(0.0, min(float(BOARD_SIZE_MM), phys_y))

            if sm.current_task == 5:
                result = sm.set_referee_target(phys_x, phys_y)
                if result == "updated":
                    print(f"[Task5] 裁判指定目标: ({phys_x:.0f}, {phys_y:.0f}) mm")
                elif result == "not_ready":
                    print("[Task5] 需先在中心稳定，再接受裁判指定目标")
            elif sm.current_task == 4:
                # Task4 也允许点击设目标
                sm.target_x = phys_x
                sm.target_y = phys_y
                sm.t_frame_pending = True
                print(f"[Task4] 目标设为: ({phys_x:.0f}, {phys_y:.0f}) mm")
            elif sm.current_task == 6:
                result = sm.set_task6_point(phys_x, phys_y)
                if result == "first":
                    print(f"[Task6] 已记录起点 A: ({phys_x:.0f}, {phys_y:.0f}) mm，请点击终点 B")
                elif result == "too_short":
                    print("[Task6] 轨迹长度必须严格大于 100mm，请重新点击终点 B")
                elif result == "updated":
                    print(f"[Task6] 往复轨迹已更新，A=({sm._reciprocate_a[0]:.0f}, {sm._reciprocate_a[1]:.0f}) mm, B=({sm._reciprocate_b[0]:.0f}, {sm._reciprocate_b[1]:.0f}) mm")

    # ---- Trackbar + 窗口 ----
    create_trackbars(hsv)
    cv2.namedWindow("Original", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("Original", mouse_callback)

    # ---- 形态学核 ----
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
    )

    prev_time: float = time.time()
    frame_count: int = 0
    lost_count: int = 0
    camera_fail_count: int = 0
    last_serial_time: float = 0.0
    last_reconnect_time: float = 0.0
    serial_ok: bool = ser is not None

    port_name = args.port if args.port else "无"
    print(f"[INFO] 串口: {port_name}")
    print("[INFO] 按 4-7 切换任务 | Space 开始 | R 重置 | S 保存 | Q 退出")
    print("[INFO] Task 6: 鼠标点击两点设置往复轨迹，长度必须严格大于 100mm")

    while True:
        ret, frame = cap.read()
        if not ret:
            camera_fail_count += 1
            lost_count += 1
            now = time.time()

            if camera_fail_count == 1:
                print("[WARN] 读取帧失败，按丢球处理")

            if args.port and (not serial_ok) and ((now - last_reconnect_time) >= RECONNECT_INTERVAL):
                last_reconnect_time = now
                ser = open_serial(args.port)
                serial_ok = ser is not None

            if serial_ok and lost_count >= LOST_THRESHOLD and (now - last_serial_time) >= SERIAL_INTERVAL:
                if not send_serial(ser, "LOST\n"):
                    serial_ok = False
                    if ser is not None:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = None
                last_serial_time = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            time.sleep(0.01)
            continue

        camera_fail_count = 0

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
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ---- 轮廓检测 ----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_found: bool = False
        real_x: float = 0.0
        real_y: float = 0.0
        ball_phys_x: float | None = None
        ball_phys_y: float | None = None

        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            if area >= MIN_CONTOUR_AREA:
                ball_found = True
                lost_count = 0

                (cx, cy), radius = cv2.minEnclosingCircle(max_contour)
                cx, cy, radius = int(cx), int(cy), int(radius)

                cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, f"Px({cx},{cy})",
                            (cx + 10, cy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                real_x, real_y = pixel_to_physical(cx, cy, M)
                ball_phys_x, ball_phys_y = real_x, real_y
                cv2.putText(frame, f"mm({real_x:.0f},{real_y:.0f})",
                            (cx + 10, cy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2, cv2.LINE_AA)

        if not ball_found:
            lost_count += 1
            if (frame_count // 15) % 2 == 0:
                text = "LOST"
                font = cv2.FONT_HERSHEY_SIMPLEX
                text_size = cv2.getTextSize(text, font, 2.0, 4)[0]
                text_x = (frame.shape[1] - text_size[0]) // 2
                text_y = (frame.shape[0] + text_size[1]) // 2
                cv2.putText(frame, text, (text_x, text_y),
                            font, 2.0, (0, 0, 255), 4, cv2.LINE_AA)

        # ---- 状态机更新 ----
        task_info = sm.update(ball_phys_x, ball_phys_y, now)

        # ---- 串口断开后自动重连 ----
        if args.port and (not serial_ok) and ((now - last_reconnect_time) >= RECONNECT_INTERVAL):
            last_reconnect_time = now
            ser = open_serial(args.port)
            serial_ok = ser is not None

        # ---- 串口发送（50Hz 限速） ----
        if serial_ok and (now - last_serial_time) >= SERIAL_INTERVAL:
            # T 帧（目标变化时，优先发送）
            if task_info["send_t_frame"]:
                t_msg = f"T:{task_info['target_x']:.1f},Y:{task_info['target_y']:.1f}\n"
                if not send_serial(ser, t_msg):
                    serial_ok = False
                    if ser is not None:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = None
                    print("[WARN] 串口已断开")
                last_serial_time = now
            # X 帧 / LOST 帧
            elif ball_found:
                msg = f"X:{real_x:.1f},Y:{real_y:.1f}\n"
                if not send_serial(ser, msg):
                    serial_ok = False
                    if ser is not None:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = None
                last_serial_time = now
            elif lost_count >= LOST_THRESHOLD:
                if not send_serial(ser, "LOST\n"):
                    serial_ok = False
                    if ser is not None:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = None
                last_serial_time = now

        # ---- 遥测数据推送 ----
        telem.push(ball_phys_x, ball_phys_y,
                   task_info["target_x"], task_info["target_y"], now)
        telem.update_plot()

        # ---- 绘制标定区域 ----
        pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 180, 0), thickness=1)

        # ---- 绘制目标点十字准星 ----
        if sm.current_task != 0:
            draw_target_crosshair(frame, task_info["target_x"],
                                  task_info["target_y"], M_inv)

        # ---- 绘制 HUD ----
        draw_task_hud(frame, task_info, fps)

        # ---- 串口状态（左下角上方） ----
        if args.port:
            if serial_ok:
                status_color = (0, 255, 0)
                status_text = f"UART: {args.port} CONNECTED"
            elif ser is None:
                status_color = (0, 165, 255)
                status_text = "UART: RECONNECTING"
            else:
                status_color = (0, 0, 255)
                status_text = "UART: DISCONNECTED"
            cv2.putText(frame, status_text, (10, frame.shape[0] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1, cv2.LINE_AA)

        # ---- 遥测状态显示 ----
        telem_color = (0, 255, 0) if telem.is_visible else (128, 128, 128)
        telem_text = "[W] Waveform: ON" if telem.is_visible else "[W] Waveform: OFF"
        cv2.putText(frame, telem_text, (frame.shape[1] - 200, frame.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, telem_color, 1, cv2.LINE_AA)

        # ---- 显示 ----
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)

        # ---- 按键处理 ----
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
                M_inv = compute_inverse_perspective(corners)
                print("[INFO] 已重新标定")
            create_trackbars(hsv)
            cv2.namedWindow("Original", cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback("Original", mouse_callback)
        elif key == ord(" "):
            sm.start()
            print(f"[SM] Task {sm.current_task} 开始!")
        elif key in (ord("4"), ord("5"), ord("6"), ord("7")):
            task_id = key - ord("0")
            sm.switch_task(task_id)
            print(f"[SM] 切换到 Task {task_id}: {sm.get_task_name()}")
        elif key == ord("0"):
            sm.switch_task(0)
            print("[SM] 已回到空闲模式")
        elif key == ord("w"):
            telem.toggle()

    # ---- 释放资源 ----
    cap.release()
    if ser is not None:
        ser.close()
        print(f"[INFO] 串口 {args.port} 已关闭")
    cv2.destroyAllWindows()
    print("[INFO] 已安全退出")


if __name__ == "__main__":
    main()
