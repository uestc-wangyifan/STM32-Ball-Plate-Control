又确认了两处必须落到补丁里的硬问题：链接脚本实际栈是 0x600，不是 0x400；USART DMA 收满 50 字节时主循环会把残留旧字节一起解析。另一个并发点是多处 ISR/主循环共享状态没有全部加 volatile。

又新增几处会直接影响稳态和首拍表现的问题：首帧坐标低通从 0 起算、死区逻辑未真正生效、积分限幅量纲不匹配，且 PID 热调参会把 Y 轴微分状态污染。现在我正把这些与空白/半合法/超长串口帧、以及 Python 端“限速≠固定 50Hz”的发送时序一起并入最小补丁并做一致性校验。

Thought for 36m 33s

按你给的五层检查，必须动手的 7 项如下。后面我把“已核对但当前不需要改”的项也单列出来，不留空白。

[S1] 严重级别：🔴必须修
问题：上电阶段的 HAL 启动返回值没查，而且真正有效的舵机归中等待发生在 PWM 启动前。
现象：上机会出现“串口没收包/控制定时器没起来/舵机还没回中就进入控制”的假故障，现场看起来像舵机乱动或系统偶发失灵。
位置：main.c:163-176
原因：HAL_UART_Receive_DMA()、HAL_TIM_PWM_Start()、HAL_TIM_Base_Start_IT() 当前都静默忽略失败；而 HAL_Delay(500) 放在 PWM 启动前，对舵机机械归中没有实际意义。
修复：

/* USER CODE BEGIN 2 */
pid_x.Kp = 1.2f; pid_x.Ki = 0.05f; pid_x.Kd = 0.3f; pid_x.integral = 0.0f; pid_x.last_measurement = 0.0f; pid_x.last_derivative = 0.0f;
pid_y = pid_x;

OLED_Init();
OLED_Clear();
OLED_ShowStr(0, 0, (unsigned char*)"Ball Plate V1.0", 1);
OLED_ShowStr(0, 2, (unsigned char*)"System Booting...", 1);
HAL_Delay(500);
OLED_Clear();

if (HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer)) != HAL_OK) {
    Error_Handler();
}
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
last_rx_tick = HAL_GetTick();

__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)SERVO_CENTER_US);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)SERVO_CENTER_US);

if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
}
if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
}

HAL_Delay(500);

if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
}
/* USER CODE END 2 */

[C1] 严重级别：🔴必须修
问题：首帧/恢复帧从 0 开始做坐标低通，而且 LOST/超时只归中舵机，不清滤波器和 PID 内部状态。
现象：一见到第一帧或刚从 LOST 恢复时，板面会突然打一把，球在中心附近也会被“甩”出去。
位置：main.c:56；main.c:233-249；main.c:292-297；main.c:403-416
原因：POS_FILTER_ALPHA=0.2 的等效时间常数约 90ms、90% 响应约 206ms，50Hz 板球环路里延迟过大；同时 last_pos_x/y 初值为 0，LOST 后又沿用旧积分/旧微分/旧测量历史。
修复：

#define POS_FILTER_ALPHA          0.5f

volatile uint8_t pos_filter_seeded = 0;

static void PID_Reset(volatile PID_TypeDef *pid, float measurement)
{
  pid->integral = 0.0f;
  pid->last_measurement = measurement;
  pid->last_derivative = 0.0f;
}

static void Control_EnterLostState(void)
{
  data_ready_flag = 0;
  servo_center_flag = 1;
  lost_frame_count = LOST_FRAME_THRESHOLD;
  control_state = STATE_LOST;
  pos_filter_seeded = 0;
  PID_Reset(&pid_x, current_x);
  PID_Reset(&pid_y, current_y);
}
if (valid_pos_frame) {
  raw_x = fminf(POS_MAX_MM, fmaxf(POS_MIN_MM, raw_x));
  raw_y = fminf(POS_MAX_MM, fmaxf(POS_MIN_MM, raw_y));

  if (!pos_filter_seeded || control_state != STATE_RUN) {
    last_pos_x = raw_x;
    last_pos_y = raw_y;
    pos_filter_seeded = 1U;
  } else {
    last_pos_x = POS_FILTER_ALPHA * raw_x + (1.0f - POS_FILTER_ALPHA) * last_pos_x;
    last_pos_y = POS_FILTER_ALPHA * raw_y + (1.0f - POS_FILTER_ALPHA) * last_pos_y;
  }

  current_x = last_pos_x;
  current_y = last_pos_y;

  if (control_state != STATE_RUN) {
    PID_Reset(&pid_x, current_x);
    PID_Reset(&pid_y, current_y);
  }

  last_rx_tick = HAL_GetTick();
  data_ready_flag = 1U;
  lost_frame_count = 0;
  invalid_frame_count = 0;
  control_state = STATE_RUN;
}
else if (strncmp(p, "LOST", 4) == 0 &&
         (p[4] == '\0' || p[4] == '\r' || p[4] == '\n' ||
          p[4] == ' ' || p[4] == '\t')) {
  Control_EnterLostState();
}
if ((HAL_GetTick() - last_rx_tick) > RX_TIMEOUT_MS) {
  Control_EnterLostState();
}

if (servo_center_flag) {
  servo_center_flag = 0;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)SERVO_CENTER_US);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)SERVO_CENTER_US);
  return;
}

if (data_ready_flag == 0 || control_state != STATE_RUN) {
  return;
}

[C2] 严重级别：🔴必须修
问题：PID 里没有 2mm 死区，而且积分限幅写成了“误差积分量”而不是“I 项输出量”。
现象：目标点附近舵机会持续细抖；板面有一点点静态偏差时，小球会慢漂，怎么加 Ki 都感觉“没吃进去”。
位置：main.c:42-45；main.c:376-395
原因：当前代码里根本没有 fabsf(err)<2.0f；INTEGRAL_MAX=200 在 Ki=0.05 时最多只产生 10us 的 I 输出，和 ±300us 控制限幅完全不在一个量级。
修复：

#define CONTROL_DT                0.02f
#define DERIV_TAU                 0.03f
#define CONTROL_OUTPUT_LIMIT      300.0f
#define INTEGRAL_OUTPUT_LIMIT      80.0f
#define ERROR_DEADBAND_MM           2.0f

float PID_Calculate(volatile PID_TypeDef *pid, float target, float current, float dt) 
{
  if (dt <= 0.0f) {
    dt = CONTROL_DT;
  }

  float error = target - current;
  if (fabsf(error) < ERROR_DEADBAND_MM) {
    error = 0.0f;
  }

  pid->integral += error * dt;

  if (pid->Ki > 0.0f) {
    float integral_limit = INTEGRAL_OUTPUT_LIMIT / pid->Ki;
    pid->integral = LIMIT(pid->integral, -integral_limit, integral_limit);
  } else {
    pid->integral = 0.0f;
  }

  float raw_derivative = -(current - pid->last_measurement) / dt;
  float alpha = dt / (DERIV_TAU + dt);
  pid->last_derivative = alpha * raw_derivative + (1.0f - alpha) * pid->last_derivative;
  pid->last_measurement = current;

  float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * pid->last_derivative);
  output = LIMIT(output, -CONTROL_OUTPUT_LIMIT, CONTROL_OUTPUT_LIMIT);

  return output;
}

[C3] 严重级别：🔴必须修
问题：PID 热调参时把 X 轴历史测量值复制给了 Y 轴，而且第三个参数后面的脏字符也会被接受。
现象：发一帧 PID 参数后，Y 轴会偶发打一把；PID:1.5,0.05,0.3abc 这种脏帧也可能改参。
位置：main.c:267-287
原因：pid_y = pid_x 会把 X 轴 last_measurement 错拷到 Y 轴；第三个 strtof() 只检查“读到了数字”，没有检查帧尾是否干净、数值是否 finite。
修复：

else if (strncmp(p, "PID:", 4) == 0) {
  char *cursor = p + 4;
  char *endptr;
  float p_val = strtof(cursor, &endptr);

  if (endptr != cursor && *endptr == ',') {
    cursor = endptr + 1;
    float i_val = strtof(cursor, &endptr);

    if (endptr != cursor && *endptr == ',') {
      cursor = endptr + 1;
      float d_val = strtof(cursor, &endptr);

      while (*endptr == ' ' || *endptr == '\r' || *endptr == '\n' || *endptr == '\t') {
        endptr++;
      }

      if (endptr != cursor &&
          *endptr == '\0' &&
          isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
          p_val > 0.0f && p_val < 10.0f &&
          i_val >= 0.0f && i_val < 5.0f &&
          d_val >= 0.0f && d_val < 5.0f) {

        __disable_irq();
        pid_x.Kp = p_val;
        pid_x.Ki = i_val;
        pid_x.Kd = d_val;
        pid_y.Kp = p_val;
        pid_y.Ki = i_val;
        pid_y.Kd = d_val;
        PID_Reset(&pid_x, current_x);
        PID_Reset(&pid_y, current_y);
        __enable_irq();
      }
    }
  }
}

[T1] 严重级别：🟡建议修
问题：短帧/空白帧/超长帧处理过松，配合 undocumented 的 bare x,y 兼容解析，会把垃圾当坐标或者累计误判 LOST。
现象：低概率突然掉进 LOST、偶发坐标跳变、串口看起来像“偶发毛刺”但实物会抖一下。
位置：stm32f1xx_it.c:244-259；main.c:204-230；main.c:298-305
原因：IDLE ISR 没先验证是否收到完整换行帧，主循环又会把只有空白的短帧计入 invalid_frame_count；而裸 x,y 解析会把被截断的尾巴误当成合法坐标。
修复：

/* stm32f1xx_it.c */
extern volatile uint8_t data_ready_flag;
extern volatile uint8_t servo_center_flag;

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);

  if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    if (HAL_UART_DMAStop(&huart1) != HAL_OK)
    {
      data_ready_flag = 0U;
      servo_center_flag = 1U;
      return;
    }

    rx_len = sizeof(rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (rx_len >= sizeof(parse_buffer))
    {
      rx_len = sizeof(parse_buffer) - 1U;
    }

    rx_flag = 0U;
    if ((rx_len > 0U) &&
        (rx_buffer[rx_len - 1U] == '\n' || rx_buffer[rx_len - 1U] == '\r'))
    {
      memcpy((void*)parse_buffer, (void*)rx_buffer, rx_len);
      parse_buffer[rx_len] = '\0';
      rx_flag = 1U;
    }
    else
    {
      parse_buffer[0] = '\0';
      rx_len = 0U;
    }

    if (HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer)) != HAL_OK)
    {
      data_ready_flag = 0U;
      servo_center_flag = 1U;
    }
  }
}
/* main.c */
static uint8_t IsBlankLine(const char *p)
{
  while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t') {
    p++;
  }
  return (*p == '\0');
}

if (p[0] == 'X' && p[1] == ':') {
  char *endptr;
  raw_x = strtof(p + 2, &endptr);
  char *comma = strchr(endptr, ',');
  if (comma && *(comma + 1) == 'Y' && *(comma + 2) == ':') {
    char *y_start = comma + 3;
    char *y_end;
    raw_y = strtof(y_start, &y_end);
    while (*y_end == ' ' || *y_end == '\r' || *y_end == '\n' || *y_end == '\t') {
      y_end++;
    }
    if (y_end != y_start && *y_end == '\0') {
      valid_pos_frame = 1U;
    }
  }
}

else if (!IsBlankLine(p)) {
  invalid_frame_count++;
  data_ready_flag = 0;
  if (invalid_frame_count >= LOST_FRAME_THRESHOLD) {
    Control_EnterLostState();
  }
}

[P1] 严重级别：🔴必须修
问题：vision.py 的串口发送节拍绑死在相机/UI 主循环上，当前只能“尽量 50Hz”，不能稳定 50Hz。
现象：开波形窗、摄像头帧率波动、cap.read() 卡顿时，MCU 收到的 X 帧会直接掉到 40Hz、30Hz 甚至更低，球会明显变软、滞后。
位置：vision.py:383-387；vision.py:394-556；telemetry.py:71-107
原因：发送节拍现在靠 time.time() + 主循环判间隔，cap.read()、imshow/waitKey、flush_events() 都会阻塞这一个线程；下面这段是最小补丁，只能把抖动和漂移压下去，真要硬 50Hz 还得拆发送线程。
修复：

# ---- 加载配置 ----
hsv, saved_corners, saved_resolution = load_config()

# ---- 打开串口 ----
ser = open_serial(args.port)

# ---- 打开摄像头 ----
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[ERROR] 无法打开摄像头")
    return

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_resolution = (actual_w, actual_h)
print(f"[INFO] 摄像头分辨率: {actual_w}x{actual_h}")
prev_time: float = time.time()
frame_count: int = 0
lost_count: int = 0
camera_fail_count: int = 0
last_t_frame_time: float = 0.0
last_reconnect_time: float = 0.0
next_serial_time: float = time.perf_counter()
serial_ok: bool = ser is not None
while True:
    ret, frame = cap.read()
    wall_now = time.time()
    mono_now = time.perf_counter()

    if not ret:
        camera_fail_count += 1
        lost_count += 1

        if camera_fail_count == 1:
            print("[WARN] 读取帧失败，按丢球处理")

        if args.port and (not serial_ok) and ((mono_now - last_reconnect_time) >= RECONNECT_INTERVAL):
            last_reconnect_time = mono_now
            ser = open_serial(args.port)
            serial_ok = ser is not None

        if serial_ok and lost_count >= LOST_THRESHOLD and mono_now >= next_serial_time:
            while next_serial_time <= mono_now:
                next_serial_time += SERIAL_INTERVAL

            if not send_serial(ser, "LOST\n"):
                serial_ok = False
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        time.sleep(0.005)
        continue

    camera_fail_count = 0
    frame_count += 1

    fps = 1.0 / (wall_now - prev_time) if (wall_now - prev_time) > 0 else 0.0
    prev_time = wall_now

    hsv = read_trackbars()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = (hsv["H_min"], hsv["S_min"], hsv["V_min"])
    upper = (hsv["H_max"], hsv["S_max"], hsv["V_max"])
    mask = cv2.inRange(hsv_frame, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_found: bool = False
    real_x: float = 0.0
    real_y: float = 0.0
    ball_phys_x: float | None = None
    ball_phys_y: float | None = None

    if contours:
        best_contour = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_CONTOUR_AREA:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.6:
                continue

            if area > best_area:
                best_area = area
                best_contour = contour

        if best_contour is not None:
            ball_found = True
            lost_count = 0

            (cx, cy), radius = cv2.minEnclosingCircle(best_contour)
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

    task_info = sm.update(ball_phys_x, ball_phys_y, wall_now)

    if args.port and (not serial_ok) and ((mono_now - last_reconnect_time) >= RECONNECT_INTERVAL):
        last_reconnect_time = mono_now
        ser = open_serial(args.port)
        serial_ok = ser is not None

    if serial_ok and mono_now >= next_serial_time:
        while next_serial_time <= mono_now:
            next_serial_time += SERIAL_INTERVAL

        need_t_frame = task_info["send_t_frame"] or (
            sm.current_task != 0 and (mono_now - last_t_frame_time) >= T_FRAME_KEEPALIVE
        )

        if need_t_frame:
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
            else:
                last_t_frame_time = mono_now
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
        elif lost_count >= LOST_THRESHOLD:
            if not send_serial(ser, "LOST\n"):
                serial_ok = False
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None

[B1] 严重级别：🔴必须修
问题：保存的标定点没有绑定相机分辨率，换分辨率后会直接复用旧 corners。
现象：画面看起来还能识别，但物理坐标整体偏斜/缩放错误，PID 再怎么调都不对。
位置：vision.py:48-80；vision.py:301-333；vision.py:603-615
原因：config.json 只存 HSV 和四角，不存标定分辨率；程序启动时也不校验 saved corners 是否仍然匹配当前输出尺寸。
修复：

def load_config():
    hsv = DEFAULT_HSV.copy()
    corners = None
    calib_resolution = None
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
            if "calibration_resolution" in data and len(data["calibration_resolution"]) == 2:
                calib_resolution = (
                    int(data["calibration_resolution"][0]),
                    int(data["calibration_resolution"][1]),
                )
            if corners is not None:
                print("[INFO] 已加载标定点")
        except (json.JSONDecodeError, IOError, ValueError, TypeError) as e:
            print(f"[WARN] 读取 config.json 失败: {e}，使用默认值")
    return hsv, corners, calib_resolution


def save_config(hsv, corners, calibration_resolution):
    data: dict = {
        "hsv": hsv,
        "calibration_resolution": [int(calibration_resolution[0]), int(calibration_resolution[1])],
    }
    if corners is not None:
        data["corners"] = corners
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[INFO] 配置已保存到 {CONFIG_PATH}")
    except IOError as e:
        print(f"[ERROR] 保存失败: {e}")
hsv, saved_corners, saved_resolution = load_config()

if saved_corners is not None and saved_resolution == actual_resolution:
    print("[INFO] 已加载标定点（按 'r' 重新标定）")
    corners = saved_corners
else:
    if saved_corners is not None and saved_resolution != actual_resolution:
        print(f"[WARN] 标定分辨率 {saved_resolution} 与当前 {actual_resolution} 不一致，已强制重新标定")
    corners = calibration_phase(cap)
    if corners is None:
        cap.release()
        cv2.destroyAllWindows()
        if ser is not None:
            ser.close()
        return
elif key == ord("s"):
    save_config(hsv, corners, actual_resolution)

已核对但当前不需要改的项：

USART1_IRQHandler 里没看到 HAL_Delay/printf/while(1) 这类显式阻塞调用；但它现在仍然偏重，所以我把 ISR 收紧放进了 [T1]。位置：stm32f1xx_it.c:237-260。
共享变量这块，rx_flag/rx_len/parse_buffer/data_ready_flag/servo_center_flag/last_rx_tick/target_x/target_y/current_x/current_y/control_state/pid_x/pid_y 都已经是 volatile；rx_buffer 不被主循环直接访问，不需要硬加 volatile。位置：main.c:95-113；stm32f1xx_it.c:62-65。
rx_flag 的清零时机是对的：先关中断，再清 rx_flag，再 memcpy。位置：main.c:190-193。
parse_buffer[rx_len] = '\0' 现代码确实能保证结尾；LOST 判断也确实用的是去前导空格后的 p。位置：stm32f1xx_it.c:252-255；main.c:195-202, 292-297。
servo_center_flag 分支在 TIM2 回调里位于所有控制 return 之前，所以 LOST 后下一次 20ms tick 会真写 1500us 中位。位置：main.c:409-416。
Python 侧 cap.read() 失败会按 LOST 路径处理，不会直接退出主循环；透视坐标已夹紧 [0,300]；circularity < 0.6 已实现；Task5 确实先中心稳定再接受裁判点击；Task6 用 dist <= 100 拒绝，满足“严格大于 100mm”。位置：vision.py:160-165, 395-424, 468-470；task_scheduler.py:90-100, 112-123, 192-200。
T 帧不是一次性发送，T_FRAME_KEEPALIVE = 0.2 已做周期重发，不会永久失步。位置：vision.py:40, 517-534。
disp_buf[20] 按当前三个格式串不会溢出；最紧的是 "L-Cnt:%d " 在 32 位有符号正整数上正好 19 字符加结尾 \0。位置：main.c:312-325。
链接脚本实际是 Heap=0x400, Stack=0x600，不是你题干里写的 0x400/0x400；按当前 RAM 占用和局部变量规模，我没看到立刻溢出的代码证据。位置：STM32F103XX_FLASH.ld:58-59。

上机前必须修复：S1、C1、C2、C3、P1、B1
建议修复：T1
可以忽略：无

PID调参建议：
先把 C1/C2/C3 这些补丁落掉，再开始调；不然你现在看到的很多抖动和慢漂，其实是代码问题，不是 PID 本体。
针对 MG996R + 30cm 板面，我给你的经验起步值是：Kp=2.2, Ki=0.03, Kd=0.45，同时固定 POS_FILTER_ALPHA=0.5、ERROR_DEADBAND_MM=2.0、DERIV_TAU=0.03、INTEGRAL_OUTPUT_LIMIT=80.0。
上板顺序：先只开 Kp 找到“能拉回但不连续振荡”的点；再加 Kd 抑制过冲；最后只用 0.01 的步进慢加 Ki 去消静差。
如果中心附近是高频细抖，先把 Kd 从 0.45 降到 0.35；如果整体反应慢，再把 Kp 从 2.2 往 2.6 试；如果能稳住但会慢慢爬偏，再把 Ki 从 0.03 加到 0.04/0.05。