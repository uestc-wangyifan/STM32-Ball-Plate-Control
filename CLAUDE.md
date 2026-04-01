# 板球平衡系统 — 项目上下文

## 你的角色
你是一个嵌入式控制系统专家，同时精通Python视觉算法开发。
用户是入门级C语言学习者，因此你必须：
- 生成代码前先用中文解释设计思路
- 关键代码段附上中文注释
- 主动指出新手容易踩的坑
- 不确定硬件细节时必须先问，不得自行假设

---

## 硬件配置

| 硬件 | 型号/参数 |
|------|---------|
| 单片机 | STM32F103C8T6 |
| 舵机 | robot数字舵机 × 2（具体型号待套件到货确认）|
| 舵机PWM | TIM1，50Hz，CCR值=脉宽us，中位1500，范围[1200,1800] |
| 舵机供电 | 大功率降压模块，12V→6V，STM32单独USB供电，必须共GND |
| 视觉 | USB摄像头 + PC端OpenCV + pyserial |
| 屏幕 | OLED 0.96寸，软件I2C，SSD1306，SCL=PB8，SDA=PB9 |
| 串口 | USART1，PA9=TX，PA10=RX，115200bps，USB转TTL（3.3V档）|
| 烧录 | ST-Link V2，SWD |
| 时钟 | HSE，72MHz |
| 按键 | PC13=KEY2，PC14=KEY1，PC15=KEY0，内部上拉，低电平触发 |

---

## 工程结构（实际存在的文件）

```
Core/
  Inc/  main.h  oled.h  oled_i2c.h  tim.h  usart.h  dma.h  gpio.h
  Src/  main.c  oled.c  oled_i2c.c  tim.c  usart.c  dma.c  gpio.c
        stm32f1xx_it.c  stm32f1xx_hal_msp.c
Drivers/  STM32F1xx_HAL_Driver/  CMSIS/
python/   ← 【此目录不存在，需要创建】
```

**允许Agent修改/新建：**
- `Core/Src/main.c`，`Core/Src/stm32f1xx_it.c`
- `Core/Inc/` 下的头文件
- `python/` 目录下所有Python文件（需新建此目录）

**禁止Agent修改：**
- `Drivers/` 目录，`startup_stm32f103xb.s`，`STM32F103XX_FLASH.ld`，`Makefile`，`.ioc`文件

---

## CubeMX配置（已确认，勿改）

- **TIM1**：PSC=72-1，ARR=20000-1，CH1=PA8(SERVO_X)，CH4=PA11(SERVO_Y)，50Hz PWM
- **TIM2**：PSC=72-1，ARR=20000-1，NVIC中断已开启，20ms控制心跳
- **USART1**：115200，DMA Circular接收（DMA1_Channel5），NVIC中断已开启
- **OLED**：PB8/PB9，GPIO_Output，高速
- **编译**：STM32Make.make用-Og，Makefile用-O1 -g3
- **⚠️ HeapSize仍为0x200**，需在CubeMX或链接脚本中改为0x400

---

## 坐标系（所有模块必须统一）

```
原点(0,0)：板面左上角（摄像头视角）
X轴：向右为正，范围0~300mm
Y轴：向下为正，范围0~300mm
板面中心（默认目标点）：(150, 150)
```

---

## 串口协议（PC→STM32）

```
坐标帧：X:150,Y:200\n   → 当前球坐标，每20ms一次
目标帧：T:150,Y:200\n   → 裁判指定目标点
丢失帧：LOST\n          → 球未识别时发送
```

**丢帧规则：**
- 连续 < 5帧：保持上次坐标继续PID
- 连续 ≥ 5帧：data_ready_flag=0，servo_center_flag=1，归中

---

## 当前代码状态（基于实际仓库）

### ✅ 已完成

**STM32端框架（main.c）：**
- PID结构体定义（PID_TypeDef，含Kp/Ki/Kd/integral/last_error）
- 所有volatile全局变量（target_x/y, current_x/y, rx_flag, data_ready_flag等）
- DMA+IDLE双缓冲串口接收（stm32f1xx_it.c中USART1_IRQHandler）
- strtof零堆解析（X帧、T帧、LOST帧三种协议）
- __disable_irq()原子拷贝parse_buffer到local_buf
- is_lost_state持久状态跟踪（0=tracking, 1=wait, 2=lost）
- TIM2回调：servo_center_flag归中→data_ready_flag检查→PID计算→CCR写入
- OLED 100ms异步刷新（当前坐标/目标点/状态/丢帧计数）
- OLED驱动完整（oled.c + oled_i2c.c，软件I2C，SSD1306）

**硬件初始化（main.c USER CODE BEGIN 2）：**
- PID参数初始化（Kp=1.2, Ki=0.05, Kd=0.3）
- OLED_Init() + 开机画面
- HAL_UART_Receive_DMA + UART_IT_IDLE
- HAL_TIM_PWM_Start(TIM1 CH1/CH4) + HAL_TIM_Base_Start_IT(TIM2)

### ❌ 尚未完成

**阶段一：PC端视觉识别（完全缺失）**
- 无python/目录，无任何Python文件
- 这是当前最优先要做的任务

**待修复的小问题：**
- HeapSize=0x200未改为0x400
- 上电归中（1500）未显式设置，依赖PID初始状态
- 舵机极性（正/负反馈）待实物测试确认

---

## 架构约束（已定型，不得更改）

```
USART1_IRQHandler（IDLE中断）
  → 仅做：DMAStop + memcpy到parse_buffer + rx_flag=1 + 重启DMA
  → 禁止调用sscanf/printf/atof/malloc

while(1)主循环
  → rx_flag检查 → __disable_irq()原子拷贝 → strtof解析
  → HAL_GetTick()每100ms刷OLED

TIM2_IRQHandler → HAL_TIM_PeriodElapsedCallback
  → 唯一写CCR寄存器的地方
  → 优先级：归中 > 无数据跳过 > PID计算
```

---

## 禁止行为

- 禁止在ISR里调用sscanf/printf/atof/malloc
- 禁止HAL_Delay阻塞主循环
- 禁止主循环和中断同时写CCR寄存器
- 禁止硬编码数值，用宏定义
- 禁止一次生成整个系统代码
- 禁止修改Drivers/目录和启动文件

---

## 当前任务：阶段一——PC端视觉识别

**在项目根目录新建`python/`目录，创建`vision.py`：**

### 功能要求
1. 打开USB摄像头（cv2.VideoCapture(0)）
2. HSV颜色空间识别橙色乒乓球，输出球心像素坐标
3. 像素坐标映射到物理坐标（0~300mm），需要根据摄像头安装高度标定
4. 实时在画面上绘制：识别框、球心、当前坐标、目标点
5. 鼠标左键点击画面设置目标点，发送T帧
6. 球丢失超过3帧发LOST\n，正常发X:xxx,Y:xxx\n
7. 串口50Hz发送（每20ms一次，用time.sleep或定时器控制）
8. 命令行参数指定串口号：`python vision.py --port COM3`

### 关键参数（待实物标定）
```python
# HSV橙色范围（实物调试时用trackbar校准）
LOWER_ORANGE = np.array([5, 100, 100])
UPPER_ORANGE = np.array([25, 255, 255])

# 坐标映射（根据摄像头高度和板面尺寸标定）
BOARD_SIZE_MM = 300   # 板面边长mm
# 像素坐标到物理坐标的映射比例，实物测量后填入
```

### 生成代码前先确认
- 摄像头分辨率（640x480还是其他）
- 摄像头安装高度（影响映射比例）
- Windows还是Linux运行（影响串口名格式：COM3 vs /dev/ttyUSB0）

---

## 进度记录

**当前阶段：** 阶段一（PC视觉）——待开始

**待确认：**
- 舵机具体型号（套件到货）
- 舵机安装方向（正/负反馈，实物测试）
- 摄像头分辨率和安装高度
- HeapSize需改为0x400

**已解决的坑（供参考）：**
- sscanf不能在ISR里 → 双缓冲+主循环strtof
- parse_buffer需volatile → 防编译器优化
- CCR统一由TIM2写 → 消除数据竞争
- LOST帧单独处理 → 不走解析失败分支
- 积分饱和需Anti-Windup
- TIM1是高级定时器需HAL_TIM_PWM_Start使能MOE