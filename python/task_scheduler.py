"""
task_scheduler.py — 赛题任务状态机
根据 M_Task.md 赛题要求，管理各任务的目标点切换与计时。

Task 4: 固定位置停留（闭环稳定）
Task 5: 裁判指定目标（先中心 → 鼠标点击指定 → 计时到达）
Task 6: 直线往复运动（>10cm，两点间周期切换）
Task 7: 颠球（纯机械控制，Python 只发坐标）
"""

import time
import math


# ---- 判定"稳定"的参数 ----
STABLE_DIST_THRESHOLD = 10.0   # 球距目标 < 10mm 判定为到达
STABLE_FRAMES_NEEDED = 25      # 连续 25 帧（约 0.5s）稳定才算真正到达

# ---- Task 6 往复运动参数 ----
RECIPROCATE_A = (100.0, 150.0)  # 往复点 A (mm)
RECIPROCATE_B = (200.0, 150.0)  # 往复点 B (mm)


class TaskStateMachine:
    """赛题任务状态机"""

    def __init__(self):
        self.current_task = 0         # 0=空闲, 4-7=对应赛题
        self.state = "IDLE"           # IDLE / READY / RUNNING / DONE
        self.target_x = 150.0
        self.target_y = 150.0
        self.start_time = 0.0
        self.elapsed = 0.0
        self.stable_count = 0         # 连续稳定帧计数
        self.t_frame_pending = False  # 需要发送 T 帧的标记

        # Task 6 专用
        self._reciprocate_index = 0   # 0=去A, 1=去B

        # Task 5 专用
        self._referee_target_set = False  # 裁判是否已指定目标

    # ---- 公开接口 ----

    def switch_task(self, task_id):
        """切换到指定任务，重置所有状态"""
        if task_id not in (0, 4, 5, 6, 7):
            return

        self.current_task = task_id
        self.state = "IDLE" if task_id == 0 else "READY"
        self.target_x = 150.0
        self.target_y = 150.0
        self.start_time = 0.0
        self.elapsed = 0.0
        self.stable_count = 0
        self.t_frame_pending = True  # 切换任务时发送一次 T 帧
        self._reciprocate_index = 0
        self._referee_target_set = False

    def start(self):
        """按 Space 开始当前任务"""
        if self.state == "READY":
            self.state = "RUNNING"
            self.start_time = time.time()
            self.stable_count = 0
            self.t_frame_pending = True

    def reset(self):
        """按 R 重置当前任务"""
        if self.current_task != 0:
            self.switch_task(self.current_task)

    def set_referee_target(self, phys_x, phys_y):
        """
        Task 5: 鼠标点击设置裁判指定目标点。
        phys_x, phys_y 是透视变换后的物理坐标 (mm)。
        """
        if self.current_task == 5 and self.state == "RUNNING":
            self.target_x = phys_x
            self.target_y = phys_y
            self._referee_target_set = True
            self.start_time = time.time()  # 重新计时
            self.stable_count = 0
            self.t_frame_pending = True

    def update(self, ball_x, ball_y, now):
        """
        每帧调用。返回状态字典供 vision.py 使用。
        ball_x, ball_y: 球的当前物理坐标 (mm)，未找到球时传 None。
        now: time.time()
        """
        result = {
            "target_x": self.target_x,
            "target_y": self.target_y,
            "send_t_frame": False,
            "task_name": self.get_task_name(),
            "state": self.state,
            "timer_text": "",
            "distance": 0.0,
        }

        if self.current_task == 0 or self.state == "IDLE":
            return result

        # 计算球到目标的距离
        dist = 0.0
        if ball_x is not None and ball_y is not None:
            dist = math.sqrt((ball_x - self.target_x) ** 2 +
                             (ball_y - self.target_y) ** 2)
        result["distance"] = dist

        # 计时
        if self.state == "RUNNING":
            self.elapsed = now - self.start_time
        result["timer_text"] = f"{self.elapsed:.1f}s"

        # 分发到各任务处理函数
        if self.state == "RUNNING":
            if self.current_task == 4:
                self._update_task4(ball_x, ball_y, dist)
            elif self.current_task == 5:
                self._update_task5(ball_x, ball_y, dist)
            elif self.current_task == 6:
                self._update_task6(ball_x, ball_y, dist)
            # Task 7 不需要特殊逻辑

        # 检查是否有待发送的 T 帧
        if self.t_frame_pending:
            result["send_t_frame"] = True
            self.t_frame_pending = False

        # 更新返回值中的目标坐标（可能被任务逻辑修改了）
        result["target_x"] = self.target_x
        result["target_y"] = self.target_y

        return result

    # ---- 各任务逻辑 ----

    def _update_task4(self, ball_x, ball_y, dist):
        """Task 4: 固定位置停留"""
        if ball_x is not None and dist < STABLE_DIST_THRESHOLD:
            self.stable_count += 1
        else:
            self.stable_count = 0

        # 稳定后标记完成（但不停止，让用户看效果）
        if self.stable_count >= STABLE_FRAMES_NEEDED:
            self.state = "DONE"

    def _update_task5(self, ball_x, ball_y, dist):
        """Task 5: 裁判指定目标"""
        if not self._referee_target_set:
            # 等待鼠标点击设置目标, 先保持在中心
            return

        if ball_x is not None and dist < STABLE_DIST_THRESHOLD:
            self.stable_count += 1
        else:
            self.stable_count = 0

        if self.stable_count >= STABLE_FRAMES_NEEDED:
            self.state = "DONE"

    def _update_task6(self, ball_x, ball_y, dist):
        """Task 6: 往复运动"""
        if ball_x is None:
            return

        # 到达当前 waypoint → 切换到另一个
        if dist < STABLE_DIST_THRESHOLD:
            self.stable_count += 1
            if self.stable_count >= 10:  # 短暂停留即切换
                self._reciprocate_index = 1 - self._reciprocate_index
                if self._reciprocate_index == 0:
                    self.target_x, self.target_y = RECIPROCATE_A
                else:
                    self.target_x, self.target_y = RECIPROCATE_B
                self.stable_count = 0
                self.t_frame_pending = True
        else:
            self.stable_count = 0

    # ---- 辅助 ----

    def get_task_name(self):
        names = {
            0: "Idle",
            4: "4-Hold Position",
            5: "5-Referee Target",
            6: "6-Reciprocate",
            7: "7-Bouncing Ball",
        }
        return names.get(self.current_task, "Unknown")
