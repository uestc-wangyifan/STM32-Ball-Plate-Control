"""
telemetry.py — 实时遥测波形模块
功能：用 matplotlib 实时绘制四条曲线：
  1. X 坐标（当前 vs 目标）
  2. Y 坐标（当前 vs 目标）
  3. X 误差
  4. Y 误差

用法：
  在 vision.py 中 import 后，每帧调用 push() 推数据，
  定期调用 update_plot() 刷新波形。按 W 键 toggle() 开关。
"""

import matplotlib  # type: ignore
matplotlib.use("TkAgg")  # 非阻塞后端
import matplotlib.pyplot as plt  # type: ignore
from collections import deque
import time


class Telemetry:
    """实时遥测波形绘制器"""

    def __init__(self, max_points: int = 500, update_interval: float = 0.08):
        """
        Args:
            max_points: 保留最近 N 帧数据
            update_interval: 最少间隔多少秒刷新一次图（防止卡顿）
        """
        self.max_points = max_points
        self.update_interval = update_interval

        # 数据缓冲
        self.t_data = deque(maxlen=max_points)
        self.ball_x = deque(maxlen=max_points)
        self.ball_y = deque(maxlen=max_points)
        self.tgt_x = deque(maxlen=max_points)
        self.tgt_y = deque(maxlen=max_points)
        self.err_x = deque(maxlen=max_points)
        self.err_y = deque(maxlen=max_points)

        self._visible = False
        self._fig = None
        self._axes = None
        self._lines = {}
        self._last_update = 0.0
        self._start_time = time.time()

    def push(self, ball_x: float | None, ball_y: float | None,
             target_x: float, target_y: float, timestamp: float):
        """推入一帧数据"""
        t = timestamp - self._start_time
        bx = ball_x if ball_x is not None else float("nan")
        by = ball_y if ball_y is not None else float("nan")

        self.t_data.append(t)
        self.ball_x.append(bx)
        self.ball_y.append(by)
        self.tgt_x.append(target_x)
        self.tgt_y.append(target_y)
        self.err_x.append(target_x - bx)
        self.err_y.append(target_y - by)

    def toggle(self):
        """切换波形窗口显示/隐藏"""
        if self._visible:
            self._close()
        else:
            self._open()

    def update_plot(self):
        """非阻塞刷新波形（受 update_interval 限速）"""
        if not self._visible or self._fig is None:
            return
        now = time.time()
        if now - self._last_update < self.update_interval:
            return
        self._last_update = now

        if len(self.t_data) < 2:
            return

        t = list(self.t_data)

        # 子图1: X坐标
        self._lines["ball_x"].set_data(t, list(self.ball_x))
        self._lines["tgt_x"].set_data(t, list(self.tgt_x))

        # 子图2: Y坐标
        self._lines["ball_y"].set_data(t, list(self.ball_y))
        self._lines["tgt_y"].set_data(t, list(self.tgt_y))

        # 子图3: X误差
        self._lines["err_x"].set_data(t, list(self.err_x))

        # 子图4: Y误差
        self._lines["err_y"].set_data(t, list(self.err_y))

        # 自动缩放
        for ax in self._axes:
            ax.relim()
            ax.autoscale_view()

        try:
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception:
            self._visible = False

    @property
    def is_visible(self) -> bool:
        return self._visible

    def _open(self):
        """创建 matplotlib 图窗"""
        plt.ion()
        self._fig, axes = plt.subplots(2, 2, figsize=(10, 6))
        self._fig.canvas.manager.set_window_title("Telemetry — 实时遥测波形")
        self._fig.set_facecolor("#1e1e1e")
        self._axes = axes.flatten()

        titles = ["X 坐标 (mm)", "Y 坐标 (mm)", "X 误差 (mm)", "Y 误差 (mm)"]
        for i, ax in enumerate(self._axes):
            ax.set_title(titles[i], color="white", fontsize=10)
            ax.set_facecolor("#2d2d2d")
            ax.tick_params(colors="white", labelsize=8)
            ax.spines["bottom"].set_color("#555")
            ax.spines["left"].set_color("#555")
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.grid(True, alpha=0.2, color="white")

        # X坐标子图
        self._lines["ball_x"], = self._axes[0].plot([], [], "c-", lw=1.2, label="Ball X")
        self._lines["tgt_x"], = self._axes[0].plot([], [], "y--", lw=1, label="Target X")
        self._axes[0].legend(loc="upper right", fontsize=7, facecolor="#333", edgecolor="#555",
                             labelcolor="white")

        # Y坐标子图
        self._lines["ball_y"], = self._axes[1].plot([], [], "c-", lw=1.2, label="Ball Y")
        self._lines["tgt_y"], = self._axes[1].plot([], [], "y--", lw=1, label="Target Y")
        self._axes[1].legend(loc="upper right", fontsize=7, facecolor="#333", edgecolor="#555",
                             labelcolor="white")

        # X误差子图
        self._lines["err_x"], = self._axes[2].plot([], [], "r-", lw=1, label="Error X")
        self._axes[2].axhline(y=0, color="white", lw=0.5, alpha=0.3)
        self._axes[2].legend(loc="upper right", fontsize=7, facecolor="#333", edgecolor="#555",
                             labelcolor="white")

        # Y误差子图
        self._lines["err_y"], = self._axes[3].plot([], [], "r-", lw=1, label="Error Y")
        self._axes[3].axhline(y=0, color="white", lw=0.5, alpha=0.3)
        self._axes[3].legend(loc="upper right", fontsize=7, facecolor="#333", edgecolor="#555",
                             labelcolor="white")

        self._fig.tight_layout()
        plt.show(block=False)
        self._visible = True
        print("[Telemetry] 波形窗口已打开")

    def _close(self):
        """关闭 matplotlib 图窗"""
        if self._fig is not None:
            plt.close(self._fig)
            self._fig = None
        self._visible = False
        self._lines.clear()
        print("[Telemetry] 波形窗口已关闭")
