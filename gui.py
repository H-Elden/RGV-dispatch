import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.gridspec import GridSpec


class Visualizer:
    def __init__(self, system, dispatcher):
        self.system = system
        self.dispatcher = dispatcher
        # 设置窗口大小和DPI
        plt.rcParams["figure.dpi"] = 100  # 确保显示清晰度

        # 使用GridSpec创建左右分屏布局
        self.fig = plt.figure(figsize=(11, 10), facecolor="white")
        gs = GridSpec(1, 2, width_ratios=[8, 2], wspace=0.05)
        self.ax = self.fig.add_subplot(gs[0])  # 左侧动画区域
        self.ax_right = self.fig.add_subplot(gs[1])  # 右侧任务区域

        self.info_annotations = []  # 存储信息文本对象
        self.offset_states = {}  # 偏移状态记录
        # 正确设置窗口标题（兼容不同后端）
        self.fig.canvas.manager.set_window_title("RGV simulation")

        # 初始化轨道线条（统一黑色，线宽2）
        (self.track_line,) = self.ax.plot([], [], "k-", lw=2, solid_joinstyle="round")
        self.vehicle_patches = []
        self.text_annotations = []  # 存储文本对象
        self.task_texts = []  # 存储任务文本对象

        # 添加时间文本初始化
        self.time_text = self.ax.text(
            0.95,
            0.95,  # 使用相对坐标（右上角）
            "",
            transform=self.ax.transAxes,  # 使用轴坐标系
            ha="right",  # 右对齐
            va="top",  # 顶部对齐
            fontsize=12,
            color="#333333",
            bbox=dict(
                facecolor="white",
                alpha=0.8,
                edgecolor="lightgray",
                boxstyle="round,pad=0.3",
            ),
            zorder=20,  # 确保在最上层
        )

        # 初始化坐标范围
        self.x_min, self.x_max = np.inf, -np.inf
        self.y_min, self.y_max = np.inf, -np.inf
        self.setup_plot()
        # 初始化任务面板
        self.init_task_panel()

    def init_task_panel(self):
        """初始化任务列表区域"""
        self.ax_right.axis("off")
        self.ax_right.set_xlim(0, 1)
        self.ax_right.set_ylim(0, 1)

        # 绘制表头
        self.ax_right.text(
            0.15, 0.95, "Tasks Queue", fontsize=12, weight="bold", color="#2F4F4F"
        )

    def setup_plot(self):
        """初始化画面最初点位信息"""
        # 绘制轨道并获取坐标范围
        x1, y1 = self.get_track_coords()
        x2, y2 = self.get_station_coords()
        x = x1 + x2
        y = y1 + y2
        self.track_line.set_data(x, y)

        # 计算坐标范围（考虑20%边距）
        margin = 0.2

        x_range = max(x) - min(x)
        y_range = max(y) - min(y)

        text_offset = 14000  # 小车ID和任务文字偏移量
        self.x_min = min(x) - x_range * margin - text_offset
        self.x_max = max(x) + x_range * margin + text_offset
        self.y_min = min(y) - y_range * margin - text_offset
        self.y_max = max(y) + y_range * margin + text_offset

        # 设置坐标轴属性
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.set_aspect("equal")
        self.ax.axis("off")  # 隐藏坐标轴

        # 初始化车辆图形
        for vehicle in self.system.vehicles:
            half_length = vehicle.length / 2
            half_width = vehicle.width / 2
            vehicle.position = self.system.dist_to_pos(vehicle.dist)
            x_center, y_center = vehicle.position
            patch = plt.Rectangle(
                (x_center - half_length, y_center - half_width),
                vehicle.length,
                vehicle.width,
                fc="none",  # 禁用填充（或 fc=(0,0,0,0) 透明填充）
                ec=(1, 0, 0),  # 边框颜色设为绿色
                lw=2,  # 边框线宽（默认1可能过细）
                zorder=5,
                rotation_point="center",  # 设置旋转中心
            )
            self.ax.add_patch(patch)
            self.vehicle_patches.append(patch)

            # 创建信息文本（初始位置将在更新时设置）
            txt = self.ax.text(
                0,
                0,  # 占位坐标
                f"ID: {vehicle.id}{vehicle.get_task()}",
                fontsize=8,
                color="white",
                ha="center",
                va="center",
                linespacing=1.2,
                bbox=dict(
                    facecolor="#1F77B4",
                    alpha=0.9,
                    edgecolor="none",
                    boxstyle="round,pad=0.3",
                ),
                rotation=0,  # 固定不旋转
                zorder=10,  # 确保文本在最上层
            )
            self.offset_states[vehicle.id] = 5000
            self.info_annotations.append(txt)

        # 优化布局
        plt.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.99)

    def get_track_coords(self):
        """获取轨道段的直角坐标"""
        x_coords = []
        y_coords = []
        for segment in self.system.track:
            if segment.type == "straight":
                # 优化直线段连接
                x_coords.extend([segment.start[0], segment.end[0], np.nan])
                y_coords.extend([segment.start[1], segment.end[1], np.nan])
            else:
                # 优化圆弧段生成（保证闭合）
                num_points = max(50, int(abs(segment.angle_span) * segment.radius))
                angles = np.linspace(
                    segment.start_angle,
                    segment.start_angle + segment.angle_span,
                    num_points,
                    endpoint=False,  # 避免终点重复
                )
                x = segment.center[0] + segment.radius * np.cos(angles)
                y = segment.center[1] + segment.radius * np.sin(angles)

                # 处理方向并添加分隔符
                if segment.angle_span < 0:
                    x = np.concatenate([x[::-1], [np.nan]])
                    y = np.concatenate([y[::-1], [np.nan]])
                else:
                    x = np.concatenate([x, [np.nan]])
                    y = np.concatenate([y, [np.nan]])

                x_coords.extend(x)
                y_coords.extend(y)

        return x_coords, y_coords

    def get_station_coords(self):
        """获取固定的站点的绘制线段，绘制站点编号"""
        x_coords = []
        y_coords = []
        for id, pos in self.system.stations.items():
            if id <= 12:
                x_coords.extend([pos[0], pos[0], np.nan])
                y_coords.extend([pos[1] - 200, pos[1] + 200, np.nan])
                self.add_text(pos[0] - 500, pos[1] + 1000, id, offset=8)
            elif id <= 18:
                x_coords.extend([pos[0] - 200, pos[0] + 200, np.nan])
                y_coords.extend([pos[1], pos[1], np.nan])
                self.add_text(pos[0] + 1000, pos[1] - 500, id, offset=8)
            else:
                x_coords.extend([pos[0] - 200, pos[0] + 200, np.nan])
                y_coords.extend([pos[1], pos[1], np.nan])
                self.add_text(pos[0] - 1000, pos[1] - 500, id, offset=8)

        return x_coords, y_coords

    def add_text(self, x, y, text, offset=5, **kwargs):
        """无边框坐标标注方法"""
        # 自动计算文本对齐方式
        ha = "left" if x >= 0 else "right"
        va = "bottom" if y >= 0 else "top"

        # 创建无边框文本对象
        txt = self.ax.text(
            x + offset * np.sign(x),  # 自动偏移
            y + offset * np.sign(y),
            text,
            fontsize=8,
            color="darkblue",
            ha=ha,
            va=va,
            **kwargs,
        )
        self.text_annotations.append(txt)
        return txt

    def get_vehicle_coords(self, rgv, dt=100):
        """实时获取小车坐标"""
        rgv.update(dt)
        rgv.position = self.system.dist_to_pos(rgv.dist)
        x, y = rgv.position

        # 由曲线斜率计算小车转角
        ddist = rgv.dist + 10
        if ddist >= self.system.max_dist:
            ddist -= self.system.max_dist
        dx, dy = self.system.dist_to_pos(ddist)
        theta = np.arctan2(dy - y, dx - x)  # 弧度制

        # 由小车当前速度大小展示不同颜色，0为红色，速度最大为绿色。实现加减速颜色渐变
        # 计算颜色渐变（红到绿）
        ratio = np.clip(rgv.speed / rgv.max_speed_empty, 0, 1)
        edge_color = (1 - ratio, ratio, 0)  # RGB格式

        # 由小车是否负载决定填充，负载：填充边框的颜色；空载：不填充。
        face_color = edge_color if rgv.carry else "none"
        return x, y, theta, edge_color, face_color

    def update_task_panel(self):
        """更新任务列表内容"""
        # 清空旧内容但保留表头
        for txt in self.task_texts:
            txt.remove()
        self.task_texts = []

        queue = self.dispatcher.show_tasks()  # 获取任务数据
        y_pos = 0.90  # 起始Y坐标

        for q in queue:
            # 构造显示字符串
            main_part = f"{q['task'].start}→{q['task'].end}"
            state_part = f" {q['task'].state}"

            # 分两部分绘制不同颜色
            task_text = self.ax_right.text(
                0.1,
                y_pos,
                main_part,
                fontsize=10,
                color="black",
                transform=self.ax_right.transAxes,
            )
            state_text = self.ax_right.text(
                0.4,
                y_pos,  # 粗略计算位置
                state_part,
                fontsize=10,
                color=q["color"],
                transform=self.ax_right.transAxes,
            )

            self.task_texts.extend([task_text, state_text])
            y_pos -= 0.04  # 行间距

    def update_animation(self, frame):
        """更新1帧动画"""
        for i, vehicle in enumerate(self.system.vehicles):
            x, y, theta, ec, fc = self.get_vehicle_coords(vehicle, dt=100)
            # 更新位置，计算矩形左下角坐标
            rect = self.vehicle_patches[i]
            half_length = rect.get_width() / 2
            half_width = rect.get_height() / 2
            rect.set_xy((x - half_length, y - half_width))

            # 更新角度，转为角度制
            rect.angle = np.degrees(theta)
            # 更新颜色
            rect.set_edgecolor(ec)
            rect.set_facecolor(fc)

            # 动态计算偏移量
            current_offset = self.calculate_text_offset(vehicle, theta)
            self.offset_states[vehicle.id] = current_offset

            # 计算实际偏移方向（保持左侧）
            left_dir = theta + np.pi / 2
            dx = current_offset * np.cos(left_dir)
            dy = current_offset * np.sin(left_dir)

            # 更新文本位置
            txt = self.info_annotations[i]
            txt.set_position((x + dx, y + dy))
            txt.set_text(f"ID: {vehicle.id}{vehicle.get_task()}")  # 更新状态

        # 更新任务面板
        self.update_task_panel()

        # 更新时间显示
        current_time = self.dispatcher.get_time()  # 假设系统提供该方法
        self.time_text.set_text(f"Time: {current_time}")

        return (
            self.vehicle_patches
            + self.info_annotations
            + self.task_texts
            + [self.time_text]
        )

    def calculate_text_offset(self, vehicle, theta):
        # 获取前车对象
        prev_vehicle = vehicle.get_front_vehicle()

        # 判断方向是否水平（角度容差±30度）
        is_horizontal = np.abs(theta % (2 * np.pi)) < np.radians(30) or np.abs(
            theta % (2 * np.pi) - np.pi
        ) < np.radians(30)

        # 初始化默认偏移
        base_offset = 6000  # 偏移量1
        alt1_offset = 10000  # 偏移量2
        alt2_offset = 14000  # 偏移量3

        # 计算车距
        distance = prev_vehicle.dist - vehicle.dist
        if distance < 0:
            distance += vehicle.system.max_dist

        # 动态调整逻辑
        if is_horizontal:
            if distance < 7000:
                if self.offset_states[prev_vehicle.id] == base_offset:
                    return alt1_offset
                elif self.offset_states[prev_vehicle.id] == alt1_offset:
                    return alt2_offset
                return base_offset
            return base_offset
        else:
            if distance < 5000:
                if self.offset_states[prev_vehicle.id] == base_offset:
                    return alt1_offset
                elif self.offset_states[prev_vehicle.id] == alt1_offset:
                    return alt2_offset
                return base_offset
            return base_offset

    def start_animation(self):
        """开启动画仿真"""
        ani = animation.FuncAnimation(
            self.fig,
            self.update_animation,
            cache_frame_data=False,
            interval=100,
            blit=True,
        )
        plt.show()
