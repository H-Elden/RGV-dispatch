import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


class Visualizer:
    def __init__(self, system):
        self.system = system
        # 设置窗口大小和DPI
        plt.rcParams["figure.dpi"] = 100  # 确保显示清晰度
        self.fig, self.ax = plt.subplots(figsize=(10, 10), facecolor="white")

        # 正确设置窗口标题（兼容不同后端）
        self.fig.canvas.manager.set_window_title("RGV simulation")

        # 初始化轨道线条（统一黑色，线宽2）
        (self.track_line,) = self.ax.plot([], [], "k-", lw=2, solid_joinstyle="round")
        self.vehicle_patches = []
        self.text_annotations = []  # 存储文本对象

        # 初始化坐标范围
        self.x_min, self.x_max = np.inf, -np.inf
        self.y_min, self.y_max = np.inf, -np.inf
        self.setup_plot()

    def setup_plot(self):
        # 绘制轨道并获取坐标范围
        x1, y1 = self.get_track_coords()
        x2, y2 = self.get_station_coords()
        x = x1 + x2
        y = y1 + y2
        self.track_line.set_data(x, y)

        # 计算坐标范围（考虑10%边距）
        margin = 0.1
        x_range = max(x) - min(x)
        y_range = max(y) - min(y)
        self.x_min = min(x) - x_range * margin
        self.x_max = max(x) + x_range * margin
        self.y_min = min(y) - y_range * margin
        self.y_max = max(y) + y_range * margin

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

        # 优化布局
        plt.subplots_adjust(left=0.03, right=0.97, bottom=0.03, top=0.97)

    def get_track_coords(self):
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
        """无边框版本的坐标标注方法"""
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
            **kwargs,  # 移除了bbox参数
        )
        self.text_annotations.append(txt)
        return txt

    def get_vehicle_coords(self, rgv, dt=100):
        # rgv.update(dt)
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

    def update_animation(self, frame):
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
        return self.vehicle_patches

    def start_animation(self):
        ani = animation.FuncAnimation(
            self.fig,
            self.update_animation,
            cache_frame_data=False,
            interval=100,
            blit=True,
        )
        plt.show()
