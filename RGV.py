import numpy as np


class TrackSegment:
    def __init__(self, start, end, dist0=0.0, type="straight", center=None):
        self.start = np.array(start, dtype=float)  # 起点坐标(mm)
        self.end = np.array(end, dtype=float)  # 终点坐标(mm)
        self.type = type  # straight/curve
        self.center = np.array(center) if center else None  # 圆弧弯道的圆心(mm)
        self.next_segment = None  # 环形轨道的下一段
        self.dist0 = dist0  # 此轨道起点相对总环形轨道0点的坐标(mm)
        self.dist1 = 0  # 此轨道终点相对总环形轨道0点的坐标(mm)

        if self.type == "curve":
            if self.center is None:
                raise ValueError("曲线段必须指定圆心坐标")
            # 计算半径
            self.radius = np.linalg.norm(self.start - self.center)
            # 验证终点到圆心距离
            if not np.isclose(self.radius, np.linalg.norm(self.end - self.center)):
                raise ValueError(
                    f"终点到圆心距离({np.linalg.norm(self.end - self.center)})不等于半径({self.radius})"
                )
            # 计算起始角度和终止角度
            self.start_angle = np.arctan2(
                self.start[1] - self.center[1], self.start[0] - self.center[0]
            )
            self.end_angle = np.arctan2(
                self.end[1] - self.center[1], self.end[0] - self.center[0]
            )
            angle_diff = self.end_angle - self.start_angle
            self.angle_span = (angle_diff + np.pi) % (2 * np.pi) - np.pi

            # 此轨道终点相对总环形轨道0点的坐标(mm), 弧长 = 角度 * 半径
            self.dist1 = self.dist0 + abs(self.angle_span) * self.radius
        else:
            # 此轨道终点相对总环形轨道0点的坐标(mm)
            self.dist1 = self.dist0 + np.linalg.norm(self.start - self.end)
            # 直线段不需要这些参数
            self.radius = None
            self.start_angle = None
            self.end_angle = None
            self.angle_span = None

        # print(self.dist0, self.dist1)


class RGV:
    def __init__(self, system, id: int, dist=0.0):
        self.length = 1880.73
        self.width = 1442.0
        self.id = id
        self.system = system
        self.position = np.array((0, 0), dtype=float)  # 当前轨道位置(m)
        self.dist = dist  # 当前位置dist
        self.speed = 0.0  # 当前速度(m/s)
        self.state = "idle"  # 状态机：idle/accel/decel/cruise
        self.task = None  # 当前任务
        self.target = dist  # 目的地的dist
        self.carry = False  # RGV是负载还是空载

        self.max_speed_empty = 180 / 60  # 最大空载速度（直线） 180m/min
        self.max_speed_loaded = 120 / 60  # 最大负载速度（直线） 120m/min
        self.max_speed_curve = 40 / 60  # 最大弯道速度 40m/min
        self.carry_accel = 0.5 / 1000  # 负载加速度 0.5m/s^2
        self.empty_accel = 1.0 / 1000  # 空载加速度 1.0m/s^2
        self.min_gap_s = 100 + self.length  # 最小跟车距离（直线轨道）
        self.min_gap_c = 1000 + self.length  # 最小跟车距离（圆弧轨道）
        self.current_segment = system.track[0]

    def assign_task(self, new_task, time):
        if self.task == new_task:
            return
        # 释放原有任务
        if self.task:
            if self.task.state == "pending":
                self.task.state = "queued"  # 释放回队列
        # 分配新任务
        self.task = new_task
        self.task.start_time = time
        self.task.now_time = time
        new_task.state = "pending"
        pos = self.system.stations[new_task.start]
        start_dist = self.system.pos_to_dist(pos)
        self.target = start_dist
        print(
            f"[{self.task.now_time/1000:.1f}s] \033[36m分配任务 RGV {self.id}: {new_task.start} -> {new_task.end}\033[0m"
        )
        # 写入 dispatch.csv
        # print(f"{self.id},{new_task.start},{new_task.end}")

    def get_task(self):
        """用于动画显示的任务字符串"""
        if self.task:
            return f"\n{self.task.start}->{self.task.end}"
        return ""

    def update(self, dt: int):
        """
        经过dt时间后的RGV

        参数：

        dt : int 时间间隔(ms)
        """
        # 获取前车信息
        front_rgv = self.get_front_vehicle()
        distance_to_front = self.system.A_to_B(self.dist, front_rgv.dist)
        if self.task:
            self.task.now_time += dt

        min_gap = self.get_min_gap()
        # 当前为停车
        if self.state == "idle":
            # 到达目的地
            if np.isclose(self.dist, self.target):
                if self.task:
                    if self.task.pick_station:
                        pos = self.system.stations[self.task.pick_station]
                        pick_dist = self.system.pos_to_dist(pos)
                        if np.isclose(self.dist, pick_dist):
                            # 需要停车等待
                            if self.task.pick_time == 0:
                                print(
                                    f"[{self.task.now_time/1000:.1f}s] \033[31mRGV {self.id} 在站台 {self.task.pick_station} 开始拣选！\033[0m"
                                )
                            if self.task.pick_time < 10 * 60 * 1000:  # 10分钟，毫秒
                                self.task.pick_time += dt
                                return
                    if self.carry:
                        print(
                            f"[{self.task.now_time/1000:.1f}s] \033[32mRGV {self.id} 任务完成！{self.task.start} -> {self.task.end}\033[0m"
                        )
                        self.task.state = "finished"
                        self.carry = False
                        self.task = None
                    else:
                        self.carry = True
                        self.task.state = "transporting"
                        pos = self.system.stations[self.task.end]
                        end_dist = self.system.pos_to_dist(pos)
                        self.target = end_dist
                        print(
                            f"[{self.task.now_time/1000:.1f}s] \033[32mRGV {self.id} 在 {self.task.start} 站台处装载！\033[0m"
                        )
                return
            # 没到目的地，但是停车了
            if distance_to_front <= min_gap + 5 and (
                front_rgv.state == "idle" or front_rgv.state == "decel"
            ):
                self.give_way()
                return
            # 前车还在运动，或者距离前车还很远，那就起步
            self.state = "accel"
            self.accelerate(dt)
        # 当前为加速
        elif self.state == "accel":
            de, stop_dis = self.will_decelerate(front_rgv)
            # 转变为 减速
            if de:
                self.state = "decel"
                self.decelerate(stop_dis, dt)
            # 转变为 匀速: 上次加速加到了最大速度
            elif np.isclose(self.speed, self.get_speed_limit()):
                self.state = "cruise"
                self.cruise(dt)
            # 继续 加速
            else:
                self.state = "accel"
                self.accelerate(dt)
        # 当前为减速
        elif self.state == "decel":
            de, stop_dis = self.will_decelerate(front_rgv)
            # 继续 减速
            if de:
                self.state = "decel"
                self.decelerate(stop_dis, dt)
            # 转变为 匀速: 弯道前减速，减到了弯道最大速度
            elif np.isclose(self.speed, self.get_speed_limit()):
                self.state = "cruise"
                self.cruise(dt)
            # 转变为 加速
            else:
                self.state = "accel"
                self.accelerate(dt)
        # 当前为匀速
        else:
            de, stop_dis = self.will_decelerate(front_rgv)
            # 转变为 减速
            if de:
                self.state = "decel"
                self.decelerate(stop_dis, dt)
            # 继续 匀速
            elif np.isclose(self.speed, self.get_speed_limit()):
                self.state = "cruise"
                self.cruise(dt)
            # 转变为 加速
            else:
                self.state = "accel"
                self.accelerate(dt)

    def get_min_gap(self):
        """获取当前最小跟车距离"""
        type0 = self.current_segment.type
        type1 = self.get_front_vehicle().current_segment.type
        if type0 == "curve" or type1 == "curve":
            return self.min_gap_c
        return self.min_gap_s

    def get_brake_dis(self) -> float:
        if self.state == "idle":
            return 0
        # v^2 = 2ax
        if self.carry:
            return self.speed**2 / (2 * self.carry_accel)
        else:
            return self.speed**2 / (2 * self.empty_accel)

    def will_decelerate(self, front_rgv):
        """是否需要减速: 前车安全距离，目的地距离，弯道起点距离"""
        inf = 1e18
        if np.isclose(0, self.speed):
            return False, inf
        # 计算距离前车距离
        distance_to_front = self.system.A_to_B(self.dist, front_rgv.dist)
        # 提前让前车让路！
        accel = self.carry_accel if self.carry else self.empty_accel
        if (
            distance_to_front + self.get_brake_dis()
            <= self.speed**2 / accel + self.get_min_gap()
            and (front_rgv.state == "idle" or front_rgv.state == "decel")
        ):
            self.give_way()
        # 保持安全距离
        if (
            distance_to_front + front_rgv.get_brake_dis()
            <= self.get_brake_dis() + self.get_min_gap()
        ):
            dis = front_rgv.dist - self.get_min_gap()
            if dis < 0:
                dis += self.system.max_dist
            return True, dis
        # 到目的地停车
        dis = self.system.A_to_B(self.dist, self.target)
        if self.get_brake_dis() >= dis:
            return True, self.target
        # 弯道提前减速
        if self.current_segment.next_segment.type == "curve":
            if self.speed > self.max_speed_curve:
                if self.carry:
                    # 求减速距离 v^2 - u^2 = 2ax
                    de_x = (self.speed**2 - self.max_speed_curve**2) / (
                        2 * self.carry_accel
                    )
                    if de_x <= self.current_segment.dist1 - self.dist:
                        return True, inf
                else:
                    # 求减速距离 v^2 - u^2 = 2ax
                    de_x = (self.speed**2 - self.max_speed_curve**2) / (
                        2 * self.empty_accel
                    )
                    if de_x >= self.current_segment.dist1 - self.dist:
                        return True, inf
        return False, inf

    def get_speed_limit(self):
        """当前状况允许加速到的最大速度"""
        if self.current_segment.type == "curve":
            return self.max_speed_curve
        elif self.carry:
            return self.max_speed_loaded
        else:
            return self.max_speed_empty

    def cruise(self, dt: int):
        """匀速行驶dt时间后的RGV"""
        self.dist += self.speed * dt
        # 更新当前轨道段
        if self.dist >= self.current_segment.dist1:
            self.current_segment = self.current_segment.next_segment
        if self.dist >= self.system.max_dist:
            self.dist -= self.system.max_dist

    def accelerate(self, dt: int):
        """加速dt时间后的RGV"""
        accel = self.carry_accel if self.carry else self.empty_accel
        speed_limit = self.get_speed_limit()

        dspeed = accel * dt  # v' = v + a * t
        speed = min(self.speed + dspeed, speed_limit)
        self.dist += (self.speed + speed) * dt / 2  # x = 1/2 * (v + v') * dt
        # 更新当前轨道段
        if self.dist >= self.current_segment.dist1:
            self.current_segment = self.current_segment.next_segment
        if self.dist >= self.system.max_dist:
            self.dist -= self.system.max_dist
        self.speed = speed

    def decelerate(self, stop_dist, dt: int):
        """减速dt时间后的RGV"""
        accel = self.carry_accel if self.carry else self.empty_accel

        dspeed = -accel * dt  # v' = v - a * t
        speed = max(self.speed + dspeed, 0)
        self.dist += min(
            self.system.A_to_B(self.dist, stop_dist), (self.speed + speed) * dt / 2
        )  # x = 1/2 * (v + v') * dt
        # 更新当前轨道段
        if self.dist >= self.current_segment.dist1:
            self.current_segment = self.current_segment.next_segment
        if self.dist >= self.system.max_dist:
            self.dist -= self.system.max_dist
        self.speed = speed
        if np.isclose(self.speed, 0):
            self.state = "idle"

    def get_front_vehicle(self):
        if self.id == 1:
            return self.system.vehicles[-1]
        return self.system.vehicles[self.id - 2]

    def get_back_vehicle(self):
        if self.id == self.system.vehicles[-1].id:
            return self.system.vehicles[0]
        return self.system.vehicles[self.id]

    def give_way(self):
        """前面闲车给我让路！"""
        f_rgv = self.get_front_vehicle()
        if f_rgv.task:
            return

        dist = self.target + self.min_gap_c
        if dist >= self.system.max_dist:
            dist -= self.system.max_dist
        f_rgv.target = dist

    def calculate_safe_speed(self, distance):
        # 根据跟车距离计算安全速度
        # 使用运动学公式：v² = u² + 2as
        min_gap = self.get_min_gap()
        max_decel_speed = np.sqrt(2 * self.decel * (distance - min_gap))
        return min(max_decel_speed, self.get_max_speed())

    def get_max_speed(self):
        """根据是否空载确定最大速度"""
        return self.max_speed_loaded if self.carry else self.max_speed_empty


class RGVSystem:
    def __init__(self):
        self.track = []  # 存储所有轨道段
        self.vehicles = []  # 存储所有RGV对象
        self.stations = {}  # 站点名称到轨道位置的映射
        self.max_dist = 0  # 轨道总长

    def add_track_segment(self, segment: TrackSegment):
        """初始化轨道的位置和形状"""
        self.track.append(segment)
        if len(self.track) > 1:
            self.track[-2].next_segment = segment
        # 环形闭合处理
        if len(self.track) > 2:
            self.track[-1].next_segment = self.track[0]
        self.max_dist = segment.dist1

    def add_station(self, id, pos):
        """初始化站点位置"""
        self.stations[id] = np.array(pos, dtype=float)

    def init_rgv(self):
        """初始化rgv的位置"""
        if (not self.track) or (not self.stations):
            raise RuntimeError("请先初始化轨道和站点位置！")
        # 作业区1：2辆RGV
        dist = self.pos_to_dist(self.stations[1])
        rgv = RGV(self, 1, dist=dist)
        # rgv.target = self.pos_to_dist(self.stations[33])
        self.vehicles.append(rgv)

        dist = self.pos_to_dist(self.stations[1]) - rgv.min_gap_s
        rgv = RGV(self, 2, dist=dist)
        self.vehicles.append(rgv)

        # 立体库区：4辆RGV
        dist = self.pos_to_dist(self.stations[19])
        rgv = RGV(self, 3, dist=dist)
        self.vehicles.append(rgv)

        dist = self.pos_to_dist(self.stations[19]) - rgv.min_gap_c + self.max_dist
        rgv = RGV(self, 4, dist=dist)
        self.vehicles.append(rgv)

        dist = self.pos_to_dist(self.stations[19]) - rgv.min_gap_c * 2 + self.max_dist
        rgv = RGV(self, 5, dist=dist)
        self.vehicles.append(rgv)

        dist = self.pos_to_dist(self.stations[19]) - rgv.min_gap_c * 3 + self.max_dist
        rgv = RGV(self, 6, dist=dist)
        self.vehicles.append(rgv)

        # 作业区3：1辆RGV
        dist = self.pos_to_dist(self.stations[16])
        rgv = RGV(self, 7, dist=dist)
        self.vehicles.append(rgv)

        # 作业区2：1辆RGV
        dist = self.pos_to_dist(self.stations[13])
        rgv = RGV(self, 8, dist=dist)
        self.vehicles.append(rgv)

    def A_to_B(self, start_dist, end_dist):
        dis = end_dist - start_dist
        if dis < 0:
            dis += self.max_dist
        return dis

    def dist_to_pos(self, dist):
        track = None
        for t in self.track:
            if t.dist0 <= dist and dist < t.dist1:
                track = t
                break
        if track is None:
            raise ValueError(f"dist 数值错误：{dist}")
        dist -= track.dist0
        if track.type == "straight":
            length = track.dist1 - track.dist0
            dx = (track.end[0] - track.start[0]) * dist / length
            dy = (track.end[1] - track.start[1]) * dist / length
            pos = track.start + np.array((dx, dy))
            return pos
        else:
            dangle = np.sign(track.angle_span) * dist / track.radius
            return rotate_point(track.start, track.center, dangle)

    def pos_to_dist(self, pos):
        for t in self.track:
            if t.type == "straight":
                if is_point_on_segment(t.start, t.end, pos):
                    return t.dist0 + np.linalg.norm(pos - t.start)
            else:
                # 点到圆心的欧氏距离等于半径
                if np.isclose(t.radius, np.linalg.norm(pos - t.center)):
                    # 计算起始角度和终止角度
                    angle = np.arctan2(pos[1] - t.center[1], pos[0] - t.center[0])
                    angle_diff = angle - t.start_angle
                    angle_span = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                    if np.sign(angle_span) == np.sign(t.angle_span) and abs(
                        angle_span
                    ) <= abs(t.angle_span):
                        # 此轨道终点相对总环形轨道0点的坐标(mm), 弧长 = 角度 * 半径
                        return t.dist0 + abs(angle_span) * t.radius
        raise ValueError(f"点不在轨道上！{pos}")


def rotate_point(point: np.ndarray, center: np.ndarray, theta: float) -> np.ndarray:
    """
    将二维点 `point` 绕 `center` 旋转 `theta` 弧度，返回旋转后的新坐标。

    参数：
    - point: shape (2,) 的 np.array，表示原始点坐标 (x, y)
    - center: shape (2,) 的 np.array，表示旋转中心 (cx, cy)
    - theta: 旋转角度（弧度，逆时针为正）

    返回：
    - new_point: shape (2,) 的 np.array，表示旋转后的坐标
    """
    # 平移至原点
    translated = point - center

    # 旋转矩阵
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

    # 应用旋转
    rotated = rotation_matrix @ translated

    # 平移回原坐标系
    new_point = rotated + center

    return new_point


def is_point_on_segment(A, B, P):
    """判断点P是否在点A和B为端点的直线段上"""
    # 处理线段退化为点的情况
    if abs(A[0] - B[0]) < 1e-9 and abs(A[1] - B[1]) < 1e-9:
        return abs(P[0] - A[0]) < 1e-9 and abs(P[1] - A[1]) < 1e-9

    # 向量计算
    ab_x = B[0] - A[0]
    ab_y = B[1] - A[1]
    ap_x = P[0] - A[0]
    ap_y = P[1] - A[1]

    # 叉积判断共线（精度容差 1e-9）
    cross = ab_x * ap_y - ab_y * ap_x
    if abs(cross) > 1e-9:
        return False

    # 点积判断位置范围
    dot = ap_x * ab_x + ap_y * ab_y
    ab_len_sq = ab_x**2 + ab_y**2

    return -1e-9 <= dot <= ab_len_sq + 1e-9
