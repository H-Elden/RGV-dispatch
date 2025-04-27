class Task:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.state = "queued"  # queued(尚未分配任务，位于全局队列)/pending(已分配车辆赶往起点)/transporting(运输中)/finished(已完成)
        if 1 <= start <= 12:
            self.pick_station = start
        elif 1 <= end <= 12:
            self.pick_station = end
        else:
            self.pick_station = 0
        self.pick_time = 0
        self.start_time = 0
        self.now_time = 0


class Dispatcher:
    def __init__(self, system):
        self.system = system
        self.task_queue = []
        self.current_sorted_tasks = []  # 维护当前最优任务序列
        self.need_resort = True  # 是否需要重新排序的标记
        self.time = 0  # 当前时间（毫秒）

    def add_task(self, task_start, task_end):
        """添加新任务并标记需要重新排序"""
        self.task_queue.append(Task(task_start, task_end))
        self.need_resort = True

    def assign_task(self, time):
        self.time = time
        # 动态决定是否需要重新排序
        if self.need_resort or not self.current_sorted_tasks:
            # 筛选待处理任务
            todo_list = [t for t in self.task_queue if t.state in ["queued", "pending"]]
            if not todo_list:
                return
            self.current_sorted_tasks = self.best_sort(todo_list)
            self.need_resort = False
            # 准备可用资源
            idle_vehicles = [v for v in self.system.vehicles if not v.carry]
            available_tasks = self.current_sorted_tasks
        else:
            idle_vehicles = [v for v in self.system.vehicles if v.task == None]
            available_tasks = [
                t for t in self.current_sorted_tasks if t.state == "queued"
            ]

        # 确定实际可分配数量
        num_assign = min(len(available_tasks), len(idle_vehicles))
        if num_assign == 0:
            return

        # 执行任务分配
        for i in range(num_assign):
            task = available_tasks[i]
            pos = self.system.stations[task.start]
            start_dist = self.system.pos_to_dist(pos)

            # 选择最优车辆
            best_vehicle = min(
                idle_vehicles, key=lambda v: self.system.A_to_B(v.dist, start_dist)
            )

            # 更新状态
            best_vehicle.assign_task(task, time)
            idle_vehicles.remove(best_vehicle)

    def best_sort(self, tasks):
        return tasks

    def show_tasks(self):
        queue = []
        for t in self.task_queue:
            if t.state == "queued":
                queue.append({"task": t, "color": "orange"})
            elif t.state == "pending":
                queue.append({"task": t, "color": "blue"})
            elif t.state == "transporting":
                queue.append({"task": t, "color": "green"})
            elif t.state == "finished":
                queue.append({"task": t, "color": "grey"})
        num = 0
        for q in queue:
            if q["task"].state == "finished":
                num += 1
            else:
                break
        num = max(0, num - 2)
        # 删去finished
        queue = queue[num:]
        # 只保留前20个任务
        queue = queue[:20]
        return queue

    def get_time(self):
        time_ms = self.time
        if time_ms < 1000:
            return f"{time_ms}ms"
        elif time_ms < 60 * 1000:
            seconds = time_ms / 1000
            return f"{seconds:.1f}s"
        elif time_ms < 3600 * 1000:
            minutes = time_ms // (60 * 1000)
            remaining = time_ms % (60 * 1000)
            seconds = remaining / 1000
            return f"{minutes}m{seconds:.1f}s"
        elif time_ms < 24 * 3600 * 1000:
            hours = time_ms // (3600 * 1000)
            remaining = time_ms % (3600 * 1000)
            minutes = remaining // (60 * 1000)
            remaining = remaining % (60 * 1000)
            seconds = remaining / 1000
            return f"{hours}h {minutes}m {seconds:.1f}s"
        else:
            days = time_ms // (24 * 3600 * 1000)
            remaining = time_ms % (24 * 3600 * 1000)
            hours = remaining // (3600 * 1000)
            remaining = remaining % (3600 * 1000)
            minutes = remaining // (60 * 1000)
            remaining = remaining % (60 * 1000)
            seconds = remaining / 1000

            parts = []
            parts.append(f"{days}day")
            if hours > 0:
                parts.append(f"{hours}h")

            min_sec = []
            if minutes > 0:
                min_sec.append(f"{minutes}m")
            # 始终显示秒部分
            min_sec.append(f"{seconds:.1f}s")
            min_sec_str = "".join(min_sec)

            # 组合天/小时部分和分钟/秒部分
            day_hour_str = "".join(parts)
            if day_hour_str and min_sec_str:
                return f"{day_hour_str} {min_sec_str}"
            else:
                return day_hour_str + min_sec_str
