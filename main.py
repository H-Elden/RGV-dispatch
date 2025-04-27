from RGV import RGVSystem, TrackSegment
from dispatch import Dispatcher
from gui import Visualizer
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import itertools
import time


def init_track(system):
    """初始化轨道"""
    df = pd.read_csv("tracks.csv")

    lstend = 0.0
    for _, row in df.iterrows():
        start = (row["startx"], row["starty"])
        end = (row["endx"], row["endy"])
        seg_type = row["type"]

        if seg_type == "curve":
            center = (row["centerx"], row["centery"])
            # 处理缺失值
            if pd.isna(center[0]) or pd.isna(center[1]):
                raise ValueError(f"轨道段{row['index']}的圆心坐标缺失")
            segment = TrackSegment(
                start, end, dist0=lstend, center=center, type=seg_type
            )
        else:
            segment = TrackSegment(start, end, dist0=lstend, type=seg_type)
        lstend = segment.dist1
        system.add_track_segment(segment)


def init_stations(system):
    """初始化站点"""
    df = pd.read_csv("stations.csv")

    for _, row in df.iterrows():
        id = int(row["index"])
        pos = (row["x"], row["y"])

        system.add_station(id, pos)


def load_tasks():
    """从csv文件中读取全部任务数据"""
    df = pd.read_csv("tasks.csv")
    tasks = []
    for _, row in df.iterrows():
        task = (row["start"], row["end"])
        tasks.append(task)
    return tasks


def vis(system, dispatcher, tasks):
    # 创建可视化器
    visualizer = Visualizer(system)

    # 仿真参数
    sim_time = 0
    ani_speed = 100  # 动画倍速
    dt = 0.1  # 时间步长（秒）
    task_interval = 10  # 每10秒添加一个任务
    current_task_index = 0

    def update(frame):
        nonlocal sim_time, current_task_index

        # 1. 添加新任务（每10秒添加一个）
        if (
            current_task_index < len(tasks)
            and sim_time >= current_task_index * task_interval
        ):
            task_start, task_end = tasks[current_task_index]
            dispatcher.add_task(task_start, task_end)
            print(
                f"\n[{sim_time:.1f}s] \033[33m添加新任务：{task_start} -> {task_end}\033[0m"
            )
            current_task_index += 1

        # 2. 调度器分配任务
        dispatcher.assign_task(sim_time * 1000)

        # 3. 更新所有车辆状态（根据dt计算）
        for vehicle in system.vehicles:
            vehicle.update(dt * 1000)  # 转换为毫秒

        # 4. 更新仿真时间
        sim_time += dt

        # 5. 检查终止条件
        if current_task_index == len(tasks):
            if all(t.state == "finished" for t in dispatcher.task_queue):
                print(f"\n\033[32m--- 仿真完成 ---\033[0m")
                print(f"总耗时：{sim_time:.1f}秒")
                print(f"任务总数：{len(dispatcher.task_queue)}")
                simulation_done = True
                ani.event_source.stop()  # 终止动画
                return []

        # 6. 更新动画（返回需要重绘的对象）
        return visualizer.update_animation(frame)

    # 启动动画时绑定update函数
    ani = animation.FuncAnimation(
        visualizer.fig,
        update,
        frames=itertools.count(),  # 无限帧
        # frames=1,
        interval=dt * 1000 / ani_speed,  # 每帧间隔（毫秒）
        blit=True,
        cache_frame_data=False,
    )
    plt.show()


def unvis(system, dispatcher, tasks):
    # 仿真参数
    sim_time = 0
    dt = 0.1  # 时间步长（秒）
    task_interval = 10  # 每10秒添加一个任务
    current_task_index = 0
    while True:
        # 1. 添加新任务（每10秒添加一个）
        if (
            current_task_index < len(tasks)
            and sim_time >= current_task_index * task_interval
        ):
            task_start, task_end = tasks[current_task_index]
            dispatcher.add_task(task_start, task_end)
            print(
                f"[{sim_time:.1f}s] \033[33m添加新任务：{task_start} -> {task_end}\033[0m"
            )
            current_task_index += 1

        # 2. 调度器分配任务
        dispatcher.assign_task(sim_time * 1000)

        # 3. 更新所有车辆状态（根据dt计算）
        for vehicle in system.vehicles:
            vehicle.update(dt * 1000)  # 转换为毫秒

        # 4. 更新仿真时间
        sim_time += dt

        # 5. 检查终止条件
        if current_task_index == len(tasks):
            if all(t.state == "finished" for t in dispatcher.task_queue):
                print(f"\n\033[32m--- 仿真完成 ---\033[0m")
                print(f"总耗时：{sim_time:.1f}秒")
                print(f"任务总数：{len(dispatcher.task_queue)}")
                return


def main():
    # 初始化系统
    system = RGVSystem()
    # 添加轨道段
    init_track(system)
    # 初始化轨道上的出入库站点
    init_stations(system)
    # 初始化轨道上的RGV位置
    system.init_rgv()
    # 读入所有任务
    tasks = load_tasks()
    tasks = tasks[:30]

    # 创建调度器
    dispatcher = Dispatcher(system)
    start_time = time.time()  # 记录开始时间
    vis(system, dispatcher, tasks)
    end_time = time.time()  # 记录结束时间

    elapsed_time = end_time - start_time
    print(f"\n\n函数运行耗时: {elapsed_time:.6f} 秒")

    # for i, t in enumerate(dispatcher.task_queue):
    #     print(
    #         f"{i+1},{t.start_time/1000:.1f},{t.now_time/1000:.1f},{(t.now_time - t.start_time)/1000:.1f}"
    #     )


if __name__ == "__main__":
    main()
