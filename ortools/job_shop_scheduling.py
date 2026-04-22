# 调度问题 —— 任务如何安排

# 工件0：先去机器0加工3分钟 → 再去机器1加工2分钟 → 再去机器2加工2分钟
# 工件1：先去机器0加工2分钟 → 再去机器2加工1分钟 → 再去机器1加工4分钟
# 工件2：先去机器1加工4分钟 → 再去机器2加工3分钟

from collections import defaultdict

from ortools.sat.python import cp_model

# 每个工件是一个列表，每个元素是(机器编号, 加工时间)
# 每个任务有三个参数，开始时间，结束时间，占用的时间区间
# 其中前两个是求解变量，第三个是区间变量IntervalVar = (start_var, duration, end_var)

jobs = [
    [(0, 3), (1, 2), (2, 2)], # 工件0
    [(0, 2), (2, 1), (1, 4)], # 工件1
    [(1, 4), (2, 3)],         # 工件2
] 

# 所有任务加工时间之和
horizon =  0 
for job in jobs:
    for machine, duration in job:
        horizon += duration

# key是(工件编号, 任务编号)，value是(start_var, end_var, interval_var)
all_tasks = {}
model = cp_model.CpModel()

machine_to_intervals = defaultdict(list)

for job_id, job in enumerate(jobs):
    for task_id, (machine, duration) in enumerate(job):
        start_var = model.new_int_var(0, horizon, f'start_{job_id}_{task_id}')
        end_var = model.new_int_var(0, horizon, f'end_{job_id}_{task_id}')
        interval_var = model.new_interval_var(start_var, duration, end_var, f'interval_{job_id}_{task_id}')
        all_tasks[(job_id, task_id)] = (start_var, end_var, interval_var)
        machine_to_intervals[machine].append(interval_var)
        if task_id > 0:
            # model.add() 在往模型里添加一个约束
            model.add(all_tasks[(job_id, task_id)][0] >= all_tasks[(job_id, task_id - 1)][1])
            # 添加顺序约束
            # 当前Task的开始时间 ≥ 上一个Task的结束时间。
# 机器约束，一个机器不能同时干多个task            
for machine, intervals in machine_to_intervals.items():          
    model.add_no_overlap(intervals)

# --------目标函数------
# 要求所有工件都完成的最早时间（Makespan）最小。
makespan = model.new_int_var(0, horizon, 'makespan')

# makespan = 所有Task结束时间的最大值
model.add_max_equality(makespan, [
    all_tasks[(job_id, len(job) - 1)][1]
    for job_id, job in enumerate(jobs)
])
# makespan == MAX( end_0_2, end_1_2, end_2_1 ),工厂的整体完工时间等于这三者里的最大值
# all_tasks[(2, 1)] = (start_2_1, end_2_1, interval_2_1)

# 目标：最小化makespan
# 给我找出一个排程方案，让这个最大值越小越好
model.minimize(makespan)


# --------求解-------
solver = cp_model.CpSolver()  # 创建求解器
status = solver.solve(model)  # 开始求解，返回求解状态
# 会有三种可能：
# cp_model.OPTIMAL：找到最优解
# cp_model.FEASIBLE：找到可行解但不确定是最优
# cp_model.INFEASIBLE：无可行

if status == cp_model.OPTIMAL:
    print(f'最优解，总完工时间：{solver.value(makespan)}')
    for job_id, job in enumerate(jobs):
        for task_id, (machine, duration) in enumerate(job):
            start = solver.value(all_tasks[(job_id, task_id)][0])
            end = solver.value(all_tasks[(job_id, task_id)][1])
            print(f'工件{job_id} Task{task_id}: 机器{machine}, {start}~{end}')

elif status == cp_model.FEASIBLE:
    print(f'可行解（非最优），总完工时间: {solver.value(makespan)}')

else:
    print('无可行解')

