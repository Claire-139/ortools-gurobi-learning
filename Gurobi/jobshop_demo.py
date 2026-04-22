
from collections import defaultdict

import gurobipy as gp
from gurobipy import GRB

# --------数据层--------

jobs = [
    [(0, 3), (1, 2), (2, 2)],
    [(0, 2), (2, 1), (1, 4)],
    [(1, 4), (2, 3)],
]

horizon = 0
for job in jobs:
    for task, duration in job:
        horizon += duration


# -------建模层-----------
start = {}
end = {}
model = gp.Model()

for job_id, job in enumerate(jobs):
    for task_id, (machine, duration) in enumerate(job):
        start[job_id, task_id] = model.addVar(lb=0, ub=horizon, vtype=GRB.INTEGER, name=f'start_{job_id}_{task_id}')
        end[job_id, task_id] = model.addVar(lb=0, ub=horizon, vtype=GRB.INTEGER, name=f'end_{job_id}_{task_id}')
        # ub = Upper Bound（上界）不能超过horizon，lb（Lower Bound，下界)
        model.addConstr(end[job_id, task_id] == start[job_id, task_id] + duration)
        if task_id > 0:
            model.addConstr(start[job_id, task_id] >= end[job_id, task_id - 1])
            # 工序约束

# 机器不重叠约束:同一台机器上，任务A和任务B不能重叠。
machin_to_tasks = defaultdict(list)

for job_id, job in enumerate(jobs):
    for task_id, (machine, duration) in enumerate(job):
        machin_to_tasks[machine].append((job_id, task_id))

for machine, tasks in machin_to_tasks.items():
    for i in range(len(tasks)):
        for j in range(i + 1, len(tasks)): # 只取i<j的组合，避免重复
            a = tasks[i] # (job_id, task_id)
            b = tasks[j]

            # 创建二进制变量：z=1表示a在b之前
            z = model.addVar(vtype=GRB.BINARY, name=f'z_{a}_{b}')

            # 两个大M约束
            model.addConstr(start[b] >= end[a] - horizon * (1 - z))
            model.addConstr(start[a] >= end[b] - horizon * z)
            # 情况1：A先做，B后做 → start[B] >= end[A] 此时z = 1,对于条件2右边为负天然成立
            # 情况2：B先做，A后做 → start[A] >= end[B] 此时z = 0,对于条件1右边为负天然成立

# --------目标函数层-----
makespan = model.addVar(lb=0, ub=horizon, vtype=GRB.INTEGER, name='makespan')

# makespan >= 每个工件最后一个Task的结束时间
for job_id, job in enumerate(jobs):
    model.addConstr(makespan >= end[job_id, len(job) - 1])

# 目标函数
model.setObjective(makespan, GRB.MINIMIZE)

# -------求解层-------
model.optimize()

# --------提取结果----------
if model.status == GRB.OPTIMAL:
    print(f'最优总价值：{model.ObjVal}')
    for job_id, job in enumerate(jobs):
        for task_id, (machine, duration) in enumerate(job):
            s = int(start[job_id, task_id].x)
            e = int(end[job_id, task_id].x)
            print(f'工作{job_id} Task{task_id}：机器{machine}, {s}~{e}')