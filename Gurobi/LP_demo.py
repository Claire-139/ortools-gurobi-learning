# 运输问题
# 2个工厂：工厂0（供应量30），工厂1（供应量40）
# 4个客户：需求量分别是10, 20, 15, 25
# 单位运输成本矩阵（工厂i → 客户j）：

#        客户0  客户1  客户2  客户3
# 工厂0  [  2,    3,    1,    4  ]
# 工厂1  [  3,    2,    4,    1  ]
# 目标：总运输成本最小

import gurobipy as gp
from gurobipy import GRB

# ----------基础数据-----------
supply = [30, 40]  # 工厂0（供应量30），工厂1（供应量40）
demand = [10, 20, 15, 25] # 4个客户：需求量分别是10, 20, 15, 25
cost = [
    [2, 3, 1, 4],
    [3, 2, 4, 1],
]
# 单位运输成本矩阵

# 1. 创建模型
model = gp.Model()

# 2. 添加决策变量
x = {}  # 待求解变量
for i in range(len(supply)):
    for j in range(len(demand)):
        x[i, j] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f'x_{i}_{j}')

# 3. 设置目标函数
# 总运输成本 = 所有路线的 成本 × 运输量 之和
# model.setObjective(表达式, 方向)
model.setObjective(gp.quicksum(cost[i][j] * x[i, j] for i in range(2) for j in range(4)), GRB.MINIMIZE)
# `quicksum`是Gurobi版的`sum()`
# GRB.MINIMIZE  # 最小化（运输成本当然越小越好）
# GRB.MAXIMIZE  # 最大化（如果目标是利润就用这个）

# 4. 添加约束addConstr
# 供应约束，对每个工厂i
for i in range(len(supply)):
    model.addConstr(gp.quicksum(x[i, j] for j in range(len(demand))) <= supply[i])

# 需求约束：对每个客户j
for j in range(len(demand)):
    model.addConstr(gp.quicksum(x[i, j] for i in range(len(supply))) == demand[j])

# 5. 求解
model.optimize()

# ---------打印----------
if model.Status == GRB.OPTIMAL:
    print(f'最优运输成本：{model.ObjVal}')
    for i in range(len(supply)):
        for j in range(len(demand)):
            if x[i, j].x > 0:
                print(f'工厂{i} -> 客户{j}: {x[i, j].x}')
# x[i,j].x  # 求解后读出来的实际数值