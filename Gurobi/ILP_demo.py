'''有5个物品，背包有两个限制：重量和体积

物品   重量  体积  价值
物品0   2    3    4
物品1   3    2    5
物品2   4    5    3
物品3   1    2    2
物品4   5    3    6

背包限制：最大重量10，最大体积12
目标：总价值最大'''

import gurobipy as gp
from gurobipy import GRB

# ----------数据层----------
weights = [2, 3, 4, 1, 5]
volumes = [3, 2, 5, 2, 3]
values = [4, 5, 3, 2, 6]
max_weight = 10
max_volume = 12

# ----------建模层----------
# 创建模型
model = gp.Model()

# 创建决策变量
y = {}
for i in range(len(weights)):
    y[i] = model.addVar(vtype=GRB.BINARY, name=f'y_{i}')

# 目标函数
model.setObjective(gp.quicksum(values[i] * y[i] for i in range(len(weights))), GRB.MAXIMIZE)

# 约束
# 重量约束（所有物品之和，不需要遍历）
model.addConstr(gp.quicksum(weights[i] * y[i] for i in range(len(weights))) <= max_weight)

# 体积约束
model.addConstr(gp.quicksum(volumes[i] * y[i] for i in range(len(volumes))) <= max_volume)

# -------求解层-----------
model.optimize()

# --------提取结果----------
if model.Status == GRB.OPTIMAL:
    print(f'最优总价值：{model.ObjVal}')
    for i in range(len(weights)):
        if y[i].x > 0.5:  # 二进制变量用0.5作为判断阈值
            print(f'选择物品{i}：重量{weights[i]}，体积{volumes[i]},价值{values[i]}')