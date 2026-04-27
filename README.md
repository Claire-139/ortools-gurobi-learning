# OR Tools & Gurobi 学习项目

学习运筹优化工具，包含OR-Tools和Gurobi的建模实践。

## 项目结构

### OR-Tools
- `ortools/vrp_demo.py`：基础VRP
- `ortools/cvrp_demo.py`：带载重限制的VRP（CVRP）
- `ortools/vrptw_demo.py`：带时间窗的VRP（VRPTW）
- `ortools/multi-depot_vrp.py`：多起点VRP + 可视化
- `ortools/job_shop_scheduling.py`：Job Shop调度（CP-SAT）

### Gurobi
- `Gurobi/LP_demo.py`：线性规划（运输问题）
- `Gurobi/ILP_demo.py`：整数规划（背包问题）
- `Gurobi/jobshop_demo.py`：Job Shop调度（大M法）
- `Gurobi/GA_demo.py`：遗传算法（背包问题）

### 完整项目
- `Solomon benchmark/demo/solomon.py`：基于Solomon C101
  数据集的VRPTW求解，100个客户节点，总行驶距离809

## 环境
- Python 3.x
- ortools
- gurobipy（需要学术License）
- matplotlib

## 运行方式
python ortools/vrp_demo.py
python Solomon\ benchmark/demo/solomon.py
