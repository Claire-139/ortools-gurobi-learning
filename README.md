# OR Tools & Gurobi 学习项目

学习运筹优化工具，包含OR-Tools和Gurobi的建模实践。

## 项目结构

## OR-Tools
- `ortools/vrp_demo.py`：基础VRP
- `ortools/cvrp_demo.py`：带载重限制的VRP（CVRP）
- `ortools/vrptw_demo.py`：带时间窗的VRP（VRPTW）
- `ortools/multi-depot_vrp.py`：多起点VRP + 可视化
- `ortools/job_shop_scheduling.py`：Job Shop调度（CP-SAT）

## Gurobi
- `Gurobi/LP_demo.py`：线性规划（运输问题）
- `Gurobi/ILP_demo.py`：整数规划（背包问题）
- `Gurobi/jobshop_demo.py`：Job Shop调度（大M法）
- `Gurobi/GA_demo.py`：遗传算法（背包问题）

## 完整项目
- `Solomon benchmark/demo/solomon.py`：Solomon C101单实例VRPTW求解，100个客户节点
- `Solomon benchmark/demo/solomon_all.py`：批量测试C101-C109共9个实例，OR-Tools与最近邻算法对比
- `Solomon benchmark/demo/solomon_NN.py`：含时间窗约束的最近邻启发式算法实现
- `Solomon benchmark/demo/visualization.py`：算法对比柱状图可视化

## 结果
- OR-Tools在9个Solomon C类实例上平均较最近邻算法优化53%，最高提升59%（C102）
- 可视化对比图见`comparsion.png`，详细数据见`results.csv`
  
## 环境
- Python 3.x
- ortools
- gurobipy（需要学术License）
- matplotlib
- pandas

## 运行方式
python ortools/vrp_demo.py
python Solomon\ benchmark/demo/solomon.py
