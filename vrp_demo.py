from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# 距离矩阵：5个地点 （0是仓库，1-4是客户） 没有看懂？
distance_matrix = [
    [0, 10, 15, 20, 25], # 0号点到各点的距离：到0是0，到1是10，到2是15...
    [10, 0, 35, 25, 30],
    [15, 35, 0, 30, 20],
    [20, 25, 30, 0, 15],
    [25, 30, 20, 15, 0],
]
# distance_matrix[i][j] 代表从地点 i 到地点 j 的距离。

# 创建路径模型：5个地点，1辆车，仓库在0号
# manager = pywrapcp.RoutingIndexManager(5, 1, 0)
manager = pywrapcp.RoutingIndexManager(5, 2, 0)
# 经理负责把“内部索引”翻译成我们的“地点编号”。
routing = pywrapcp.RoutingModel(manager) # 定义决策变量 x_ij
# Model（模型）： 它是“计算器”。
# 后续所有的约束条件、目标函数都往这个 routing 模型里塞。

# 定义距离回调函数（告诉模型如何计算距离）
def distance_callback(from_index, to_index):
    # step 1 把内部索引翻译成我们能看懂的地点编号
    from_node = manager.IndexToNode(int(from_index))
    to_node = manager.IndexToNode(int(to_index))
    # step 2 去距离矩阵里查表，返回距离
    return distance_matrix[from_node][to_node] # 定义目标函数中的系数c_ij

# 把这个函数注册给模型，拿到一个索引
# Callback（回调）： 这是一个“查表员”。求解器在算题时，会不停地问：“如果从 A 走到 B 多少钱？”。这个函数就是回答这个问题的。
transit_callback_index = routing.RegisterTransitCallback(distance_callback)
# 告诉模型：把这个距离作为“成本”（Cost）
# 目标就是让所有路段的“成本”加起来最小
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# 设置搜索策略,基于贪心算法PATH_CHEAPEST_ARC再优化
search_params = pywrapcp.DefaultRoutingSearchParameters()
search_params.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# 求解
solution = routing.SolveWithParameters(search_params)

# 输出结果
# if solution:
    # index = routing.Start(0) # 从0开始
    # route = []
    # while not routing.IsEnd(index): # 只要没有到终点
        # route.append(manager.IndexToNode(index)) # 把点记录
        # index = solution.Value(routing.NextVar(index)) # 下一点去哪
    # route.append(0) # 回到终点
    # print(f"最优路径: {route}")
    # print(f"总距离: {solution.ObjectiveValue()}")

if solution:
    total_distance = 0
    for vehicle_id in range(2): # 遍历车辆
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            previous_index = index  # 第一步：记住我现在站的位置（起点）
            index = solution.Value(routing.NextVar(index)) # 第二步：跳到下一个位置（终点）
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            # “获取（Get）指定车辆（Vehicle）在某段弧（Arc）上的成本（Cost）”
        route.append(0)
        print(f"车辆{vehicle_id+1}路径: {route}, 距离: {route_distance}")
        total_distance += route_distance
    print(f"总距离: {total_distance}")