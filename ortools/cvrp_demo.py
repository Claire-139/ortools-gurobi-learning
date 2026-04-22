from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# 距离矩阵：6个地点 （0是仓库，1-5是客户） 
distance_matrix = [
    [0, 10, 15, 20, 25, 35], 
    [10, 0, 35, 25, 30, 15],
    [15, 35, 0, 30, 20, 10],
    [20, 25, 30, 0, 15, 25],
    [25, 30, 20, 15, 0, 10],
    [35, 15, 10, 25, 10, 0],
]
# 每个节点的货物需求
demands = [0, 10, 40, 25, 15, 10]

# 每辆车的最大载重（两辆车都是50kg）
vehicle_capacities = [50, 50]

# 创建路径模型：6个地点，2辆车，仓库在0号
manager = pywrapcp.RoutingIndexManager(6, 2, 0)
routing = pywrapcp.RoutingModel(manager) # 定义决策变量 x_ij

# 定义距离回调函数（告诉模型如何计算距离）
def distance_callback(from_index, to_index):
    # step 1 把内部索引翻译成我们能看懂的地点编号
    from_node = manager.IndexToNode(int(from_index))
    to_node = manager.IndexToNode(int(to_index))
    # step 2 去距离矩阵里查表，返回距离
    return distance_matrix[from_node][to_node] # 定义目标函数中的系数c_ij

# 定义需求回调
def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return demands[from_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
# 告诉模型：把这个距离作为“成本”（Cost）
# 目标就是让所有路段的“成本”加起来最小
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# 需求callback对应的注册函数
need_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
# 注册载重约束
routing.AddDimension(need_callback_index, 0, 50, True, "Capacity")
# AddDimension()第三个参数这里只接受一个整数作为上限，
# 如果每辆车载重不同，才需要用另一个方法AddDimensionWithVehicleCapacity()——那个才接受列表
# 第四个参数是布尔值，"起点的累计载重，要不要固定为0？"

# 设置搜索策略,基于贪心算法PATH_CHEAPEST_ARC再优化
search_params = pywrapcp.DefaultRoutingSearchParameters()
search_params.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# 求解
solution = routing.SolveWithParameters(search_params)

# 输出结果
if solution:
    total_distance = 0
    total_demand = 0
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    for vehicle_id in range(2): # 遍历车辆
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0

        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            previous_index = index  # 第一步：记住我现在站的位置（起点）
            index = solution.Value(routing.NextVar(index)) # 第二步：跳到下一个位置（终点）
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            
        route.append(0)
        route_demand = solution.Min(capacity_dimension.CumulVar(previous_index))
        print(f"车辆{vehicle_id+1}路径: {route}, 距离: {route_distance}, 需求：{route_demand}")
        total_distance += route_distance
        total_demand += route_demand
    print(f"总距离: {total_distance}")
    print(f"总需求: {total_demand}")
else:
    print("无可行解：约束条件无法同时满足")