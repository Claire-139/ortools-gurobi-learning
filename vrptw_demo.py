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

# 节点间行驶时间（以分钟为例）
time_matrix = distance_matrix

# 每个节点的时间窗[(earliest, latest), ...]
# 仓库设(0, 480)就是8小时工作制且以分钟为单位
time_windows = [(0, 480), (50, 260), (0, 150), (110, 410), (0, 480), (200, 450)]

# 每个节点的服务时间
service_times = [0, 15, 20, 5, 30, 50]


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

# 定义时间回调
def time_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    # a到b的时间消耗 = 路程时间 + 在a停留多久
    return time_matrix[from_node][to_node] + service_times[from_node]

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

# 时间callback注册函数
# 与距离的相同
time_callback_index = routing.RegisterTransitCallback(time_callback)
routing.AddDimension(time_callback_index, 480, 480, False, "Time")
# 第二个表示需要等待多久，直接设置成足够大的数
# 第三个参数表示当前维度的上限，一般直接用仓库时间的上限
# 第四个参数时间维度一般填False——因为车不一定必须在时间0出发

# 每个节点设置时间窗
time_dimension = routing.GetDimensionOrDie("Time") # 用名字把刚才注册的维度取出来
# OrDie的意思是：如果找不到这个名字的维度，直接报错
for i in range(1,6):
    index = manager.NodeToIndex(i) # 把地点编码翻译成内部索引
    # CumulVar(index)：某个节点上，累计值的变量 
    # Cumul = Cumulative（累积的）Var = Variable（变量）
    # SetRange(earliest, latest)：给这个问号设置范围
    # CumulVar(节点1).SetRange(50, 260) = 到达节点1的时间，必须在50到260分钟之间
    time_dimension.CumulVar(index).SetRange(
        time_windows[i][0],  # earliest
        time_windows[i][1]   # latest
    )

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

    for vehicle_id in range(2): # 遍历车辆
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0
        route_demand = 0
        route_time = [] # 记录到达的时间
        # 汽车出发时间
        start_index = routing.Start(vehicle_id)
        # 求解之后cumulvar变成一个具体数值，求解之前是一个待求解的变量
        start_time = solution.Min(time_dimension.CumulVar(start_index))
        route_time.append(start_time)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            previous_index = index  # 第一步：记住我现在站的位置（起点）
            index = solution.Value(routing.NextVar(index)) # 第二步：跳到下一个位置（终点）
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            # “获取（Get）指定车辆（Vehicle）在某段弧（Arc）上的成本（Cost）”
            # 先把内部编号换成节点编号,记录累计需求
            node = manager.IndexToNode(index)
            route_demand += demands[node]
            # 时间
            arrival = solution.Min(time_dimension.CumulVar(index))
            route_time.append(arrival)
        route.append(0)
        print(f"车辆{vehicle_id+1}路径: {route}, 距离: {route_distance}, 需求：{route_demand}, 到达时间{route_time}")
        total_distance += route_distance
        total_demand += route_demand
    print(f"总距离: {total_distance}")
    print(f"总需求: {total_demand}")

else:
    print("无可行解：约束条件无法同时满足")