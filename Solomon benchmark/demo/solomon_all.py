import os 
import csv



import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def read_solomon(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    # 读取车辆信息
    vehicle_info = lines[4].split()  # 把第5行按空格切开
    vehicle_count = int(vehicle_info[0]) 
    vehicle_capacity = int(vehicle_info[1])

    # 读节点数据，从第9行开始
    nodes = []
    for line in lines[9:]:
        if line.strip() == '': # 跳过空行，strip清楚空格
            continue
        parts = line.split()
        nodes.append({
            'id':           int(parts[0]),
            'x':            int(parts[1]),
            'y':            int(parts[2]),
            'demand':       int(parts[3]),
            'ready_time':   int(parts[4]),
            'due_time':     int(parts[5]), # horizon值node[0]时
            'service_time': int(parts[6]),
        })
    return nodes, vehicle_count, vehicle_capacity

# ----------绘图部分------
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

def plot_routes(routes, coordinates):
    fig, ax = plt.subplots(figsize=(12, 10))

    for i, (x, y) in enumerate(coordinates):
        if i == 0: # 仓库
            ax.plot(x, y, 's', color='red', markersize=10)
        else:
            ax.plot(x, y, 'o', color='blue', markersize=8)
        # ax.text(x+1, y+1, str(i))
    
    colors = [cm.tab20(i / vehicle_count) for i in range(vehicle_count)]
    for vehichle_id, route in enumerate(routes):
        if len(route) <= 2: # 跳过空路线
            continue
        for i in range(len(route) - 1):
            x1, y1 = coordinates[route[i]]
            x2, y2 = coordinates[route[i + 1]]
            ax.annotate('', xy=(x2, y2), xytext=(x1, y1), arrowprops=dict(arrowstyle='->', color=colors[vehichle_id]))
        
    plt.savefig('routes.png')
    plt.show()

def nearest_neighbor(nodes, vehicle_capacity, distance_matrix):
    unvisited = list(range(1, len(nodes)))
    routes = []

    while unvisited:
        # 派一辆新车
        current = 0 # 从仓库出发
        load = 0 # 车是空的
        route = [0] # 线路从0开始
        current_time = 0 # 当前时间

        while True:
            best_node = None
            best_dist = float('inf')
            # 找最近的、装得下的客户
            for node in unvisited:
                travel_time = distance_matrix[current][node]
                arrival = max(current_time + travel_time, nodes[node]['ready_time'])
                if (nodes[node]['demand'] + load <= vehicle_capacity) and (arrival <= nodes[node]['due_time']):
                    dist = distance_matrix[current][node]
                    if dist < best_dist:
                        best_dist = dist
                        best_node = node
                        
            # 去那个客户
            if best_node is not None:
                # 计算到达时间
                travel_time = distance_matrix[current][best_node]
                arrival = max(current_time + travel_time, nodes[best_node]['ready_time'])
                current_time = arrival + nodes[best_node]['service_time'] # 离开时间

                route.append(best_node)
                unvisited.remove(best_node)
                load += nodes[best_node]['demand']
                current = best_node
            # 找不到回仓库
            else:
                route.append(0)
                break

        
        routes.append(route)
    return routes

# 距离矩阵
def build_distance_matrix(nodes):
    n = len(nodes)
    matrix = []
    for i in range(n):
        row = []
        for j in range(n):
            # 计算节点ij之间距离
            dist = math.sqrt((nodes[i]['x'] - nodes[j]['x']) ** 2 + (nodes[i]['y'] - nodes[j]['y']) ** 2)
            row.append(dist)
        matrix.append(row)
    return matrix


# 建模
def solve_vrptw(nodes, vehicle_count, vehicle_capacity, verbose=False):
    # 计算距离矩阵

    
    # 调用距离矩阵
    distance_matrix = build_distance_matrix(nodes)

    # 创建路径模型
    manager = pywrapcp.RoutingIndexManager(len(nodes), vehicle_count, 0)
    routing = pywrapcp.RoutingModel(manager)

    # 定义距离回调函数
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(int(from_index))
        to_node = manager.IndexToNode(int(to_index))
        return int(distance_matrix[from_node][to_node])
    
    # 注册距离约束
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 定义需求回调函数
    def demand_callback(from_index):
        from_node = manager.IndexToNode(int(from_index))
        return nodes[from_node]['demand']

    # 注册需求约束
    need_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimension(need_callback_index, 0, vehicle_capacity, True, "Capacity")

    # 定义时间回调函数
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(int(from_index))
        to_node = manager.IndexToNode(int(to_index))
        return int(distance_matrix[from_node][to_node]) + nodes[from_node]['service_time']
    
    # 时间上限
    horizon = nodes[0]['due_time']

    # 注册时间约束
    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(time_callback_index, horizon, horizon, False, "Time")

    # 时间窗
    time_dimension = routing.GetDimensionOrDie("Time")
    for i in range(1, len(nodes)):
        index = manager.NodeToIndex(i)
        time_dimension.CumulVar(index).SetRange(
            nodes[i]['ready_time'], # earliest
            nodes[i]['due_time']    # latest
        )

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_params)

    if solution:
        total_distance = 0
        total_demand = 0
        capacity_dimension = routing.GetDimensionOrDie('Capacity')
        # 记录所有客户是否都跑了
        all_routes = []

        for vehicle_id in range(vehicle_count):
            index = routing.Start(vehicle_id)
            route = []
            route_distance = 0
            route_time = []
            start_index = routing.Start(vehicle_id)
            start_time = solution.Min(time_dimension.CumulVar(start_index))
            route_time.append(start_time)
            
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

                arrival = solution.Min(time_dimension.CumulVar(index))
                route_time.append(arrival)
            route.append(0)
            route_demand = solution.Min(capacity_dimension.CumulVar(previous_index))
            if verbose:
                print(f"车辆{vehicle_id + 1}路径: {route}, 距离: {route_distance}, 需求：{route_demand}, 到达时间{route_time}")
            total_distance += route_distance
            total_demand += route_demand
            all_routes.append(route)
        
        all_customers = []
        for route in all_routes:
            all_customers.extend([n for n in route if n != 0])
        if verbose:
            print(f'访问客户数：{len(all_customers)}')
            print(f'是否全覆盖：{sorted(all_customers) == list(range(1, 101))}')
            print(f"总距离: {total_distance}")
            print(f"总需求: {total_demand}")
            coordinates = [(node['x'], node['y']) for node in nodes]
            plot_routes(all_routes, coordinates)
        return all_routes, total_distance
    else:
        print("无可行解：约束条件无法同时满足")
        return [], 0



# 批量测试
instance_dir = r'Solomon benchmark\solomon-100\In'
results = []

for filename in sorted(os.listdir(instance_dir)):
    if not filename.startswith('c1') or not filename.endswith('.txt'):
        continue

    filepath = os.path.join(instance_dir, filename)
    instance_name = filename.replace('.txt', '').upper()

    print(f'\n正在处理{instance_name}...')

    # 读数据
    nodes, vehicle_count, vehicle_capacity = read_solomon(filepath)
    distance_matrix = build_distance_matrix(nodes)

    # ortools求解
    ortools_routes, ortools_distance = solve_vrptw(nodes, vehicle_count, vehicle_capacity)

    # 最近邻求解
    nn_routes = nearest_neighbor(nodes, vehicle_capacity, distance_matrix)
    nn_distance = sum(
        distance_matrix[route[i]][route[i + 1]]
        for route in nn_routes
        for i in range(len(route) - 1)
    )


    improvement = (nn_distance - ortools_distance) / nn_distance * 100

    results.append({
        'instance': instance_name,
        'ortools_distance': round(ortools_distance, 2),
        'nn_distance': round(nn_distance, 2),
        'ortools_vehicles': len([r for r in ortools_routes if len(r) > 2]),
        'nn_vehicles': len(nn_routes),
        'improvement': round(improvement, 2)
    })
    print(f'{instance_name}: OR-Tools={ortools_distance}, NN={nn_distance:.2f}, 提升={improvement:.1f}%')


# 保存csv
with open('results.csv', 'w', newline='', encoding='utf-8') as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

print('\n结果已保存到results.csv')