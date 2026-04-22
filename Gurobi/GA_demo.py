import random

# ------数据层------
weights = [2, 3, 4, 1, 5]
volumes = [3, 2, 5, 2, 3]
values = [4, 5, 3, 2, 6]
max_weight = 10
max_volume = 12

# 随机生成一个 个体
def create_individual():
    return [random.randint(0, 1) for _ in range(len(weights))]

# 计算适应度
# 一个个体的适应度 = 总价值
# 但如果超重或超体积，适应度 = 0（不合法的解直接淘汰）

def fitness(individual):
    total_weight = sum(weights[i] * individual[i] for i in range(len(weights)))
    total_volume = sum(volumes[i] * individual[i] for i in range(len(volumes)))
    total_value = sum(values[i] * individual[i] for i in range(len(values)))
    if total_weight > max_weight or total_volume > max_volume:
        return 0
    return total_value


# 初始化种群
def create_population(size):
    return [create_individual() for _ in range(size)]

# 选择
# 常用方法：锦标赛选择（Tournament Selection）
# 从种群里随机抽k个个体，选出适应度最高的那个作为父代。

def tournament_select(population, k=3):
    candidates = random.sample(population, k)
    return max(candidates, key=fitness)
# random.sample：从种群里不重复地抽k个
# max(..., key=fitness)：选适应度最高的。

# 单点交叉（Single Point Crossover）
# 随机一个切割点，切割点以前来自A，之后来自B
def crossover(parent_a, parent_b):
    point = random.randint(1, len(parent_a) - 1) # 随机切割点，避开两端
    child = parent_a[:point] + parent_b[point:]
    return child

# 变异（Mutation）
# 以一定概率（比如0.1）随机翻转某个基因
def mutate(individual, rate=0.1):
    for i in range(len(individual)):
        if random.random() < rate: # 每个基因有10%概率变异
            individual[i] = 1 - individual[i] # 0→1，1→0
    return individual

# 主循环
def genetic_algorithm(pop_size=50, generations=100):
    # 初始化种群
    population = create_population(pop_size)

    for gen in range(generations):
        new_population = []
        for _ in range(pop_size):
            # 选择两个父代
            parent_a = tournament_select(population)
            parent_b = tournament_select(population)
            # 交叉生成子代
            child = crossover(parent_a, parent_b)
            # 变异
            child = mutate(child)
            new_population.append(child)
        population = new_population
    
    # 返回最优个体
    best = max(population, key=fitness)
    return best, fitness(best)

best, score = genetic_algorithm()
print(f'遗传算法最优解：{best}')
print(f'总价值：{score}')
    
    