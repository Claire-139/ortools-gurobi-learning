import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# 读取结果
df = pd.read_csv('results.csv')

# 画柱状图
x = range(len(df))
width = 0.35

fig, ax = plt.subplots(figsize=(12, 6))

bars1 = ax.bar([i - width/2 for i in x], df['ortools_distance'], width, label='OR-Tools', color='steelblue')
bars2 = ax.bar([i + width/2 for i in x], df['nn_distance'], width, label='最近邻算法', color='coral')

ax.set_xlabel('实例')
ax.set_ylabel('总行驶距离')
ax.set_title('OR-Tools vs 最近邻算法 - Solomon C类实例对比')
ax.set_xticks(x)
ax.set_xticklabels(df['instance'])
ax.legend()

plt.tight_layout()
plt.savefig('comparsion.png')
plt.show()