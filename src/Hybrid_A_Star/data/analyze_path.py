#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题

# 读取CSV文件
file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_analysis.csv')
df = pd.read_csv(file_path)

# 分离原始路径和优化路径的数据
original_df = df[df['path_type'] == 'original']
optimized_df = df[df['path_type'] == 'optimized']

# 创建一个包含多个子图的图表
fig = plt.figure(figsize=(20, 16))
fig.suptitle('Path Parameter Comparison: Original vs Optimized', fontsize=16)

# 1. 路径轨迹图
ax1 = fig.add_subplot(2, 3, 1)
ax1.plot(original_df['x'].values, original_df['y'].values, 'b-', label='Original Path')
ax1.plot(optimized_df['x'].values, optimized_df['y'].values, 'r-', label='Optimized Path')
ax1.set_title('Path Trajectory Comparison')
ax1.set_xlabel('X Coordinate')
ax1.set_ylabel('Y Coordinate')
ax1.grid(True)
ax1.legend()
ax1.axis('equal')

# 2. 曲率对比图
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(original_df['path_length'].values, original_df['curvature'].values, 'b-', label='Original Path')
ax2.plot(optimized_df['path_length'].values, optimized_df['curvature'].values, 'r-', label='Optimized Path')
ax2.set_title('Path Curvature Comparison')
ax2.set_xlabel('Path Length')
ax2.set_ylabel('Curvature')
ax2.grid(True)
ax2.legend()

# 3. 障碍物距离对比图
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(original_df['path_length'].values, original_df['obstacle_distance'].values, 'b-', label='Original Path')
ax3.plot(optimized_df['path_length'].values, optimized_df['obstacle_distance'].values, 'r-', label='Optimized Path')
ax3.set_title('Obstacle Distance Comparison')
ax3.set_xlabel('Path Length')
ax3.set_ylabel('Obstacle Distance')
ax3.grid(True)
ax3.legend()

# 4. 航向角对比图
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(original_df['path_length'].values, original_df['theta'].values, 'b-', label='Original Path')
ax4.plot(optimized_df['path_length'].values, optimized_df['theta'].values, 'r-', label='Optimized Path')
ax4.set_title('Heading Angle Comparison')
ax4.set_xlabel('Path Length')
ax4.set_ylabel('Heading Angle (rad)')
ax4.grid(True)
ax4.legend()

# 5. 转向角度对比图
ax5 = fig.add_subplot(2, 3, 5)
ax5.plot(original_df['path_length'].iloc[1:].values, original_df['steering_angle'].iloc[1:].values, 'b-', label='Original Path')
ax5.plot(optimized_df['path_length'].iloc[1:].values, optimized_df['steering_angle'].iloc[1:].values, 'r-', label='Optimized Path')
ax5.set_title('Steering Angle Comparison')
ax5.set_xlabel('Path Length')
ax5.set_ylabel('Steering Angle (rad)')
ax5.grid(True)
ax5.legend()

# 6. 综合参数统计
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')  # 关闭坐标轴

# 计算一些统计指标
original_path_length = original_df['path_length'].max()
optimized_path_length = optimized_df['path_length'].max()
path_length_diff = ((optimized_path_length - original_path_length) / original_path_length) * 100

original_avg_curvature = original_df['curvature'].mean()
optimized_avg_curvature = optimized_df['curvature'].mean()
curvature_diff = ((optimized_avg_curvature - original_avg_curvature) / original_avg_curvature) * 100 if original_avg_curvature != 0 else float('inf')

original_max_curvature = original_df['curvature'].max()
optimized_max_curvature = optimized_df['curvature'].max()
max_curvature_diff = ((optimized_max_curvature - original_max_curvature) / original_max_curvature) * 100 if original_max_curvature != 0 else float('inf')

original_avg_obstacle_dist = original_df['obstacle_distance'].mean()
optimized_avg_obstacle_dist = optimized_df['obstacle_distance'].mean()
obstacle_dist_diff = ((optimized_avg_obstacle_dist - original_avg_obstacle_dist) / original_avg_obstacle_dist) * 100

original_min_obstacle_dist = original_df['obstacle_distance'].min()
optimized_min_obstacle_dist = optimized_df['obstacle_distance'].min()
min_obstacle_dist_diff = ((optimized_min_obstacle_dist - original_min_obstacle_dist) / original_min_obstacle_dist) * 100

original_steering_var = original_df['steering_angle'].var()
optimized_steering_var = optimized_df['steering_angle'].var()
steering_var_diff = ((optimized_steering_var - original_steering_var) / original_steering_var) * 100 if original_steering_var != 0 else float('inf')

# 创建文本表格
table_text = [
    ["Parameter", "Original Path", "Optimized Path", "Change (%)"],
    ["Total Path Length", f"{original_path_length:.2f}", f"{optimized_path_length:.2f}", f"{path_length_diff:.2f}%"],
    ["Average Curvature", f"{original_avg_curvature:.6f}", f"{optimized_avg_curvature:.6f}", f"{curvature_diff:.2f}%" if curvature_diff != float('inf') else "N/A"],
    ["Maximum Curvature", f"{original_max_curvature:.6f}", f"{optimized_max_curvature:.6f}", f"{max_curvature_diff:.2f}%" if max_curvature_diff != float('inf') else "N/A"],
    ["Average Obstacle Distance", f"{original_avg_obstacle_dist:.2f}", f"{optimized_avg_obstacle_dist:.2f}", f"{obstacle_dist_diff:.2f}%"],
    ["Minimum Obstacle Distance", f"{original_min_obstacle_dist:.2f}", f"{optimized_min_obstacle_dist:.2f}", f"{min_obstacle_dist_diff:.2f}%"],
    ["Steering Angle Variance", f"{original_steering_var:.6f}", f"{optimized_steering_var:.6f}", f"{steering_var_diff:.2f}%" if steering_var_diff != float('inf') else "N/A"]
]

# 计算统计表格
table = ax6.table(cellText=table_text,
                 colWidths=[0.25, 0.25, 0.25, 0.25],
                 cellLoc='center',
                 loc='center')
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1, 1.5)  # 调整表格大小

# 调整布局
plt.tight_layout(rect=[0, 0, 1, 0.95])  # 为标题留出空间

# 保存图表
plt.savefig(os.path.join(os.path.dirname(file_path), 'path_comparison.png'), dpi=300)

# 显示图表（如果是在有GUI的环境下运行）
plt.show()

print("Path parameter comparison analysis completed, chart saved to:", os.path.join(os.path.dirname(file_path), 'path_comparison.png'))

# 额外生成更详细的单独图表
def save_detailed_plot(x_data, y_data1, y_data2, title, xlabel, ylabel, filename, ylim=None):
    plt.figure(figsize=(12, 8))
    plt.plot(x_data, y_data1, 'b-', label='Original Path')
    plt.plot(x_data, y_data2, 'r-', label='Optimized Path')
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    if ylim is not None:
        plt.ylim(ylim)
    plt.savefig(os.path.join(os.path.dirname(file_path), filename), dpi=300)
    plt.close()

# 生成各项参数的单独详细图表
# 曲率详细图
max_len = min(original_df['path_length'].max(), optimized_df['path_length'].max())
original_filtered = original_df[original_df['path_length'] <= max_len]
optimized_filtered = optimized_df[optimized_df['path_length'] <= max_len]

save_detailed_plot(
    original_filtered['path_length'].values,
    original_filtered['curvature'].values,
    optimized_filtered['curvature'].values,
    'Detailed Path Curvature Comparison', 'Path Length', 'Curvature', 'curvature_comparison.png'
)

# 障碍物距离详细图
save_detailed_plot(
    original_filtered['path_length'].values,
    original_filtered['obstacle_distance'].values,
    optimized_filtered['obstacle_distance'].values,
    'Detailed Obstacle Distance Comparison', 'Path Length', 'Obstacle Distance', 'obstacle_distance_comparison.png'
)

# 航向角详细图
save_detailed_plot(
    original_filtered['path_length'].values,
    original_filtered['theta'].values,
    optimized_filtered['theta'].values,
    'Detailed Heading Angle Comparison', 'Path Length', 'Heading Angle (rad)', 'theta_comparison.png'
)

# 转向角度详细图
save_detailed_plot(
    original_filtered['path_length'].iloc[1:].values,
    original_filtered['steering_angle'].iloc[1:].values,
    optimized_filtered['steering_angle'].iloc[1:].values,
    'Detailed Steering Angle Comparison', 'Path Length', 'Steering Angle (rad)', 'steering_angle_comparison.png'
)

print("Detailed analysis charts have been generated.") 