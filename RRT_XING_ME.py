# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 17:05:04 2023

@author: 74626
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 16:08:55 2023

@author: 74626
"""

import matplotlib.pyplot as plt
import random
import math
import numpy as np

class Except:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.grid_area = 0.01

    def cross_grid(self, path_length, start_row, end_row, start_col, end_col,population_density):
        # 注意这里用了self.grid_map
        cross_grid_left = self.grid_map[start_row:end_row, start_col:end_col]
        true_count = np.count_nonzero(cross_grid_left)
        total_count = end_row * (end_col - start_col)
        crossable_grid = str(total_count - true_count)
        if path_length == path_length_left:
            print(f"地图左侧可通行的栅格数有 {crossable_grid} 格")
        else:
            print(f"地图右侧可通行的栅格数有 {crossable_grid} 格")
        return self.add_parameter(crossable_grid, path_length,population_density)

    def add_parameter(self, crossable_grid, robot_cross_grid , population_density):
        crossable_grid = int(crossable_grid)  # 确保是整数
        population_numble = math.ceil(population_density * crossable_grid * self.grid_area)
        
        # 输入验证
        if crossable_grid - population_numble - 1 < 0:
            raise ValueError("Invalid input for factorial")
            
        # 概率计算
        probability = 1 - math.pow((crossable_grid - 1)/crossable_grid,population_numble)
        
        extra_grid = 2*(robot_cross_grid * probability)  # 注意这里可能需要更多的检查和修正
        return extra_grid
    
    def final_path(self,path_length_left,path_length_right,except_left_grid,except_right_grid):
        if path_length_left + except_left_grid > path_length_right + except_right_grid:
            print('地图右侧的路径优于左侧')
        if path_length_left + except_left_grid < path_length_right + except_right_grid:
            print('地图左侧的路径优于右侧')
        if path_length_left + except_left_grid == path_length_right + except_right_grid:
            if except_left_grid > except_right_grid:
                print('地图右侧的路径优于左侧')
            else:
                print('地图左侧的路径优于右侧')



class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def is_inside_obstacle(x, y, obstacles):
    for (ox, oy, w, h) in obstacles:
        if ox <= x <= ox + w and oy <= y <= oy + h:
            return True
    return False

def distance(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def draw_path(goal, color, to_draw):
    node = goal
    while node.parent is not None:
        to_draw.append((node.x, node.parent.x, node.y, node.parent.y, color))
        node = node.parent

def rrt_left(start, goal, obstacles, map_dim, max_paths=1, max_iter=4000, step_size=2.0):
    paths_found = 0
    nodes = [start]
    while paths_found < max_paths and len(nodes) < max_iter:
        if paths_found % 2 == 0:  # 控制随机点在地图的左半部分
            rnd_x = random.uniform(0, (map_dim[0] / 2)+10)
        else:  # 控制随机点在地图的右半部分
            rnd_x = random.uniform(map_dim[0] / 2, map_dim[0])
        rnd_y = random.uniform(0, map_dim[1])

        if is_inside_obstacle(rnd_x, rnd_y, obstacles):
            continue

        min_dist = float('inf')
        nearest_node = None
        for node in nodes:
            d = distance(Node(rnd_x, rnd_y), node)
            if d < min_dist:
                min_dist = d
                nearest_node = node

        theta = math.atan2(rnd_y - nearest_node.y, rnd_x - nearest_node.x)
        new_x = nearest_node.x + step_size * math.cos(theta)
        new_y = nearest_node.y + step_size * math.sin(theta)

        if is_inside_obstacle(new_x, new_y, obstacles):
            continue

        new_node = Node(new_x, new_y)
        new_node.parent = nearest_node
        nodes.append(new_node)

        if distance(new_node, goal) < step_size:
            unique_path = True
            for node in nodes:
                if node.parent is not None and distance(new_node, node.parent) < step_size:
                    unique_path = False
                    break
            if unique_path:
                goal.parent = new_node
                draw_path(goal, 'C' + str(paths_found),to_draw)  # 使用不同颜色标识路径
                paths_found += 1
                
                # Calculate and print the number of grid cells passed
                path_length = 0
                current_node = goal
                while current_node.parent is not None:
                    path_length += 1
                    current_node = current_node.parent
                print(f"Path left passes through {path_length} grid cells")                    
                return path_length
                                
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r-', linewidth=0.5)
    for (ox, oy, w, h) in obstacles:
        plt.gca().add_patch(plt.Rectangle((ox, oy), w, h, color='grey', fill=True))
    
    if paths_found ==0:
        return None
    return path_length

def rrt_right(start, goal, obstacles, map_dim, max_paths=1, max_iter=4000, step_size=2.0):
    paths_found = 0
    nodes = [start]
    while paths_found < max_paths and len(nodes) < max_iter:
        if paths_found % 2 == 0:  # 控制随机点在地图的左半部分
            rnd_x = random.uniform(map_dim[0]/2,map_dim[0])
        else:  # 控制随机点在地图的右半部分
            rnd_x = random.uniform(map_dim[0] / 2, map_dim[0])
        rnd_y = random.uniform(0, map_dim[1])

        if is_inside_obstacle(rnd_x, rnd_y, obstacles):
            continue

        min_dist = float('inf')
        nearest_node = None
        for node in nodes:
            d = distance(Node(rnd_x, rnd_y), node)
            if d < min_dist:
                min_dist = d
                nearest_node = node

        theta = math.atan2(rnd_y - nearest_node.y, rnd_x - nearest_node.x)
        new_x = nearest_node.x + step_size * math.cos(theta)
        new_y = nearest_node.y + step_size * math.sin(theta)

        if is_inside_obstacle(new_x, new_y, obstacles):
            continue

        new_node = Node(new_x, new_y)
        new_node.parent = nearest_node
        nodes.append(new_node)

        if distance(new_node, goal) < step_size:
            unique_path = True
            for node in nodes:
                if node.parent is not None and distance(new_node, node.parent) < step_size:
                    unique_path = False
                    break
            if unique_path:
                goal.parent = new_node
                draw_path(goal, 'C' + str(paths_found),to_draw)  # 使用不同颜色标识路径
                paths_found += 1
                
                # Calculate and print the number of grid cells passed
                path_length = 0
                current_node = goal
                while current_node.parent is not None:
                    path_length += 1
                    current_node = current_node.parent
                print(f"Path right passes through {path_length} grid cells")
                
            
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r-', linewidth=0.5)
    for (ox, oy, w, h) in obstacles:
        plt.gca().add_patch(plt.Rectangle((ox, oy), w, h, color='grey', fill=True))

    if paths_found ==0:
        return None
    return path_length
        
if __name__ == "__main__":
    start = Node(50, 0)
    goal = Node(60, 90)
    to_draw = []
    
    obstacles = [(0,0,1,100),(0,0,100,1),(0,100,100,1),(100,0,1,100),
                 (40, 20, 20, 60), (15, 20, 25, 10),(60,20,20,10),(0,60,30,10)]
    map_dim = (100, 100)
    grid_map = np.zeros((map_dim[0], map_dim[1]))
    for obstacle in obstacles[0:2]:
        x, y, width, height = obstacle
        grid_map[y:y+height, x:x+width] = 1
    for obstacle in obstacles[2:3]:
        x, y, width, height = obstacle
        grid_map[y-1:y+height, x:x+width] = 1
    for obstacle in obstacles[3:4]:
        x, y, width, height = obstacle
        grid_map[y:y+height, x-1:x+width] = 1
    
    for obstacle in obstacles[4:8]:
        x, y, width, height = obstacle
        grid_map[y:(y+height), x:(x+width)] = 1
    
    while True:
        to_draw.clear()
        path_length_left  =  rrt_left(start, goal, obstacles, map_dim)
        path_length_right =  rrt_right(start, goal, obstacles, map_dim)
        if path_length_left is not None and path_length_right is not None:
            break
        print('没有找到合适路径，重新运行')
    # 创建Except类的实例
    except_instance = Except(grid_map)
    # 使用实例方法
    except_left_grid = except_instance.cross_grid(path_length_left, start_row=0, end_row=100, start_col=0, end_col=50,population_density=0.2)
    except_right_grid = except_instance.cross_grid(path_length_right, start_row=0, end_row=100, start_col=50, end_col=100,population_density=80)
    for x1, x2, y1, y2, color in to_draw:
        plt.plot([x1, x2], [y1, y2], color + '-', linewidth=4.0)
    print(f"地图左侧需额外考虑通行的栅格数有 {except_left_grid} 格")
    print(f"地图右侧需额外考虑通行的栅格数有 {except_right_grid} 格")
    except_instance.final_path(except_left_grid,except_right_grid,path_length_left,path_length_right)
    plt.plot(start.x, start.y, 'go')
    plt.plot(goal.x, goal.y, 'ro')
    plt.grid(True)
    plt.show()
    