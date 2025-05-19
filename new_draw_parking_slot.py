# -*- coding: utf-8 -*-
"""
Created on Wed Jun  5 11:08:30 2024

@author: 74626
"""

import cv2
import os
from my_ideal import my_ideal
from scipy.spatial import KDTree
import math
def count_images_in_folder(folder_path, image_extensions=['.jpg', '.jpeg', '.png', '.bmp']):
    image_count = 0
    for filename in os.listdir(folder_path):
        if any(filename.lower().endswith(ext) for ext in image_extensions):
            image_count += 1
    return image_count

def count_center_point(parking_point):
    center_points = []
    for i in parking_point:
        x1, y1, x2, y2 = i
        center_point_x = (x2 + x1) / 2
        center_point_y = (y2 + y1) / 2
        center_point = [int(center_point_x), int(center_point_y)]
        center_points.append(center_point)
    return center_points

def remove_duplicate_pairs(pairs):
    unique_pairs = set()
    result = []
    
    # 遍历每个组合对
    for i in range(0, len(pairs), 2):
        # 确保每次处理两个元素对
        if i+1 < len(pairs):
            pair1 = pairs[i]
            pair2 = pairs[i+1]
            # 使用frozenset将两个对合并成一个无序组合
            pair_set = frozenset((tuple(pair1), tuple(pair2)))
            # 如果这个组合没有被添加过，则添加
            if pair_set not in unique_pairs:
                unique_pairs.add(pair_set)
                result.extend([pair1, pair2])
        else:
            result = pairs
    return result
        
def draw_center_point_area(folder_path,parking_point):
    # 图像处理
    image = cv2.imread(file_path)
    image_areas = []
    for i in parking_point:
        x1, y1, x2, y2 = i
        image_area = image[y1:y2, x1:x2]
        image_area = cv2.resize(image_area, (80,80))
        image_area = cv2.GaussianBlur(image_area, (3, 3), 0)
        image_area = cv2.cvtColor(image_area, cv2.COLOR_BGR2GRAY)
        _,image_area = cv2.threshold(image_area, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        cv2.circle(image_area, (40,40), radius=8, color=(0,0,255))
        image_areas.append(image_area)
    return image_areas

def count_normal_vector(unique_pairs):
    if len(unique_pairs) >= 2:
        for i in range(0, len(unique_pairs), 2):
            x1, y1 = unique_pairs[i]
            x2, y2 = unique_pairs[i+1]
            line_distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
            vector = [math.abs((x1-x2)/line_distance),math.abs((y1-y2)/line_distance)]


folder_path = 'E:/knowledge/opencv_text/yolo/yolov5-master/data/images'
num_images = count_images_in_folder(folder_path)
picture_name = [item for item in my_ideal if isinstance(item, str)]

for i in picture_name:
    # 构建图像文件路径
    file_path = os.path.join(folder_path, i)
    pair_point = []
    # 提取所需要的各项数据
    picture_name_index = my_ideal.index(i)
    parking_point_num = my_ideal[picture_name_index + 1]
    parking_point = my_ideal[picture_name_index + 2 : picture_name_index + 2 + parking_point_num]
    center_points = count_center_point(parking_point)
    # 图像处理
    image = cv2.imread(file_path)
    image = cv2.GaussianBlur(image, (3, 3), 0)
    image_areas = draw_center_point_area(file_path,parking_point)
    tree = KDTree(center_points)
    
    for j in range(len(center_points)):
        cv2.circle(image, tuple(center_points[j]), 8, (0,255,0), -1)
        query_point = center_points[j]
        # 查找最近的三个邻居
        distances, indices = tree.query(query_point, k=3)
        pair_index = [index for index, distance in enumerate(distances) if (125 <= distance <= 190  or 310 <= distance <= 390) and index != 0]
        if pair_index:
            my_pair_point = center_points[indices[pair_index[0]]]
            cv2.line(image, tuple(center_points[j]), tuple(my_pair_point), (0,0,255), 2)
            pair_point.append(center_points[j])
            pair_point.append(my_pair_point)
    unique_pairs = remove_duplicate_pairs(pair_point)
    
    print(unique_pairs)
    print("**********************************************")
