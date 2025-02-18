import open3d as o3d
import numpy as np
import os
import colorsys

class points:
    def __init__(self):
        pass

def create_color_from_intensity(intensity, min_intensity=0, max_intensity=300):
    normalized_intensity = (intensity - min_intensity) / (max_intensity - min_intensity)
    rgb = colorsys.hsv_to_rgb(normalized_intensity, 1, 1)
    #rgb = colorsys.hsv_to_rgb(0, 0, 0.23)
    return rgb #записывает цвет точки по ее интенсивности

def read_data_from_file_with_parameters(filename):
    points.data = [] 
    points.intens = []
    with open(filename, 'r') as file:
        for line in file:
            values = list(map(float, line.strip().split()))
            if len(values) >= 4 and values[3] > 30 and values[0] > 0 and values[1] > 0:  # Проверяем, что есть хотя бы 4 значения с интенсивностью >30
                points.data.append(values[:3])  # Берем только первые три значения (x, y, z)
                points.intens.append(values[3]) # Берем только третье значение интенсивности


def smooth_points(pcd, k = 10): #k - количество соседей использованное для сглаживания поверхности
    points = np.asarray(pcd.points)
    smoothed_points = np.zeros_like(points)
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    for i in range(len(points)):
        [_, idx, _] = kdtree.search_knn_vector_3d(points[i], k)
        smoothed_points[i] = np.mean(points[idx], axis=0)

    pcd.points = o3d.utility.Vector3dVector(smoothed_points)


def create_mesh(pcd):
    # 1. Нормализация облака точек (если еще не сделано) создание нормалей к точкам 
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=10)) #0.1 30 #радиус для взятия нормали и max количество точек для взятия 
    pcd.normals = o3d.utility.Vector3dVector(-np.asarray(pcd.normals)) #направить норамли в другую сторону
    # 2. Построение поверхности с использованием алгоритма Триангуляции Делоне
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=15)

    # 3. Удаление малозначительных треугольников (если необходимо)
    #vertices_to_remove = densities < np.quantile(densities, 0.5)
    #mesh.remove_vertices_by_mask(vertices_to_remove)
    
    return mesh


def main ():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(current_dir+"/logs", 'pointcloud_log_3.txt')
    read_data_from_file_with_parameters(filename)

    colors = []
    for intensity in points.intens:
        colors.append(create_color_from_intensity(intensity))
    
    pcd = o3d.geometry.PointCloud() #создание массива точек
    pcd.points = o3d.utility.Vector3dVector(points.data)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    
    smooth_points(pcd)
    pcd = pcd.voxel_down_sample(voxel_size = 0.02) #размер вокселя для сбора точек в нем и приведение всех к среднему
    mesh = create_mesh(pcd)
    
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([mesh])

if __name__ == "__main__":
    main()