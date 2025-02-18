import open3d as o3d

# Загружаем .ply файл
filtered_points = o3d.io.read_point_cloud("../points_and_meshes/filtered_points.pcd")
smoothed_points = o3d.io.read_point_cloud("../points_and_meshes/smoothed_points.pcd")
normals_points = o3d.io.read_point_cloud("../points_and_meshes/normals_points.pcd")
mesh = o3d.io.read_triangle_mesh("../points_and_meshes/mesh_smooth.ply")


if filtered_points.is_empty():
    print("Ошибка загрузки filtered_points")
else:
    print(filtered_points)
    o3d.visualization.draw_geometries([filtered_points], window_name="filtered_points")


if smoothed_points.is_empty():
    print("Ошибка загрузки smoothed_points")
else:
    print(smoothed_points)
    o3d.visualization.draw_geometries([smoothed_points], window_name="smoothed_points")


if mesh.is_empty():
    print("Ошибка загрузки mesh")
else:
    print(mesh)
    # Включаем отображение вершинных нормалей и цветовых атрибутов (если есть)
    mesh.compute_vertex_normals()
    # Создаем визуализатор и добавляем сетку
    o3d.visualization.draw_geometries([mesh], window_name="mesh_smooth",
                                      mesh_show_back_face=True)   