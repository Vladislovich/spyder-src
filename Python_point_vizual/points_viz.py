import open3d as o3d

# Загружаем .ply файл
points = o3d.io.read_point_cloud("../points_and_meshes/smoothed_points.pcd")

# Проверка успешности загрузки
if points.is_empty():
    print("Ошибка загрузки сетки.")
else:
    # Показываем информацию о сетке
    print(points)

    # Создаем визуализатор и добавляем сетку
    o3d.visualization.draw_geometries([points], window_name="points Visualization")