import open3d as o3d

# Загружаем .ply файл
mesh_sm = o3d.io.read_triangle_mesh("../points_and_meshes/mesh_smooth.ply")
mesh_no_sm = o3d.io.read_triangle_mesh("../points_and_meshes/mesh_no_smooth.ply")

# Проверка успешности загрузки
if mesh_sm.is_empty():
    print("Ошибка загрузки сетки.")
else:
    # Показываем информацию о сетке
    print(mesh_sm)

    # Включаем отображение вершинных нормалей и цветовых атрибутов (если есть)
    mesh_sm.compute_vertex_normals()

    # Создаем визуализатор и добавляем сетку
    o3d.visualization.draw_geometries([mesh_sm], window_name="mesh_smooth",
                                      mesh_show_back_face=True)
    

if mesh_no_sm.is_empty():
    print("Ошибка загрузки сетки.")
else:
    # Показываем информацию о сетке
    print(mesh_no_sm)

    # Включаем отображение вершинных нормалей и цветовых атрибутов (если есть)
    mesh_no_sm.compute_vertex_normals()

    # Создаем визуализатор и добавляем сетку
    o3d.visualization.draw_geometries([mesh_no_sm], window_name="mesh_no_smooth",
                                      mesh_show_back_face=True)