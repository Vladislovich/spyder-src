import open3d as o3d

# Загружаем .ply файл
mesh = o3d.io.read_triangle_mesh("../points_and_meshes/smoothed_mesh.ply")

# Проверка успешности загрузки
if mesh.is_empty():
    print("Ошибка загрузки сетки.")
else:
    # Показываем информацию о сетке
    print(mesh)

    # Включаем отображение вершинных нормалей и цветовых атрибутов (если есть)
    mesh.compute_vertex_normals()

    # Создаем визуализатор и добавляем сетку
    o3d.visualization.draw_geometries([mesh], window_name="smoothed_mesh",
                                      mesh_show_back_face=True)