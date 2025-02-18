import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.interpolate import griddata
import os

# Функция для чтения данных из файла
def read_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            # Разделяем строку на элементы и преобразуем их в числа
            values = list(map(float, line.strip().split()))
            data.append(values)
    return np.array(data)

# Чтение данных из файла
current_dir = os.path.dirname(os.path.realpath(__file__))
filename = os.path.join(current_dir+"/logs", 'pointcloud_log_2.txt')

DATA = read_data_from_file(filename)

Xs = DATA[:, 0]
Ys = DATA[:, 1]
Zs = DATA[:, 2]

# Извлечение координат X, Y, Z
Xs = DATA[:, 0]
Ys = DATA[:, 1]
Zs = DATA[:, 2]

# ======
## plot:

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

surf = ax.plot_trisurf(Xs, Ys, Zs, cmap=cm.jet, linewidth=0)
fig.colorbar(surf)

ax.xaxis.set_major_locator(MaxNLocator(5))
ax.yaxis.set_major_locator(MaxNLocator(6))
ax.zaxis.set_major_locator(MaxNLocator(5))

fig.tight_layout()

plt.show() 