import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.filters import minimum_filter

# 定义三个点的坐标
points = [(-5, -5), (-5, 5), (5, 5)]

# 定义热力图的范围和分辨率
x = np.linspace(-10, 10, 100)
y = np.linspace(-10, 10, 100)
X, Y = np.meshgrid(x, y)

# 系数列表
coefficients = [(0.3, 0.3, 0.3), (0.5, 0.3, 0.2), (0.8, 0.1, 0.1)]
# coefficients = [(0.5, 0.3, 0.2)]

# 绘制三幅图
for i, coeffs in enumerate(coefficients):
    # 计算每个点的热力值
    Z = np.zeros_like(X)
    for j, point in enumerate(points):
        distance = ((X - point[0])**2 + (Y - point[1])**2)
        # Z +=distance* coeffs[j] *(distance/(0.05*pow(distance,2)+50))
        Z +=distance* coeffs[j] 


    # 绘制热力图
    plt.subplot(2, 2, i+1)
    plt.pcolormesh(X, Y, Z, cmap='rainbow')
    plt.colorbar()
    plt.title(f'Heatmap {i+1}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')

    # 标记初始定义的三个点
    for point in points:
        plt.plot(point[0], point[1], 'bo')

    # 找到热力值最小的点
    min_idx = np.unravel_index(np.argmin(Z), Z.shape)
    min_point = (X[min_idx], Y[min_idx])
    plt.plot(min_point[0], min_point[1], 'go')
    print(f"minmax: {min_point[0]},{min_point[1]}")

    # 找到极小值点
    neighborhood_size = 3
    local_minima = minimum_filter(Z, size=neighborhood_size) == Z
    plt.plot(X[local_minima], Y[local_minima], 'ro')

    local_minima_indices = np.argwhere(local_minima)
    for idx in local_minima_indices:
        x_coord = X[idx[0], idx[1]]
        y_coord = Y[idx[0], idx[1]]
        print(f"局部极小值点坐标：({x_coord}, {y_coord})")

# 调整子图布局
plt.tight_layout()
plt.show()





# # 绘制对勾函数
# import numpy as np
# import matplotlib.pyplot as plt

# # 生成 x 值的范围
# x = np.linspace(-10, 10, 100)

# # 计算对应的 y 值
# y = x / (0.5 * x**2 + 0.5)

# # 绘制图像
# plt.plot(x, y)
# plt.xlabel('x')
# plt.ylabel('f(x)')
# plt.title('Plot of f(x) = x / (0.5x^2 + 0.5)')
# plt.grid(True)
# plt.axis('equal')
# plt.show()