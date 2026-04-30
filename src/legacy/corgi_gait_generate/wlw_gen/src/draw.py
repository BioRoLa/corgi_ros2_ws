import numpy as np
import matplotlib.pyplot as plt

# 設定 x 的範圍，這裡選擇了兩個週期左右
period = 2 * np.pi * 1211.0
x = np.linspace(-period, period, 1000)
y = 0.149+0.02 * np.cos(x / 1211.0)

plt.figure(figsize=(8, 4))
plt.plot(x, y)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Plot of y = 0.05*cos(x/1211.0)")
plt.grid(True)
plt.show()
