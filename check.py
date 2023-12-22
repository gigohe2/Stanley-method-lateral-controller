import csv
import matplotlib.pyplot as plt
import time
import numpy as np
# CSV 파일에서 UTM 좌표 데이터 읽기
x_values = []
y_values = []

with open('utm-k_path_2gong.csv', 'r') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # 첫 번째 행은 헤더이므로 건너뜁니다.
    for row in reader:
        x_values.append(float(row[0]))
        y_values.append(float(row[1]))


def Calc_slopeofpath(minimum_idx):
    idx_1 = minimum_idx
    idx_2 = minimum_idx + 1
    
    x_1, y_1 = x_values[idx_1], y_values[idx_1]
    x_2, y_2 = x_values[idx_2], y_values[idx_2]

    dx = x_1 - x_2
    dy = y_1 - y_2

    slope = np.arctan2(dx, dy)
    perp = slope + np.pi /2
    x, y = x_1 + np.cos(perp), y_1 + np.sin(perp)
    cte = np.dot([x,y], [np.cos(perp), np.sin(perp)])
    cross_track_steering = np.arctan(cte / 2)
    print(slope, perp, cross_track_steering)

plt.figure(figsize=(8, 6))
plt.plot(x_values, y_values, linestyle='-') 
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)


plt.ion()
plt.show()


for idx in range(1000,len(x_values)):
    plt.plot(x_values[idx], y_values[idx], 'ro')  
    Calc_slopeofpath(idx)
    plt.pause(1e-3)


plt.ioff()


plt.show()
