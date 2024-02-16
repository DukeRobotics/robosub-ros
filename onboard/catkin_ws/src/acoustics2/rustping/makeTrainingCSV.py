from math import atan2, pi, floor
import matplotlib.pyplot as plt
import numpy as np

# newfilelines = ['x:1_2,x:1_3,x:2_3,y']
newfilelines = ['h1_2,h1_3,h2_3,y']

xya = []

with open('measurements.csv', 'r') as file:
    lines = file.readlines()
    lines = lines[1:]
    for line in lines:
        parts = line.split(',')
        dx = float(parts[0])
        dy = float(parts[1])
        h1_2 = float(parts[2])
        h1_3 = float(parts[3])
        h2_3 = float(parts[4])
        angle = floor((atan2(dy, dx) + pi) * 6/(2*pi))
        newfilelines.append(f'{h1_2}, {h1_3}, {h2_3}, {angle}')
        xya.append([dx, dy, angle])
        
xya = np.array(xya)
xs = xya[:, 0]
ys = xya[:, 1]
angles = xya[:, 2]
print(angles.shape)
plt.scatter(xs, ys, c=angles)
plt.colorbar()

# draw 60 degree lines through origin
for i in range(6):
    plt.plot([0, 5*np.cos(i*pi/3)], [0, 5*np.sin(i*pi/3)], 'k--')

plt.show()


# with open('training.csv', 'w') as file:
#     file.write('\n'.join(newfilelines))
        
    