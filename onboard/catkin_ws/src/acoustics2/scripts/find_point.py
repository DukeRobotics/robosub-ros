import numpy as np
from scipy.optimize import differential_evolution, fsolve
import time
import csv

D = 1       # Side Distance
v = 1500.0  # Speed of sound in water

htimes = [0.0, 0.0, 0.0]
hpts_t = [(0, -1.1547*D), (D, 0.57735*D), (-D, 0.57735*D)]
hpts =   [(0, 0), (0, D), (D, 0)]


def TDOA_equation(vars):
    # Uses t1 as base hydrophone
    x, y = vars

    return [
        np.sqrt((x-hpts[0][0])**2 + (y-hpts[0][1])**2) - np.sqrt((x-hpts[1][0])**2 + (y-hpts[1][1])**2) - v*(htimes[0]-htimes[1]),
        np.sqrt((x-hpts[0][0])**2 + (y-hpts[0][1])**2) - np.sqrt((x-hpts[2][0])**2 + (y-hpts[2][1])**2) - v*(htimes[0]-htimes[2])
    ]

def objective(vars):
    return np.sum(np.square(TDOA_equation(vars)))

def solve_position():
    bounds = [(-30, 30), (-30, 30)]
    result = differential_evolution(objective, bounds)
    if result.success:
        return fsolve(TDOA_equation, result.x)
    return None

def sqrt(x):
    return np.sqrt(x)

with open('onboard/catkin_ws/src/acoustics2/fake_data/final_calc_square2.csv2', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    
    with open('onboard/catkin_ws/src/acoustics2/fake_data/timing_points_square.csv2', 'r') as timing_points:
        csv_reader = csv.reader(timing_points)
        for row in csv_reader:
            Rx, Ry, t1, t2, t3 = map(float, row)
            start_time = time.time()

            htimes = [t1, t2, t3]

            D1 = v*(t1-t2)
            D2 = v*(t1-t3)

            elta_time = time.time() - start_time 
            #csv_writer.writerow([Rx, Ry, x, y, delta_time])

            #wait 1 sec
            time.sleep(1)
