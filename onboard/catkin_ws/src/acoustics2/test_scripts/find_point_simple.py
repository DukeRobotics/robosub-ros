import numpy as np
import pandas as pd
from scipy.optimize import differential_evolution, fsolve
import time
import csv
from scipy.optimize import least_squares

D = 1       # Side Distance
v = 1500.0  # Speed of sound in water

        # H1   H2   H3
htimes = [0.0, 0.0, 0.0] # Global array for times

# H3
# |             
# |             
# |            
# |             
# H1 --------- H2
# Side distance is D

# Generate lookup table from file
data = pd.read_csv('onboard/catkin_ws/src/acoustics2/fake_data/lookup_table.csv2')
data['X'] = np.round(data['X']).astype(int)
data['Y'] = np.round(data['Y']).astype(int)
lookup_table = data.groupby(['X', 'Y'])[['t1', 't2', 't3']].mean().reset_index()


# Function to get the initial guess from the lookup table
def get_initial_guess(t1, t2, t3, lookup_table):
    lookup_table['diff'] = np.abs(lookup_table['t1'] - t1) + np.abs(lookup_table['t2'] - t2) + np.abs(lookup_table['t3'] - t3)
    initial_guess_row = lookup_table.loc[lookup_table['diff'].idxmin()]
    return initial_guess_row['X'], initial_guess_row['Y']


def TDOA_equation(vars):
    # Uses t1 as base hydrophone (https://www.desmos.com/calculator/lb4yhnluzj)
    x, y = vars
    return np.array([
        np.sqrt(x**2 + y**2) - np.sqrt((x-D)**2 + y**2) - v*(htimes[0]-htimes[1]),
        np.sqrt(x**2 + y**2) - np.sqrt(x**2 + (y-D)**2) - v*(htimes[0]-htimes[2])
    ])


with open('onboard/catkin_ws/src/acoustics2/fake_data/final_calc_square4.csv2', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    
    with open('onboard/catkin_ws/src/acoustics2/fake_data/lookup_table.csv2', 'r') as timing_points:
        csv_reader = csv.reader(timing_points)
        next(csv_reader)
        for row in csv_reader:

            Rx, Ry, t1, t2, t3 = map(float, row)
            start_time = time.time()

            htimes = [t1, t2, t3]

            D1 = v*(t1-t2)
            D2 = v*(t1-t3)

            initial_guess = get_initial_guess(t1, t2, t3, lookup_table)

            res = least_squares(TDOA_equation, 
                                initial_guess, 
                                bounds=([-30, -30], [30, 30]), 
                                method='trf', 
                                ftol=1e-10, 
                                xtol=1e-10, 
                                gtol=1e-10)
            
            if not res.success:
                print(f"Optimization failed for Rx={Rx}, Ry={Ry}. Message: {res.message}")

            x,y = res.x[0], res.x[1]
            delta_time = time.time() - start_time   

            csv_writer.writerow([Rx, Ry, round(x,2), round(y,2), round(delta_time,3)])

