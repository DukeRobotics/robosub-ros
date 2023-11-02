import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from extract_ping import detect_wave_packet
from data_sim import gen_timeseries, SAMPLE_RATE
from matplotlib import pyplot as plt

# H3
# |
# |
# |
# |
# H1 --------- H2
# Side distance is D

class PingFinder:
    SIDE_DISTANCE = 1          # Side Distance
    SPEED_OF_SOUND_W = 1500.0  # Speed of sound in water
    LOWER_BOUNDS = [-30, -30]  # Lower bounds for the solver
    UPPER_BOUNDS = [30, 30]    # Upper bounds for the solver

    def __init__(self):
        data = pd.read_csv('onboard/catkin_ws/src/acoustics2/fake_data/lookup_table.csv2') # Generate lookup table
        data['X'] = np.round(data['X']).astype(int)
        data['Y'] = np.round(data['Y']).astype(int)
        self.lookup_table = data.groupby(['X', 'Y'])[['Dt1', 'Dt2']].mean().reset_index()

    def get_initial_guess(self, Dt1, Dt2):
        """
        Gets an initial guess from the given lookup table

        Args:
            Dt1 (float): time difference of hydrophone 0 and 1
            Dt2 (float): time difference of hydrophone 0 and 2

        Returns:
            tuple (float, flot): tuple of (x, y) initial guess

        """
        self.lookup_table['diff'] = np.abs(self.lookup_table['Dt1'] - Dt1) + np.abs(self.lookup_table['Dt2'] - Dt2)
        initial_guess_row = self.lookup_table.loc[self.lookup_table['diff'].idxmin()]
        return (initial_guess_row['X'], initial_guess_row['Y'])
    
    def TDOA_equation(self, vars, Dt1, Dt2):
        """
        Returns the two TDOA Calculations given by this desmos plot https://www.desmos.com/calculator/lb4yhnluzj

        Both equations are hyperbolic and use hydrophone 1 as the base hydrophone

        Args:
            Dt1 (float): time difference of hydrophone 0 and 1
            Dt2 (float): time difference of hydrophone 0 and 2

        Returns:
            np.array: array of the two TDOA equations
        """
        x, y = vars
        return np.array([
            np.sqrt(x**2 + y**2) - np.sqrt((x-self.SIDE_DISTANCE)**2 + y**2) - self.SPEED_OF_SOUND_W*Dt1,
            np.sqrt(x**2 + y**2) - np.sqrt(x**2 + (y-self.SIDE_DISTANCE)**2) - self.SPEED_OF_SOUND_W*Dt2
        ])
    
    def make_guess(self, t1, t2, t3):
        """
        Uses a numerical solving method to calculate the point from 3 given pings

        Takes the 3 times and computes 2 delta times which is then fed into a premade
        lookup table. That determines an initial guess which is used to solve the
        TDOA_equation.

        Args:
            t1 (float): time of hydrophone 0 ping
            t2 (float): time of hydrophone 1 ping
            t3 (float): time of hydrophone 2 ping

        Returns:
            tuple (float, flot): tuple of (x, y) guessed point
        """
        Dt1 = t1-t2 # delta time between hydrophone 1 and 2
        Dt2 = t1-t3 # delta time between hydrophone 1 and 3

        print(Dt1, Dt2)

        initial_guess = self.get_initial_guess(Dt1, Dt2)
        print(initial_guess)

        res = least_squares(self.TDOA_equation, 
                            initial_guess, 
                            args=(Dt1, Dt2),
                            bounds=(self.LOWER_BOUNDS, self.UPPER_BOUNDS), 
                            method='trf', 
                            ftol=1e-10, 
                            xtol=1e-10, 
                            gtol=1e-10)
        
        if not res.success:
            print(f"Error in acoustic solution")
            return (0, 0)

        return (res.x[0], res.x[1])

if __name__ == "__main__":
    timeseries = gen_timeseries(np.array([10,10,0]))
    
    peaks = []
    filtered_signals = []
    for channel_data in timeseries:
        channel_peaks, filtered_signal = detect_wave_packet(channel_data)
        peaks.append(channel_peaks)
        filtered_signals.append(filtered_signal)

    pf = PingFinder()

    #t0 = 
    #t1 = 
    #t2 = 

    print(pf.make_guess(1.29875,1.299026,1.298078))


    
