import numpy as np
import matplotlib.pyplot as plt

HYDROPHONE_LOCATIONS = np.array([[0, 0, 0], [0.6, 0, 0], [0.6, 0.6, 0], [0, 0.6, 0]]) #meters

SPEED_SOUND = 1500 #m/s
FREQ = 35_000 #Hz
NUM_SAMPLES = 100000
PACKET_SIZE = 0.005 #seconds
SIGMA = 20 #wave packet standard deviation
NOISE_LEVEL = 0.3 #must be below 1 to have a distinguishable signal 

# H0 --------- H1
# |             |
# |             |
# |             |
# |             |
# H3 --------- H2

def calc_toa(pinger_loc, hydrophone_loc):
    """
    Calculates the time of arrival of a signal from a pinger at a hydrophone
    """
    return np.linalg.norm(pinger_loc - hydrophone_loc) / SPEED_SOUND


def gen_wave_packet(max_time, center_time, freq=FREQ):
    """
    Generates a wave packet with a Gaussian envelope
    """

    t = np.linspace(0, max_time, NUM_SAMPLES)

    raw_signal = np.sin(2*np.pi*freq*t)

    # Use Gaussian envelope
    envelope = np.exp(-((t - center_time) / (PACKET_SIZE/SIGMA))**2)

    return raw_signal * envelope

def gen_noise(pure_signal):
    """
    Generates a noisy signal by adding white noise and random wave packets to mimic reflections
    """

    t = np.linspace(0, PACKET_SIZE, NUM_SAMPLES)
    #add white noise
    pure_noise = np.random.normal(0,NOISE_LEVEL,len(t))

    # Generate some random wave packets
    for i in range(5):
        center_time = np.random.uniform(PACKET_SIZE/2, PACKET_SIZE)
        pure_noise += NOISE_LEVEL*gen_wave_packet(PACKET_SIZE, center_time)
    
    return pure_signal + pure_noise 

def gen_timeseries(pinger_loc=np.array([10,10,0])):
    """
    Generates a timeseries for each hydrophone given a pinger location
    Only simulates one ping.
    Adds extraneous wave packets following the ping to simulate reflections
    Also adds white noise

    pinger_loc: np.array([x,y,z]) in meters
    """

    toas = np.array([calc_toa(pinger_loc, hydrophone_loc) for hydrophone_loc in HYDROPHONE_LOCATIONS])
    
    max_time = np.max(toas)

    t = np.linspace(0, 2*max_time, NUM_SAMPLES)

    timeseries = [gen_wave_packet(2*max_time, toa) for toa in toas]

    # Add noise
    timeseries = [gen_noise(timeseries[i]) for i in range(4)]

    return timeseries
    

def main():

    pinger_loc = np.array([10,10,0])

    timeseries = gen_timeseries(pinger_loc)

    # Plot the timeseries
    fig, ax = plt.subplots(4,1, figsize=(10,10))

    for i in range(4):
        ax[i].plot(timeseries[i])
        ax[i].set_title(f"Hydrophone {i}")
        ax[i].set_xlabel("Time (s)")
        ax[i].set_ylabel("Amplitude")

    plt.tight_layout()
    plt.show()

    # Plot the locations of the hydrophones and the pinger
    fig, ax = plt.subplots()

    ax.scatter(HYDROPHONE_LOCATIONS[:,0], HYDROPHONE_LOCATIONS[:,1], label="Hydrophones")
    ax.scatter(pinger_loc[0], pinger_loc[1], label="Pinger")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()

    plt.show()


if __name__ == '__main__':
    main()