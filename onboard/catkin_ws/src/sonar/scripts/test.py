
BAUD_RATE = 115200  # hz
SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
SPEED_OF_SOUND_IN_WATER = 1480  # m/s

import numpy as np
databruh = np.ones(200)
databruh[156]= 3
class Sonar:
    def __init__(self):
        self.sample_period = 1

    def get_biggest_byte(self):
        """Get the biggest value of the byte array.

        Returns:
            int: index of the biggest value.
        """

        import numpy as np

        data = databruh
        split_bytes = [data[i:i+1] for i in range(len(data))]
        bruhbytes = split_bytes[100:]
        best = np.argmax(bruhbytes)
        print(best)
        print(
            f"{split_bytes[best+100]} {best+100} {self.get_distance_of_sample(best+100)}")
        return np.argmax(split_bytes)

    def get_distance_of_sample(self, sample_index):
        """Get the distance in meters of a sample given its index in the data array returned from the device.

        Computes distance using formula from https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/

        Args:
            sample_index (int): Index of the sample in the data array, from 0 to N-1, where N = number of samples.

        Returns:
            float: Distance in meters of the sample from the sonar device.
        """
        sample_number = sample_index + 1
        distance = SPEED_OF_SOUND_IN_WATER * ((self.sample_period * SAMPLE_PERIOD_TICK_DURATION) * sample_number) / 2.0
        return distance
sonar = Sonar()
sonar.get_biggest_byte()




