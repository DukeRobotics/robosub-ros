# Acoustics

This package handles the calculations for locating an underwater acoustics pinger that is pinging at a certain frequency.

## Execution

To launch the main sampling and processing nodes, you can use the launch file provided. Note that this expects the Logic software to already be running and the saleae device connected via USB.
```
roslaunch acoustics acoustics.launch
```

Refer to the section on the acoustics wrapper structure below for the specific information to send to start processing. To start an action via a GUI (without an action client) refer to this [tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/Calling%20Action%20Server%20without%20Action%20Client).

To run the data_sim server that will generate test data, you can use the command:
```
rosrun acoustics data_server.py
```

Refer to the section on structure below for the specific information to send to start the data generation.

## Structure

This package contains 5 nodes to handle various acoustics tasks: the acoustics wrapper, the saleae node, the guess server, the processing server, and the data server.

### Acoustics Wrapper
The acoustics wrapper node provides an external interface (an action server) to allow client code to easily call all of the steps involved in acoustics processing without providing lower level information such as the sampling frequency or file paths for saved data.

The node itself provides an action server of type `custom_msgs/AcousticsWrapper.action`, which client code will use to call the action. Client code is required to provide the target frequency of the pinger (Robosub provides this depending on what course we are using), and the wrapper server will return the horizontal angle to which the pinger is located at. It will also update the client as to which step is being done via its feedback messages.

### Saleae Node

The saleae node interfaces with the saleae and tells it to sample data for a given duration and sample for a given amount. It saves the data to a file located in the data folder. It contains an action server using type `custom_msgs/Saleae.action`.

### Guessing Node

The guessing node contains code to make a guess for which octant the pinger is located in, which provides the processing node the necessary information to do its computations. The code is separated into `guess_server.py` and `cheap_cross_cor.py` to separate the ROS interface from the underlying acoustics code, to allow for the guessing code to execute without ROS. The ROS node contains an action server using type `custom_msgs/AcousticsGuess.action`.

### Processing Node

The processing node contains code to actually compute the horizontal angle using cross correlation. The code is separated into `processing_server.py` and `cross_corr_fft.py` to separate the ROS interface from the underlying acoustics code, to allow for the processing code to execute without ROS. The ROS node contains an action server using type `custom_msgs/AcousticsProcessing.action`.

### Data Simulation Node

The data simulation node contains code to simulate the saleae and generate data that represents the hydrophones at a specific point. This is used for testing the guessing and processing algorithms. The code is separated into `data_server.py` and `data_sim.py` to separate the ROS interface from the underlying acoustics code, to allow for the processing code to execute without ROS. The ROS node contains an action server using type `custom_msgs/AcousticsData.action`.

