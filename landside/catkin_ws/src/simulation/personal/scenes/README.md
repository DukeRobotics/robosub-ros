## Publishing Data from your Personal Computer
To publish data from your personal computer to the ROS in the docker container, you need to modify the `ros_publishing` method in `pub_code.lua`. You will modify the array of strings and the array of floats that it returns.

Each message that gets published corresponds to a contiguous section of each array, mapping the data in the array to its name in the tree structure of ROS messages. For example, the first 10 values in each array might correspond to one message, and the next 6 might correspond to another (the numbers 10 and 6 were chosen at random and have no special meaning). The order of these blocks does not matter (e.g. placing the 10 block before the 6 block should be identical to placing the 6 block before the 10. The order of values inside each block *shouldn't* matter, except for the first two values in each block.

After assembling your tables of strings and floats (details below), use the `tableConcat` function to append them to `outStrings` and outFloats, such that the values are published.

### outStrings
The first value in each block in your string table is the desired topic name (e.g. `/sim/pose`), and the second is the message type (e.g. `geometry_msgs/PoseStamped`). Then, each following value is the name of a lowest-level value (e.g. `pose.position.x`). This will work for single numbers.

There are two cases accounted for in this code where data is stored in a ROS message that is not a single number. The first is for headers for stamped messages. Simply treat the header as a lowest level value and place the name of the header variable (most likely header) in the string table.

The second case is for storing an array of numbers. Format your string first with the name of the value, then a colon, then the numbers separated by a single comma (e.g. `linear_acceleration_covariance:0,0,0,0,0,0,0,0,0`).

### outFloats
The first and second value in each block should be -1. Subsequent values should be the numeric value corresponding with the name placed in the string table. Again, there are two exceptions. For headers, use the constant `HEAD_FLAG`. For arrays, use the constant `ARRAY_FLAG`.
