# robosoccer_firmware
Sensor interfacing: Reads IMU, force sensors, encoders

### Publishes
* `JointState`: `/joint_states`: Robot limb joint states
* `Float32`: `/imu/data`: IMU reading from robot
* `Float32`: `/imu/euler_ori`: IMU Euler Orientation from robot
* `Float32`: `/left/force`: Left foot force reading
* `Float32`: `/right/force`: Right foot force reading

### Subscribes to
* `JointState`: `/joint_cmd`: Robot limb joint command

