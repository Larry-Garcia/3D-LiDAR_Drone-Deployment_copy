system_launch.py
Starts/stops the nodes and passes parameters.
This is where enable_2d_motion_detector, enable_2d_pan_tracker, and enable_servo_motor_node are set.

object_motion_detector_2d.py
This is the main 2D logic now. It does:

reads /scan
detects motion
computes target angle
computes pan command (clamped -90..90)
publishes /servo_angles
publishes marker + debug topics
servo_motor_node.py
Consumes /servo_angles and sends serial commands to the actual pan/tilt controller.

rplidar_node (from rplidar_ros package)
Publishes /scan with frame laser_frame (driver node, not your custom script).

Not used for tracking right now:

object_pan_tracker_2d.py
Legacy separate tracker file, currently not launched.
Optional and only used if enabled:

scan_sequence_node.cpp
Publishes sweep commands; if enabled, it can fight your tracker because both write /servo_angles.
