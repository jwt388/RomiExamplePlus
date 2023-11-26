# RomiExamplePlus

Extension of the RomiReference example in WPILib to pull in additional features from other examples into a single program.

## Additional Features Include
- Ability to select arcade drive, tank drive or curvature drive (with or without zero turning)
- Moving deadband and squaring out of drivetrain and onto joystick inputs only
- Data logging
- Publishing test points for display on dashboards
- Creating odometry and field attributes
- Allow startup of driver station
- Trigger new commands from the controller
- More Autonomous Commands

## Controller Triggered Commands
- X: Turn to 90 degrees
- B: Turn to -90 degrees
- A: Reset odometry to field start position
- Right Bumper: drive at half speed
- Left Bumper: stabilize heading to drive straight (not yet working)

## Autonomous Commands
- Drive a box pattern
- Turn to 0 degrees using PID control from the gyro
- Drive a specified distance using PID or profiled PID control 