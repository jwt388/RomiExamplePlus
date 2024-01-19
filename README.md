# RomiExamplePlus-2024

Extension of the RomiReference example in WPILib to pull in additional features from other examples into a single program.

(Converted to WPILib 2024 Beta)

## Additional Features Include
- Ability to select arcade drive, tank drive or curvature drive (with or without zero turning)
- Moving deadband and squaring out of drivetrain and onto joystick inputs only
- Data logging
- Publishing test points for display on dashboards
- Creating odometry and field attributes
- Allow startup of driver station
- Trigger new commands from the controller
- More Autonomous Commands
- Show examples of setting parameters at runtime using SmartDashboard.getNumber, Network Table getEntry and preferences. These are enabled through flags in the Constants file.

## Controller Triggered Commands
- X: Turn to 90 degrees
- B: Turn to -90 degrees
- A: Reset odometry to field start position
- Right Bumper: drive at half speed
- Left Bumper: stabilize heading to drive straight (not yet working)

## Autonomous Commands
- Drive a box pattern
- Turn to 0 degrees using PID control from the gyro (controller tuned via SmartDashboard.getNumber)
- Drive a specified distance using PID control (controller tuned via Network Table getEntry)
- Drive a specified distance using profiled PID control (controller tuned via preferences)
- Follow a trajectory with a Ramsete controller following a path generated from a list of points
- Follow a trajectory with a Ramsete controller following a path generated from PathWeaver
