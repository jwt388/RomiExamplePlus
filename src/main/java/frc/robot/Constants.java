// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Romi device specific constants 
    public static final  double kTrackWidth = 5.551*0.0254; // meters (5.551 inches)
    public static final  double kWheelRadius = 0.07/2; // meters (d=2.75591 inches, 70 mm)
    public static final  int kEncoderResolution = 1440;

    // Starting field position for odometry
    public static final double startX = 1.6;
    public static final double startY = 4.7;

    // Heading stabilization constants
    public static final  double kPStabilization = 0.005;
    public static final  double kIStabilization = 0.0;
    public static final  double kDStabilization = 0.0;

    // Turn to angle PID constants
    public static final double kPTurn = 0.004;
    public static final double kITurn = 0.0;
    public static final double kDTurn = 0.0;
    public static final double kTurnToleranceDeg = 1.0;
    public static final double kTurnRateToleranceDegPerS = 10.0;

    // For distances PID
    public static final double kPDrivePID = 1.2;
    public static final double kIDrivePID = 0.2;
    public static final double kDDrivePID = 0;

    // For profiled distances PID
    public static final double kPDriveProfiled = 1.2;
    public static final double kIDriveProfiled = 0.0;
    public static final double kDDriveProfiled = 0.0;

    // Max speed and acceleration of the robot
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelMetersPerSecondSquared = 1.0;

    // Debug and tuning enable/disable
    public static final boolean enableDistanceTune = true;
    public static final boolean enableAngleTune = false;

    public static final double ksVolts = 0.2;
    public static final double kvVoltSecondsPerMeter = 8.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 4.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


}
