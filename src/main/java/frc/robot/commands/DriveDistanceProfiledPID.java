// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceProfiledPID extends ProfiledPIDCommand {

  ProfiledPIDController m_controller;

  /**
   * Drives robot the specified distance using profiled PID feedback control.
   *
   * @param targetDistance The distance to drive in meters
   * @param drivetrain The drive subsystem to use
   */
  /** Creates a new DriveDistanceProfiledPID. */
  public DriveDistanceProfiledPID(double targetDistance, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.kPDriveProfiled,
            Constants.kIDriveProfiled,
            Constants.kDDriveProfiled,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.kMaxSpeedMetersPerSecond, 
                                            Constants.kMaxAccelMetersPerSecondSquared)),        
            // This should return the measurement
            () -> drivetrain.getAverageDistanceMeters(),
            // This should return the goal
            () -> new TrapezoidProfile.State(targetDistance,0),
            // Use the calculated velocity at each setpoint
            (output, setpoint) -> {
              drivetrain.arcadeDrive(output, 0, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Configure additional PID options by calling `getController` here.
    m_controller = getController();
    m_controller.setTolerance(0.05, 0.06);
  }

  @Override
  public void initialize() {
    super.initialize();

    // Override PID parameters from preferences. Preferences are initialized in the drivetrain.
    if (Constants.enableProfilePIDTune) {

      m_controller.setP(
          Preferences.getDouble(Constants.kPProfiledKey, Constants.kPDriveProfiled));
      m_controller.setI(
          Preferences.getDouble(Constants.kIProfiledKey, Constants.kIDriveProfiled));
      m_controller.setD(
          Preferences.getDouble(Constants.kDProfiledKey, Constants.kDDriveProfiled));

      // Read Preferences for Trapezoid Profile and update
      double vMax = Preferences.getDouble(Constants.kVMaxProfiledKey, Constants.kMaxSpeedMetersPerSecond);
      double aMax = Preferences.getDouble(Constants.kAMaxProfiledKey, Constants.kMaxAccelMetersPerSecondSquared);

      m_controller.setConstraints(new TrapezoidProfile.Constraints(vMax, aMax));

    }

  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
