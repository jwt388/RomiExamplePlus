// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// Based on FRC 2928 training example

public class DriveDistancePID extends PIDCommand {
  /**
   * Drives robot the specified distance using profiled PID feedback control.
   *
   * @param targetDistance The distance to drive in meters
   * @param drivetrain The drive subsystem to use
   */
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable m_table;
  
  /** Creates a new DriveDistanceProfiledPID. */
  public DriveDistancePID(double targetDistance, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kPDrivePID,
                          Constants.kIDrivePID,
                          Constants.kDDrivePID),
        // This should return the measurement
        () -> drivetrain.getAverageDistanceMeters(),
        // drivetrain::getAverageDistanceMeters,
        // This should return the setpoint (can also be a constant)
        () -> targetDistance,
        // This uses the output
            output -> {
              drivetrain.arcadeDrive(output, 0, false);
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.05, 0.06);
  }

  public void initialize() {
    super.initialize();

    // Override PID parameters from Shuffleboard
    if (Constants.enableDistanceTune) {
      m_table = inst.getTable("Shuffleboard/PID Tuning");
      getController().setP(m_table.getEntry("kP-Dist").getDouble(Constants.kPDrivePID));
      getController().setI(m_table.getEntry("kI-Dist").getDouble(Constants.kIDrivePID));
      getController().setD(m_table.getEntry("kD-Dist").getDouble(Constants.kDDrivePID));
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
