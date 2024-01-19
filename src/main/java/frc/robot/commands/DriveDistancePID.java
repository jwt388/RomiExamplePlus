// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// Based on FRC 2928 training example

public class DriveDistancePID extends PIDCommand {

  private final double m_duration;
  private long m_startTime;

  /**
   * Drives robot the specified distance using profiled PID feedback control.
   *
   * @param targetDistance The distance to drive in meters
   * @param drivetrain The drive subsystem to use
   */
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
    getController().setTolerance(0.02, 0.01);

    // Set time limit to 10 seconds
    m_duration = 30.0d * 1000;

  }

  public void initialize() {
    super.initialize();

    m_startTime = System.currentTimeMillis();

    // Override PID parameters from Shuffleboard. Values are initialized in the drivetrain.
    if (Constants.enableDistanceTune) {

      NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Shuffleboard/Dist PID Tune");
      getController().setP(m_table.getEntry("kP-Dist").getDouble(Constants.kPDrivePID));
      getController().setI(m_table.getEntry("kI-Dist").getDouble(Constants.kIDrivePID));
      getController().setD(m_table.getEntry("kD-Dist").getDouble(Constants.kDDrivePID));

    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Change to run for specified time to allow demonstration of behavior with bad gains
    // return getController().atSetpoint();
    return (System.currentTimeMillis() - m_startTime) >= m_duration;

  }
}
