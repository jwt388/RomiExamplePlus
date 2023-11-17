// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// Based on gyrodrivecommands example

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  private final Drivetrain m_drivetrain;


  public TurnToAngle(double targetAngleDegrees, Drivetrain drive) {
    super(
        new PIDController(Constants.kTurnP, Constants.kTurnI, Constants.kTurnD),
        // Close loop on heading
        drive::getGyroAngleZ,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0, output, false),
        // Require the drive
        drive);

    m_drivetrain = drive;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.kTurnToleranceDeg, Constants.kTurnRateToleranceDegPerS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.arcadeDrive(0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drivetrain.arcadeDrive(0, 0, false);
    }
}