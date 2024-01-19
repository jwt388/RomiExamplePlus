// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// Based on gyrodrivecommands example

/** A command that will turn the robot to the specified angle. 
 *  Demonstrates use of 
*/
public class TurnToAngle extends PIDCommand {
  /**
   * Turns to robot to the specified angle using PID feedback control.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drivetrain The drive subsystem to use
   */
  private final Drivetrain m_drivetrain;

  public TurnToAngle(double targetAngleDegrees, Drivetrain drivetrain) {
    super(
        new PIDController(Constants.kPTurn, Constants.kITurn, Constants.kDTurn),
        // Close loop on heading
        drivetrain::getGyroAngleZ,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drivetrain.arcadeDrive(0, output, false),
        // Require the drive
        drivetrain);

    m_drivetrain = drivetrain;

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

    // Override PID parameters from Shuffleboard. Values are initialized in the drivetrain.
    if (Constants.enableAngleTune) {

      getController().setP(SmartDashboard.getNumber("kP-Angle", Constants.kPTurn));
      getController().setI(SmartDashboard.getNumber("kI-Angle", Constants.kITurn));
      getController().setD(SmartDashboard.getNumber("kD-Angle", Constants.kDTurn));

    }

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