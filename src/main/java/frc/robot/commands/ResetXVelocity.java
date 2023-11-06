// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetXVelocity extends CommandBase {
  private final Drivetrain m_drive;

  /**
   * Creates a new ResetXVelocity command. This command will reset the X acceleration offset and velocity.
   *
   * @param drive The drivetrain subsystem on which this command will run
   */
  public ResetXVelocity(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetVelocity();
    System.out.println("X Velocity Reset");
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (true);
  }
}
