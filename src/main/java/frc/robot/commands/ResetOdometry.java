// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends InstantCommand {

  private final Drivetrain m_drive;

  public ResetOdometry(Drivetrain drive) {

    m_drive = drive;
    
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(new Pose2d(Constants.startX, Constants.startY, new Rotation2d()));
  }
}
