// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveBox extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn right, drive, and repeat to follow 4 sides of a box.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public DriveBox(Drivetrain drivetrain) {
    addCommands(
      new DriveDistance(-0.5, 0.5, drivetrain),
      new TurnDegrees(0.5, 90, drivetrain),
      new DriveDistance(-0.5, 0.5, drivetrain),
      new TurnDegrees(0.5, 90, drivetrain),
      new DriveDistance(-0.5, 0.5, drivetrain),
      new TurnDegrees(0.5, 90, drivetrain),
      new DriveDistance(-0.5, 0.5, drivetrain),
      new TurnDegrees(0.5, 90, drivetrain));
  }
}
