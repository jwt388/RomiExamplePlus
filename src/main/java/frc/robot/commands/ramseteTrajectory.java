// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ramseteTrajectory extends RamseteCommand {

  private static final DifferentialDriveKinematics kDriveKinematics =
    new DifferentialDriveKinematics(Constants.kTrackWidth);

  private Trajectory m_Trajectory;
  private Drivetrain m_robotDrive;

  /** Creates a new Ramsete command from a path generated manually from a list of points. */
  public ramseteTrajectory(Drivetrain robotDrive, Trajectory Trajectory) {
    super(
      Trajectory,
      robotDrive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
      kDriveKinematics,
      robotDrive::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      robotDrive::tankDriveVolts,
      robotDrive);

      m_robotDrive = robotDrive;
      m_Trajectory = Trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    super.initialize();

    // Set starting pose and show the trajectory on the field
    m_robotDrive.resetOdometry(m_Trajectory.getInitialPose());
    m_robotDrive.showTrajectory(m_Trajectory);

  }

}
