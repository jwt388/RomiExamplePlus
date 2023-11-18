// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveBox;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.TankDrive;
import frc.robot.commands.DriveArcDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.OUTPUT, ChannelMode.OUTPUT); // Enable LEDs

  // Assumes a gamepad plugged into channel 0
  //private final Joystick m_controller = new Joystick(0);
  private final XboxController m_controller = new XboxController(0); 
  //private final CommandXboxController m_controller = new CommandXboxController(0); 

  // Create SmartDashboard choosers for autonomous routines and drive mode
  private final SendableChooser<Command> m_chooserAuto = new SendableChooser<>();
  private final SendableChooser<String> m_chooserDrive = new SendableChooser<>();

  // Slew rate limiters for joystick inputs
  private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(10);

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_onboardIO.setGreenLed(false);
    m_onboardIO.setRedLed(false);
    
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(m_drivetrain);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Setup chooser for selecting drive mode
    m_chooserDrive.setDefaultOption("Drive Mode - Arcade", "arcade");
    m_chooserDrive.addOption("Drive Mode - Tank", "tank");
    m_chooserDrive.addOption("Drive Mode - Curvature", "curve");
    SmartDashboard.putData(m_chooserDrive);

    // Default command is manual drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getDriveCommand());
    m_drivetrain.setMaxOutput( 1.0);

    // Drive at half speed when the right bumper is held
    new JoystickButton(m_controller, Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> m_drivetrain.setMaxOutput(0.5)))
      .onFalse(new InstantCommand(() -> m_drivetrain.setMaxOutput( 1.0)));

    // Based on gyrodrivecommands example

    // Stabilize the robot to drive straight when left bumper is held

    new JoystickButton(m_controller, Button.kLeftBumper.value)
      .whileTrue(
        new PIDCommand(
          new PIDController(Constants.kStabilizationP, Constants.kStabilizationI, Constants.kStabilizationD), 
          // Close the loop on turn rate
          m_drivetrain::getGyroRateZ,
          // Set point is 0 deg/sec
          0,
          // Pipe the output to the turning control
          output ->  m_drivetrain.arcadeDrive(-m_controller.getRawAxis(1), output, false),
          // Drivetrain is required
          m_drivetrain));

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(m_controller, Button.kX.value)
        .onTrue(new TurnToAngle(90, m_drivetrain).withTimeout(5));
        
    // Turn to -90 degrees when the 'B' button is pressed, with a 5 second timeout
    new JoystickButton(m_controller, Button.kB.value)
        .onTrue(new TurnToAngle(-90, m_drivetrain).withTimeout(5));

    /* Example of how to use the Romi onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Romi Button A Pressed"))
        .onFalse(new PrintCommand("Romi Button A Released"));
    */

    m_onboardIO.setRedLed(false);
    m_onboardIO.setGreenLed(false);
    //m_onboardIO.setYellowLed(false);

    // Setup triggers for controller buttons
    Trigger aButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
    //Trigger xButton = m_controller.x(); // For CommandXboxController

    // Reset gyro and odometry when 'A' button of the contoroller is pressed
    aButton
    .onTrue(new ResetOdometry(m_drivetrain));
      // .onTrue(new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(Constants.startX, Constants.startY, new Rotation2d())) ));

    // Setup SmartDashboard options
    m_chooserAuto.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooserAuto.addOption("Auto Routine Turn to 0", new TurnToAngle(0, m_drivetrain));
    m_chooserAuto.addOption("Auto Routine Box", new DriveBox(m_drivetrain));
    m_chooserAuto.addOption("Auto Routine ArcDistance", new DriveArcDistance(0.5, 25.0, 0.25, m_drivetrain));
    SmartDashboard.putData(m_chooserAuto);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooserAuto.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getDriveCommand() {

    switch(m_chooserDrive.getSelected()) {

      case "tank":
        return new TankDrive(
            m_drivetrain, () -> -m_leftLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(1),0.1)),
            () -> -m_rightLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(5),0.1)));

      case "curve":
        return new CurvatureDrive(
            m_drivetrain, () -> -m_leftLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(1),0.1)),
            () -> -m_rightLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(4),0.1)));

      case "arcade":

            default:
        return new ArcadeDrive(
            m_drivetrain, () -> -m_leftLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(1),0.1)),
            () -> -m_rightLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(4),0.1)));
    }
  }

  
}
