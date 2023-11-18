// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.commands.ResetOdometry;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
/* import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; */
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Adapted from differentialdrivebot example

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;
  private final Field2d m_field = new Field2d();
  
  public static final double kS = 0.38069; // from Daltz333 example
  public static final double kV = 9.5975; //

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV);


  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  NetworkTableEntry m_headingEntry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    m_gyro.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);


    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
    resetEncoders();

    m_diffDrive.setDeadband(0.0);

    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    setupShuffleboard();

  }

    /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    /*final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond); */

    final double leftOutput =
        m_leftPIDController.calculate(0.1, speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(0.1, speeds.rightMetersPerSecond);

    m_leftMotor.setVoltage(leftOutput + leftFeedforward);
    m_rightMotor.setVoltage(rightOutput + rightFeedforward);

    SmartDashboard.putNumber("Left PID output", leftOutput);
    SmartDashboard.putNumber("Right PID output", rightOutput);

  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /** Reset gyro and odometry to zero. */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    m_odometry.resetPosition(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate, boolean squareInputs) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
  } 
  public void tankDrive(double leftSpeed, double rightRotate) {
    m_diffDrive.tankDrive(leftSpeed, rightRotate, true);
  } 

  public void curvatureDrive(double xaxisSpeed, double zaxisRotate, boolean allowTurnInPlace) {
    m_diffDrive.curvatureDrive(xaxisSpeed, zaxisRotate, allowTurnInPlace);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis (wraps at +/- 180).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return -Math.IEEEremainder(m_gyro.getAngleZ(), 360);
  }

  /**
   * Current angle of the Romi around the Z-axis (continuous at +/- 180).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  /**
   * Rate of turn in degrees-per-second around the Z-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getGyroRateZ() {
    return -m_gyro.getRateZ();
  }

  // Reset the gyro. 
  public void resetGyro() {
    m_gyro.reset();
  }

  private void setupShuffleboard() {
    // Create a tab for the Drivetrain
/*     ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drivetrain");
    m_driveTab
        .add("Differential Drive", m_diffDrive); */

    SmartDashboard.putData(m_diffDrive);

    // Create a tab for the Odometry and Field
/*     ShuffleboardTab m_fieldTab = Shuffleboard.getTab("Field");
    m_fieldTab
        .add("Field", m_field); */
    SmartDashboard.putData(m_field);

/*     ShuffleboardLayout commands = m_fieldTab.getLayout("Commands",BuiltInLayouts.kList);
    commands.add(new InstantCommand (() ->  resetOdometry())); */
    // SmartDashboard.putData(new InstantCommand (() ->  resetOdometry(new Pose2d(Constants.startX, Constants.startY, new Rotation2d())))); 
    SmartDashboard.putData(new ResetOdometry(this));

  } 

  public void updateShuffleboard() {
    SmartDashboard.putNumber("Z Angle", getGyroAngleZ()); 
    SmartDashboard.putNumber("Z Rate", getGyroRateZ()); 

    SmartDashboard.putNumber("Pose Deg", m_pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Pose X", m_pose.getX());
    SmartDashboard.putNumber("Pose Y", m_pose.getY());

    SmartDashboard.putNumber("Left Rate", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Rate", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Speed", (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_pose = m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    updateShuffleboard();

  }

}
