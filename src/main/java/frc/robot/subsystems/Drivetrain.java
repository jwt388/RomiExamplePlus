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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.commands.ResetOdometry;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  // Used to put telemetry data onto Shuffleboard
  GenericEntry m_headingEntry, m_avgDistanceEntry, m_speedEntry;
  GenericEntry m_leftWheelSpeedsEntry, m_rightWheelSpeedsEntry;
  GenericEntry m_leftWheelPositionEntry, m_rightWheelPositionEntry;
  GenericEntry m_distanceP, m_distanceD, m_distanceI, m_distanceV, m_distanceA;
  GenericEntry m_driveProfiledP, m_driveProfiledD, m_driveProfiledI;
  GenericEntry m_angleI, m_angleP, m_angleD;

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
        getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Set starting pose
    resetOdometry(new Pose2d(Constants.startX, Constants.startY, new Rotation2d()));

    setupShuffleboard();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_pose = m_odometry.update(
        getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData(m_field);

    updateShuffleboard();

  }
    /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

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
        getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /** Reset gyro and odometry to zero. */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    m_odometry.resetPosition(
      getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate, boolean squareInputs) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, squareInputs);
  } 

  public void tankDrive(double leftSpeed, double rightRotate) {
    m_diffDrive.tankDrive(leftSpeed, rightRotate, true);
  } 

    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  public void curvatureDrive(double xaxisSpeed, double zaxisRotate, boolean allowTurnInPlace) {
    m_diffDrive.curvatureDrive(xaxisSpeed, zaxisRotate, allowTurnInPlace);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
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
    return getRotation2d().getDegrees();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-m_gyro.getAngleZ());
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

    SmartDashboard.putData(m_diffDrive);
    SmartDashboard.putData(new ResetOdometry(this));


    // Create a tab for the distance PID tuning if enabled
    if (Constants.enableDistanceTune) {
      ShuffleboardTab m_tuneTab = Shuffleboard.getTab("PID Tuning");

      m_avgDistanceEntry = m_tuneTab.add("Distance (m)", getAverageDistanceMeters())
          .withWidget(BuiltInWidgets.kGraph)      
          .withSize(4,3)
          .withPosition(5, 0)
          .getEntry();

      m_speedEntry = m_tuneTab.add("Speed (m)", (m_leftEncoder.getRate() + m_rightEncoder.getRate())/2)
          .withWidget(BuiltInWidgets.kGraph)      
          .withSize(4,3)
          .withPosition(1, 0)
          .getEntry();

      // Add PID tuning parameters 
      m_distanceP = m_tuneTab.add("kP-Dist", Constants.kPDriveProfiled)
      .withPosition(0, 0)
      .getEntry();

      m_distanceI = m_tuneTab.add("kI-Dist", Constants.kIDriveProfiled)
      .withPosition(0, 1)
      .getEntry();

      m_distanceD = m_tuneTab.add("kD-Dist", Constants.kDDriveProfiled)
      .withPosition(0, 2)
      .getEntry();

      m_distanceV = m_tuneTab.add("Vmx-Dist", Constants.kMaxSpeedMetersPerSecond)
      .withPosition(0, 3)
      .getEntry();

      m_distanceA = m_tuneTab.add("Amx-Dist", Constants.kMaxAccelMetersPerSecondSquared)
      .withPosition(0, 4)
      .getEntry();

    }

    // Create a tab for the distance PID tuning if enabled
    if (Constants.enableAngleTune) {
      ShuffleboardTab m_tuneTab = Shuffleboard.getTab("PID Tuning");

      // Add telemetry data to the tab
      m_headingEntry = m_tuneTab.add("Heading Deg.", getHeading())
          .withWidget(BuiltInWidgets.kGraph)      
          .withSize(4,3)
          .withPosition(3, 0)
          .getEntry();

      // Add PID tuning parameters 
      m_angleP = m_tuneTab.add("kP-Angle", Constants.kPTurn)
      .withPosition(0, 0)
      .getEntry();

      m_angleI = m_tuneTab.add("kI-Angle", Constants.kITurn)
      .withPosition(0, 1)
      .getEntry();

      m_angleD = m_tuneTab.add("kD-Angle", Constants.kDTurn)
      .withPosition(0, 2)
      .getEntry();
      
    }

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
    SmartDashboard.putNumber("Left Command", m_leftMotor.get());
    SmartDashboard.putNumber("Right Command", m_rightMotor.get());



    // Update tuning debug if enabled
    if (Constants.enableDistanceTune) {

      m_avgDistanceEntry.setDouble(getAverageDistanceMeters());
      m_speedEntry.setDouble((m_leftEncoder.getRate() + m_rightEncoder.getRate())/2);

    }

    if (Constants.enableAngleTune) {

      m_avgDistanceEntry.setDouble(getAverageDistanceMeters());

    }

  }

  public void showTrajectory (Trajectory trajectory) {
    m_field.getObject("traj").setTrajectory(trajectory);
  }

}
