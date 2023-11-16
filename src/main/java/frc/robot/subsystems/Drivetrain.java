// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {


  public static final double kMaxSpeed = 0.7; // meters per second
  public static final double kMaxAngularSpeed = 1.5 * 2 * Math.PI; // 1.5 rotation per second

  private static final double kTrackWidth = 5.551*0.0254; // meters (5.551 inches)
  private static final double kWheelRadius = 0.07/2; // meters (d=2.75591 inches, 70 mm)

  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

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

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  private double xVelocity;
  private double accelXOffset = 0.0;
  private double filteredAccelX;
  LinearFilter filterAccel = LinearFilter.singlePoleIIR(0.25,0.02);

  // Variables to calculate wheel velocities
  /*private double lastLeftDistanceInch;
  private double lastRightDistanceInch;
  private double newLeftDistanceInch;
  private double newRightDistanceInch;
  private double leftVelocity;
  private double rightVelocity;
  */
  //LinearFilter filterLeft = LinearFilter.movingAverage(10);
  //LinearFilter filterRight = LinearFilter.movingAverage(10);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((2 * Math.PI * kWheelRadius) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((2 * Math.PI * kWheelRadius) / kCountsPerRevolution);
    resetEncoders();

    m_diffDrive.setDeadband(0.0);
    SmartDashboard.putData(m_diffDrive);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, false);
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

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
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
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return -Math.IEEEremainder(m_gyro.getAngleZ(), 360);
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

  // Reset the accel offset and velocity estimate. Should only be called when level and not moving.
  public void resetVelocity() {
    accelXOffset = -filteredAccelX;
    xVelocity = 0.0;
  }

  @Override
  public void periodic() {

    filteredAccelX = filterAccel.calculate(getAccelX());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("X Accel", getAccelX());
    SmartDashboard.putNumber("Z Angle", getGyroAngleZ()); 
    SmartDashboard.putNumber("Z Rate", getGyroRateZ()); 

    // Update X velocity from acceleration 
    xVelocity += (getAccelX() + accelXOffset) * 0.02;
    SmartDashboard.putNumber("X Accel Filtered", filteredAccelX);

    // Update wheel velocities
    /*newLeftDistanceInch = getLeftDistanceInch();
    newRightDistanceInch = getRightDistanceInch();
    leftVelocity = filterLeft.calculate((newLeftDistanceInch - lastLeftDistanceInch) * 50); // Filter and Scale for 20 msec frame time
    rightVelocity = filterRight.calculate((newRightDistanceInch - lastRightDistanceInch) * 50);
    lastLeftDistanceInch = newLeftDistanceInch;
    lastRightDistanceInch = newRightDistanceInch;
    SmartDashboard.putNumber("Average Velocity", (leftVelocity + rightVelocity)/2);
    */

    SmartDashboard.putNumber("Left Rate", m_leftEncoder.getRate());
    SmartDashboard.putNumber("RIght Rate", m_rightEncoder.getRate());

  }

}
