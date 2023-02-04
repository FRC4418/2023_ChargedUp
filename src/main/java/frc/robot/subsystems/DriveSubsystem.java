// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private Pose2d previousPose;
  private Pose2d currentPose;
  // Create Pose Estimator
  // create new DifferentialDrivePoseEstimator
  public static DifferentialDrivePoseEstimator estimator;

  private TrajectoryConfig trajConfig;

  // The motors on the left side of the drive.
  final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(2);
  final WPI_TalonFX leftBackMotor = new WPI_TalonFX(3);
  MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(5);
  final WPI_TalonFX rightBackMotor = new WPI_TalonFX(6);
  MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  // kinematics
  public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.63);

  // The robot's drive
  public final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  public final AHRS ahrs = new AHRS();

  private DifferentialDriveVoltageConstraint autoVoltageConstraint;

  // The left-side drive encoder
  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);
  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);
  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // vision
  private static Vision vision;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Vision vision) {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead  
  
    
    //  m_rightMotors.setInverted(true);

    this.vision = vision;

    estimator = new DifferentialDrivePoseEstimator(
        kinematics,
        ahrs.getRotation2d(),
        getLeftEncoder().getDistance(),
        getRightEncoder().getDistance(),
        new Pose2d());

    ahrs.calibrate();

    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

    trajConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);

    m_leftMotors.setInverted(true);

    // leftFrontMotor.setSafetyEnabled(false);
    // leftBackMotor.setSafetyEnabled(false);
    // rightFrontMotor.setSafetyEnabled(false);
    // rightBackMotor.setSafetyEnabled(false);

    // leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    // leftBackMotor.setNeutralMode(NeutralMode.Coast);
    // rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    // rightBackMotor.setNeutralMode(NeutralMode.Coast);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), getHeading(), getAverageEncoderDistance());
  
    currentPose = estimator.getEstimatedPosition();
    previousPose = currentPose;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void updateOdometry() {
    estimator.update(ahrs.getRotation2d(), getLeftEncoder().getDistance(), getRightEncoder().getDistance());
    estimator.addVisionMeasurement(vision.getCameraToTarget(), Timer.getFPGATimestamp() - 0.3);
  }

  public Pose2d getPose(){
    //return estimator.getEstimatedPosition();
    //return new Pose2d();
    //return null;
    //return m_odometry.getPoseMeters();
  
    currentPose = estimator.getEstimatedPosition();
    
    if (currentPose.getRotation() == null || currentPose.getTranslation() == null){
      return previousPose;
    } else{
      previousPose = currentPose;
      return currentPose;
    }
  }

  public DifferentialDriveVoltageConstraint getAutoVoltageConstraint() {
    return autoVoltageConstraint;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public TrajectoryConfig getTrajConfig(){
    return trajConfig;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(ahrs.getRotation2d(), getHeading(), getAverageEncoderDistance(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void setVelocity(Number Velocity) {
    m_leftMotors.set((double) Velocity);
    m_rightMotors.set((double) Velocity);
  }

  public void stop() {
    m_leftMotors.set((double) 0);
    m_rightMotors.set((double) 0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return ahrs.getRate();
  }
}
