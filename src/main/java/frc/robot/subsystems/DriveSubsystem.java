// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(4);
  final WPI_TalonFX leftBackMotor = new WPI_TalonFX(5);
  MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(3);
  final WPI_TalonFX rightBackMotor = new WPI_TalonFX(2);
  MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();
  public final AHRS ahrs = new AHRS();
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public DifferentialDrivePoseEstimator estimator;

  public final DifferentialDriveKinematics kinematics;

  private Pose2d previousPose;
  private Pose2d currentPose;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    kinematics = new DifferentialDriveKinematics(0.63);
    
    ahrs.calibrate();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(false);
    m_leftMotors.setInverted(true);

    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  
    estimator = new DifferentialDrivePoseEstimator(
      kinematics,
      ahrs.getRotation2d(),
      getLeftEncoder().getDistance(),
      getRightEncoder().getDistance(),
      new Pose2d());

    currentPose = estimator.getEstimatedPosition();
    previousPose = currentPose;
  }

  @Override
  public void periodic() {
    //get vision results
    //PhotonTrackedTarget target = result.getBestTarget();
    // Update the odometry in the periodic block
    m_odometry.update(
        ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    //update Pose estimator
    estimator.update(ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    SmartDashboard.putString("estimator pose", estimator.getEstimatedPosition().toString());
  }

  public void tankDrive(double left, double right) {
    
      m_drive.tankDrive(left, right, false);
    }

  public void curvatureDrive(double xSpeed, double zRotation, double baseTS) {
      // Clamp all inputs to valid values
      xSpeed = SLMath.clamp(xSpeed, -1.0, 1.0);
      zRotation = SLMath.clamp(zRotation, -1.0, 1.0);
      baseTS = SLMath.clamp(baseTS, 0.0, 1.0);

      // Find the amount to slow down turning by.
      // This is proportional to the speed but has a base value
      // that it starts from (allows turning in place)
      double turnAdj = Math.max(baseTS, Math.abs(xSpeed));

      // Find the speeds of the left and right wheels
      double lSpeed = xSpeed + zRotation * turnAdj;
      double rSpeed = xSpeed - zRotation * turnAdj;

      // Find the maximum output of the wheels, so that if a wheel tries to go > 1.0
      // it will be scaled down proportionally with the other wheels.
      double scale = Math.max(1.0, Math.max(Math.abs(lSpeed), Math.abs(rSpeed)));

      lSpeed /= scale;
      rSpeed /= scale;

      // Feed the inputs to the drivetrain
      tankDrive(lSpeed, rSpeed);
  }

  public void curvatureDrive(double xSpeed, double zRotation) {
    this.curvatureDrive(xSpeed, zRotation, 0.45);
}

  public void impulseDrive(double xSpeed, double zRotation) {
      // If the speed is negative and the steering setpoint is small, then invert the
      // steering controls
      if (xSpeed < -0.05 && Math.abs(zRotation) < 0.15) {
        curvatureDrive(xSpeed, zRotation); // Inverted steering
      } else {
        curvatureDrive(xSpeed, -zRotation); // Standard steering
      }
    }

  public Pose2d getEstimatorPose(){
    periodic();
    return estimator.getEstimatedPosition();
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

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
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
   * @param leftVolts the commanded left output
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
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
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

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -ahrs.getRate();
  }
}
