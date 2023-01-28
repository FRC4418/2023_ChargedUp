// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.commands.tippedBackwards;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private static final PoseStrategy PoseStrategy;
  

  private static AprilTagFieldLayout aprilTagFieldLayout;

  private Command m_autonomousCommand;
  
  private int stallCounter = 0;
  public static AHRS ahrs = new AHRS();

  private RobotContainer m_robotContainer;

  public static PhotonPoseEstimator m_poseEstimator;

  public static PoseStrategy poseStrategy;

  public static PhotonCamera camera;

  public static DifferentialDrivePoseEstimator estimator;
  //Vision Stuff
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    ahrs.calibrate();
    System.out.println("NAVX STARTEDNAVX STARTEDNAVX STARTEDNAVX STARTEDNAVX STARTEDNAVX STARTED");

    try{
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      } catch(IOException exception){
    
      }

      camera = new PhotonCamera("USB_Camera");
  
      Transform3d cameraToDrivetrain = new Transform3d(new Translation3d(0.56,0.25,0), new Rotation3d(0,0,0));
    
      m_poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy.AVERAGE_BEST_TARGETS, camera, cameraToDrivetrain);

      // estimator = new DifferentialDrivePoseEstimator(ahrs.getAngle(), m_robotContainer.driveTrain.getLeftEncoder(), m_robotContainer.driveTrain.getRightEncoder(), new Pose2d(), 
      // VecBuilder.fill(0.05,0.05,Units.degreesToRadians(5),0.01,0.01),
      // VecBuilder.fill(0.02,0.02, Units.degreesToRadians(1)),
      // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)))
      // );
      estimator = new DifferentialDrivePoseEstimator(
        m_robotContainer.driveTrain.kinematics, 
        ahrs.getRotation2d(), 
        m_robotContainer.driveTrain.getLeftEncoder().getDistance(), 
        m_robotContainer.driveTrain.getRightEncoder().getDistance(), 
        m_robotContainer.driveTrain.getPose()
        );
    }
      
  

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    var result = camera.getLatestResult();
    boolean hasTarget = result.hasTargets();
    //PhotonTrackedTarget target = result.getBestTarget();


    if(hasTarget){
    var imageCaptureTime = result.getTimestampSeconds();
    var camToTargetTranslation = result.getBestTarget().getBestCameraToTarget();
    var camPose =  Constants.kFarTargetPose.transformBy(camToTargetTranslation.inverse());
    //m_poseEstimator.
    estimator.addVisionMeasurement(camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
    
    estimator.update(ahrs.getRotation2d(), m_robotContainer.driveTrain.getLeftEncoder().getDistance(), m_robotContainer.driveTrain.getRightEncoder().getDistance());

    double measuredDistanceX = estimator.getEstimatedPosition().getX();
    double adjustedDistanceX = estimator.getEstimatedPosition().getX() + 0.24*estimator.getEstimatedPosition().getX()-0.1114;

    double measuredDistanceY = estimator.getEstimatedPosition().getY();
    double adjustedDistanceY = estimator.getEstimatedPosition().getY() + 0.24*estimator.getEstimatedPosition().getX()-0.1114;



    SmartDashboard.putNumber("Pose X Value", adjustedDistanceX);
    SmartDashboard.putNumber("Pose Y Value", adjustedDistanceY);

    SmartDashboard.putString("ToString of Translation", estimator.getEstimatedPosition().toString());

    SmartDashboard.putString("ToString of Current Pose", m_robotContainer.driveTrain.getPose().toString());
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      //ahrs.calibrate();
      
    }
    

    


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var result = camera.getLatestResult();
    boolean hasTarget = result.hasTargets();
    //PhotonTrackedTarget target = result.getBestTarget();


    if(hasTarget){
    var imageCaptureTime = result.getTimestampSeconds();
    var camToTargetTranslation = result.getBestTarget().getBestCameraToTarget();
    var camPose =  Constants.kFarTargetPose.transformBy(camToTargetTranslation.inverse());
    //m_poseEstimator.
    estimator.addVisionMeasurement(camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
    
    estimator.update(ahrs.getRotation2d(), m_robotContainer.driveTrain.getLeftEncoder().getDistance(), m_robotContainer.driveTrain.getRightEncoder().getDistance());

    double measuredDistanceX = estimator.getEstimatedPosition().getX();
    double adjustedDistanceX = estimator.getEstimatedPosition().getX() + 0.24*estimator.getEstimatedPosition().getX()-0.1114;

    double measuredDistanceY = estimator.getEstimatedPosition().getY();
    double adjustedDistanceY = estimator.getEstimatedPosition().getY() + 0.24*estimator.getEstimatedPosition().getX()-0.1114;



    SmartDashboard.putNumber("Pose X Value", adjustedDistanceX);
    SmartDashboard.putNumber("Pose Y Value", adjustedDistanceY);

    SmartDashboard.putString("ToString of Translation", estimator.getEstimatedPosition().toString());

    SmartDashboard.putString("ToString of Current Pose", m_robotContainer.driveTrain.getPose().toString());
    
      //double targetYaw  = target.getYaw();

    // double range = PhotonUtils.calculateDistanceToTargetMeters(0.59, 0.51, Units.degreesToRadians(0), 0);

    // SmartDashboard.putBoolean("Has Target", hasTarget);
    // SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
    // SmartDashboard.putNumber("Target Yaw", targetYaw);
    // SmartDashboard.putNumber("Inverse Yaw", -targetYaw);

    // SmartDashboard.putNumber("Default Distance to target", range);
    // double adjustedRange = 0.24*range-0.0114;
    // SmartDashboard.putNumber("Adjusted Distance", range + adjustedRange);


    }

}



  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
