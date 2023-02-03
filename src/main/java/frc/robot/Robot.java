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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private static final PoseStrategy PoseStrategy;
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static Transform3d camToTargetTranslation;
  
  public static DifferentialDrivePoseEstimator estimator;
  
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-schedule
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

    @Override
    public void autonomousInit() {
      //TODO: MAYBE MOVE TO AUTO. PERIODIC
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      //m_autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic(){
      m_robotContainer.driveTrain.updateOdometry();
      m_robotContainer.traj.periodic();
    //TODO: PUT AUTO COMMAND HERE

    // SmartDashboard.putNumber("traj hashcode", m_autonomousCommand.hashCode());
    
      
      // SmartDashboard.putString("Translation to target", m_robotContainer.vision.getCameraToTarget().toString());
      // SmartDashboard.putString("Current Pose", DriveSubsystem.estimator.getEstimatedPosition().toString());
      // SmartDashboard.putString("PoseMeters", m_robotContainer.driveTrain.getPose().toString());

      // // Trajectory testTraj = TrajectoryGenerator.generateTrajectory(m_robotContainer.driveTrain.estimator.getEstimatedPosition(), List.of(), m_robotContainer.vision.getCameraToTarget(), m_robotContainer.driveTrain.getTrajConfig());
    
      // // SmartDashboard.putString("Trajectory toString", testTraj.toString());
      // // SmartDashboard.putNumber("Trajectory duration", testTraj.getTotalTimeSeconds());
    }


    @Override
    public void teleopInit(){

    }

    @Override
    public void teleopPeriodic(){
      SmartDashboard.putString("Translation to target", m_robotContainer.vision.getCameraToTarget().toString());
      SmartDashboard.putString("Current Pose", DriveSubsystem.estimator.getEstimatedPosition().toString());
      SmartDashboard.putString("PoseMeters", m_robotContainer.driveTrain.getPose().toString());
    }
}