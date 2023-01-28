// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.rangedDrive;
import frc.robot.commands.tippedBackwards;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //create a new navx object
  //public AHRS ahrs = new AHRS();

  private static final PoseStrategy PoseStrategy = null;

  //Drivetrain object
  final DriveSubsystem driveTrain = new DriveSubsystem();

  public static PhotonCamera camera = new PhotonCamera("USB_Camera");  

  private static Transform3d cameraToDrivetrain = new Transform3d();
 
  public static PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(null, PoseStrategy, camera, cameraToDrivetrain);
  //returns the navX object
  // public AHRS returnAhrs(){
  //   return ahrs;
  // }

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    //driveTrain.setDefaultCommand(new tippedBackwards(driveTrain));
    driveTrain.setDefaultCommand(new rangedDrive(driveTrain));
  }

  public void getTargetPose(){  
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    final PhotonTrackedTarget target = result.getBestTarget();
    
  }

  public void update(double leftDist, double rightDish){
    
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var result = camera.getLatestResult();
    //boolean hasTarget = result.hasTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    //Transform3d distToTarget = target.getBestCameraToTarget();
    
    double range = PhotonUtils.calculateDistanceToTargetMeters(0.69, 0.67, Units.degreesToRadians(-9.7), 0);
    double adjustedRange = 0.24*range-0.0114;
    
    //SmartDashboard.putNumber("Distance to Target", adjustedRange);
    
    
    var autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    //An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(0.25, 0), new Translation2d(0.4, 0)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         //A 1 MEANS 10 METERS, DO MATH
    //         new Pose2d(0.6, 0, new Rotation2d(1.57)),
    //         // Pass config
    //         config);

    //Translation2d translationToTarget = PhotonUtils.estimateCameraToTargetTranslation(adjustedRange, Rotation2d.fromDegrees(-target.getYaw()));          
    
Trajectory photonTrak = TrajectoryGenerator.generateTrajectory(
      driveTrain.getPose(), 
      List.of(Robot.estimator.getEstimatedPosition().getTranslation()
      ), 
      new Pose2d(), config
      );

Trajectory photonTrak2 = TrajectoryGenerator.generateTrajectory(
        driveTrain.getPose(), 
        List.of(Robot.estimator.getEstimatedPosition().getTranslation()
        ), 
        new Pose2d(Robot.estimator.getEstimatedPosition().getX(), Robot.estimator.getEstimatedPosition().getY(), new Rotation2d(-target.getYaw())), config
        );
      

    //Trajectory trajToObject = TrajectoryGenerator.generateTrajectory(driveTrain.getPose(), null, null, null);
     
    

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            photonTrak2,
            driveTrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(photonTrak2.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
    //FollowPathWithEvents command = new FollowPathWithEvents(, Robot.Traj.getMarkers(), Robot.eventMap)
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   //return Autos.exampleAuto(m_exampleSubsystem);
  // }

