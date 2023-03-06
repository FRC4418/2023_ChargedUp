// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.autoCommands.defaultAuto;
import frc.robot.commands.autoCommands.twoPieaceAuto;
import frc.robot.commands.dopeSlopeCommands.armDown;
import frc.robot.commands.dopeSlopeCommands.armStop;
import frc.robot.commands.dopeSlopeCommands.armGoTo;
import frc.robot.commands.dopeSlopeCommands.armHoldAt;
import frc.robot.commands.dopeSlopeCommands.dumbArmStop;
import frc.robot.commands.manndibleCommands.ArmsCloseCone;
import frc.robot.commands.manndibleCommands.ArmsCloseCube;
import frc.robot.commands.manndibleCommands.ArmsCloseCubeAuto;
import frc.robot.commands.manndibleCommands.ArmsOpen;
import frc.robot.commands.manndibleCommands.ArmsOpenAuto;
import frc.robot.commands.manndibleCommands.Calibrate;
import frc.robot.commands.manndibleCommands.IntakePull;
import frc.robot.commands.manndibleCommands.IntakePullAuto;
import frc.robot.commands.manndibleCommands.IntakePush;
import frc.robot.commands.manndibleCommands.armsClose;
import frc.robot.commands.dopeSlopeCommands.armGoTo;
import frc.robot.commands.dopeSlopeCommands.armHoldAt;
import frc.robot.commands.miscCommands.resetOdometry;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final AutoGamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  public final AutoGamepad spotter = new AutoGamepad(Ports.Gamepad.OPERATOR);

  public final Arms mannArm = new Arms();

  public final Intake intake = new Intake();
  
  public final Vision vision = new Vision();
  
  public final DriveSubsystem driveTrain = new DriveSubsystem(vision);

  public final ArmSubsystem arm = new ArmSubsystem();

  private PIDController leftPID = new PIDController(
    Constants.AutoPIDs.kP, 
    Constants.AutoPIDs.kI,
    Constants.AutoPIDs.kD);
  private PIDController rightPID = new PIDController(
    Constants.AutoPIDs.kP, 
    Constants.AutoPIDs.kI,
    Constants.AutoPIDs.kD);

  SendableChooser<Command> m_Chooser =  new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putData("Left Auto PID", leftPID);
    SmartDashboard.putData("Right Auto PID", rightPID);

    mannArm.resetEncoder();

    SmartDashboard.putData(m_Chooser);

    PathPlannerTrajectory driveOut = PathPlanner.loadPath("CommToMarkOut", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    PathPlannerTrajectory driveIn = PathPlanner.loadPath("MarkToCommIn", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
    m_Chooser.addOption("2 pieace", new twoPieaceAuto(arm, intake, mannArm, true, leftPID, rightPID, driveTrain, driveOut, driveIn));
    
    configureDefualtCommands();
    configureCommnads();
    PathPlannerServer.startServer(5811);
    m_Chooser.setDefaultOption("Default", new defaultAuto(arm, intake, mannArm));
  }
  public void configureDefualtCommands(){
    driveTrain.setDefaultCommand(new DrivetrainDrive(driveTrain, driver));
    arm.setDefaultCommand(new armStop(arm));
  }
  public void configureCommnads(){
    driver.getDPadRight().whileTrue(new resetOdometry(driveTrain, mannArm, arm));
    driver.getRightButton().whileTrue(new armsClose(mannArm));
    driver.getLeftButton().whileTrue(new ArmsOpen(mannArm));
    //driver.getRightButton().whileTrue(new ArmsCloseCone(mannArm, 0));
    //driver.getLeftButton().whileTrue(new ArmsOpen(mannArm));
    //driver.getDPadDown().whileTrue(new dumbdArmIn(dumbArm));

    //spotter controller arm
    spotter.getDPadUp().onTrue(new armHoldAt(arm, -290000));
    spotter.getDPadLeft().onTrue(new armHoldAt(arm, -170000));
    //low position is -140000 if needed.
    spotter.getDPadDown().whileTrue(new armDown(arm));
    //spotter manndible
    spotter.getTopButton().whileTrue(new ArmsCloseCone(mannArm));
    spotter.getBottomButton().whileTrue(new ArmsCloseCube(mannArm));
    spotter.getRightButton().whileTrue(new IntakePush(intake));
    spotter.getLeftButton().whileTrue(new IntakePull(intake));
    spotter.getRightBumper().whileTrue(new Calibrate(mannArm));
    //NOT FINAL VERSION OF COMMAND
  }

  public Command visionAlign() {
    // Create a voltage constraint to ensure we don't accelerate too fast
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

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         //A 1 MEANS 10 METERS, DO MATH
    //         new Pose2d(0.5, 0, new Rotation2d(0)),
    //         // Pass config
    //         config); 
    Trajectory photonTrak = TrajectoryGenerator.generateTrajectory(
        vision.getCameraToTarget(),
        List.of(), 
        new Pose2d(
          Constants.visionTrajEndPoint.xOffset,
          Constants.visionTrajEndPoint.yOffset, 
          new Rotation2d(Units.degreesToRadians(180))),
        config
      );

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            photonTrak,
            /**
            This is the main difference between V1 and V2. V2 is using the getPose method
            which returns values from odometry as opposed to V1 which used the pose estimator
            relative to the target. The thinking behind this was that becuase we were having trouble
            with looisng the target, if we drove the trajectory using odometry instead of pose
            estimation, we wouldn't need to deal with that whole null handeling and realignment.
            **/
            driveTrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(6.0, 1.5, 2.0),
            new PIDController(6.0, 1.5, 2.0),
            // RamseteCommand passes volts to the callback
            
            driveTrain::tankDriveVolts,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(photonTrak.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));

  }

public Command getAutonomousCommand(boolean isFirstPath){
   //return new SequentialCommandGroup(drivePath(true, "Pos3Across"), visionAlign()); 
   //return drivePath(isFirstPath, "markToComm").andThen(visionAlign(), null);
  //return m_Chooser.getSelected();
  //ParallelCommandGroup mandibleStuff = new ParallelCommandGroup(new ArmsCloseCubeAuto(mannArm), new IntakePull(intake));
  //SequentialCommandGroup armStuff = new SequentialCommandGroup(new armGoTo(arm, Constants.armPositionControl.mediumPosition), new ParallelCommandGroup(new IntakePush(intake), new ArmsOpen(mannArm)));
  //return new IntakePullAuto(intake).andThen(new armGoTo(arm, Constants.armPositionControl.highPosition)).andThen(new WaitCommand(3)).andThen(new IntakePush(intake));
  //return new ParallelCommandGroup(mandibleStuff, armStuff);   
  return m_Chooser.getSelected();
}
}
