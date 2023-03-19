// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.drivePath;
import frc.robot.commands.intakeDefault;
import frc.robot.commands.intakeSpit;
import frc.robot.commands.intakeSpitAuto;
import frc.robot.commands.intakeStop;
import frc.robot.commands.intakeSuck;
import frc.robot.commands.moveIntakePos;
import frc.robot.commands.rollersStop;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Rollers;

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
  
  public final DriveSubsystem driveTrain = new DriveSubsystem();

  public final ArmSubsystem intake = new ArmSubsystem();

  public final Rollers rollers = new Rollers();

  private PIDController leftPID = new PIDController(
    Constants.AutoPIDs.kP, 
    Constants.AutoPIDs.kI,
    Constants.AutoPIDs.kD);
  private PIDController rightPID = new PIDController(
    Constants.AutoPIDs.kP, 
    Constants.AutoPIDs.kI,
    Constants.AutoPIDs.kD);

  SendableChooser<Command> m_Chooser =  new SendableChooser<>();

  PathPlannerTrajectory driveToSM = PathPlanner.loadPath("Test", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  PathPlannerTrajectory driveToComm = PathPlanner.loadPath("Back", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putData("Left Auto PID", leftPID);
    SmartDashboard.putData("Right Auto PID", rightPID);

    intake.resetSensor();

    
    configureDefualtCommands();
    configureCommnads();
    PathPlannerServer.startServer(5811);
  }
  public void configureDefualtCommands(){
    driveTrain.setDefaultCommand(new DrivetrainDrive(driveTrain, driver));
    intake.setDefaultCommand(new intakeDefault(intake));
    rollers.setDefaultCommand(new rollersStop(rollers));
    
  }
  public void configureCommnads(){
    //move intake to low position, VALUE NEEDED
    spotter.getDPadUp().onTrue(new moveIntakePos(intake, Constants.intakePositionControl.downPos));
    //move intake to back position, VALUE NEEDED
    spotter.getDPadDown().onTrue(new moveIntakePos(intake, Constants.intakePositionControl.farBackPos));

    spotter.getLeftButton().whileTrue(new intakeSuck(rollers));
    spotter.getRightButton().whileTrue(new intakeSpit(rollers));
  }


public Command getAutonomousCommand(boolean isFirstPath){ 
  return new SequentialCommandGroup(
    new moveIntakePos(intake, Constants.intakePositionControl.downPos), 
    new intakeSpitAuto(rollers), 
    new drivePath(driveTrain, driveToSM, true, leftPID, rightPID), 
    new ParallelRaceGroup(
      new moveIntakePos(intake, Constants.intakePositionControl.downPos), 
      new intakeSuck(rollers)), 
    new drivePath(driveTrain, driveToComm, false, leftPID, rightPID), 
    new intakeSpit(rollers));
}
}
