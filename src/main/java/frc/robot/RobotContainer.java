// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.climberDown;
import frc.robot.commands.climberStop;
import frc.robot.commands.climberUp;
import frc.robot.commands.doNothing;
import frc.robot.commands.intakeDefault;
import frc.robot.commands.intakeSpinAuto;
import frc.robot.commands.intakeSpit;
import frc.robot.commands.intakeStop;
import frc.robot.commands.intakeSuck;
import frc.robot.commands.moveIntakePos;
import frc.robot.commands.moveIntakePosAuto;
import frc.robot.commands.rollersStop;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climber;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final AutoGamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  public final AutoGamepad spotter = new AutoGamepad(Ports.Gamepad.OPERATOR);

  public final DriveSubsystem driveTrain = new DriveSubsystem();

  public final ArmSubsystem intake = new ArmSubsystem();

  public final Rollers rollers = new Rollers();

  public final Climber climber = new Climber();

  private Timer timer = new Timer();

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private PIDController leftPID = new PIDController(
      Constants.AutoPIDs.kP,
      Constants.AutoPIDs.kI,
      Constants.AutoPIDs.kD);
  private PIDController rightPID = new PIDController(
      Constants.AutoPIDs.kP,
      Constants.AutoPIDs.kI,
      Constants.AutoPIDs.kD);

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  PathPlannerTrajectory driveToSM = PathPlanner.loadPath("Test",
      new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  PathPlannerTrajectory driveToComm = PathPlanner.loadPath("Back",
      new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

  PathPlannerTrajectory driveToCS = PathPlanner.loadPath("onCS",
      new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @return 
   */

   private final Command balAuto = new SequentialCommandGroup(new intakeSpinAuto(rollers, 0.5), drivePath(true, "longOnCS"));

  public RobotContainer() {

    
    driveTrain.ahrs.reset();
    m_Chooser.setDefaultOption("Do Nothings", new doNothing());
    //
    //m_chooser.addOption("1 + bal", balAuto);

    SmartDashboard.putData(m_chooser);
    // Configure the trigger bindings
    SmartDashboard.putData("Left Auto PID", leftPID);
    SmartDashboard.putData("Right Auto PID", rightPID);

    intake.resetSensor();

    configureDefualtCommands();
    configureCommnads();
    PathPlannerServer.startServer(5811);
  }

  public void configureDefualtCommands() {
    driveTrain.setDefaultCommand(new DrivetrainDrive(driveTrain, driver));
    intake.setDefaultCommand(new intakeDefault(intake));
    rollers.setDefaultCommand(new rollersStop(rollers));
    climber.setDefaultCommand(new climberStop(climber));
  }

  public void configureCommnads(){
    //move intake to low position, VALUE NEEDED
    //spotter.getDPadUp().onTrue(new moveIntakePos(intake, Constants.intakePositionControl.downPos));
    //spotter.getDPadRight().onTrue(new moveIntakePos(intake, Constants.intakePositionControl.conePos));
    //move intake to back position, VALUE NEEDED
    //spotter.getDPadDown().onTrue(new moveIntakePos(intake, Constants.intakePositionControl.farBackPos));

    //spotter.getLeftButton().whileTrue(new intakeSuck(rollers));
    //spotter.getRightButton().whileTrue(new intakeSpit(rollers));
    driver.getDPadUp().whileTrue(new climberUp(climber));
    driver.getDPadDown().whileTrue(new climberDown(climber));
    
    spotter.getTopButton().whileTrue(new ParallelCommandGroup(new moveIntakePos(intake, Constants.intakePositionControl.conePos), new intakeSpit(rollers)));
    
  
    spotter.getRightButton().whileTrue(new intakeSpit(rollers));

    spotter.getBottomButton().whileTrue(new intakeSuck(rollers));

    spotter.getLeftButton().whileTrue(new ParallelCommandGroup(new moveIntakePos(intake, Constants.intakePositionControl.downPos), new intakeSuck(rollers)));
  }

//   public Command drivePath(boolean isFirstPath,String nameOfPath, DriveSubsystem driveTrain, PIDController leftPID,PIDController rightPID){
   
//     PathPlannerTrajectory path = PathPlanner.loadPath(nameOfPath, new PathConstraints(0.6, 0.3));
//     PathPlannerServer.sendActivePath(path.getStates());


//     return new SequentialCommandGroup(
//       new InstantCommand(() -> {
//         // Reset odometry for the first path you run during autoPathPlannerServer.sendActivePath(path.getStates());
//         //driveTrain.resetEncoders();
//         timer.start();
//         if(isFirstPath){
//           Pose2d e = path.getInitialPose();  
//           //Pose2d flippedPose = new Pose2d(e.getX(),e.getY(),e.getRotation().minus(Rotation2d.fromDegrees(180)));
//           //driveTrain.resetOdometry(flippedPose);
//           driveTrain.resetOdometry(e);
//         }
//       }),
//       new PPRamseteCommand(
//           path, 
//           driveTrain::getPose, // Pose supplier
//           new RamseteController(),  
//           new SimpleMotorFeedforward(0.14971, 0.0073732, 0.0092238),
//           driveTrain.kinematics, // DifferentialDriveKinematics
//           driveTrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
//           leftPID, // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
//           rightPID, // Right controller (usually the same values as left controller)
//           driveTrain::tankDriveVolts, // Voltage bicnsumer
//           false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//           driveTrain // Requires this drive subsystem
//       )
//   );
// }
public Command drivePath(boolean isFirstPath, String nameOfPath) {
  // An example command will be run in autonomous

  PathPlannerTrajectory drivePath1 = PathPlanner.loadPath(nameOfPath, new PathConstraints(1.5, 3.0));
  PathPlannerServer.sendActivePath(drivePath1.getStates());

  return new SequentialCommandGroup(
    new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if(isFirstPath){
        Pose2d e = drivePath1.getInitialPose();  
        //Pose2d flippedPose = new Pose2d(e.getX(),e.getY(),e.getRotation().minus(Rotation2d.fromDegrees(180)));
        //driveTrain.resetOdometry(flippedPose);
        driveTrain.resetOdometry(e);
      }
    }),
    new PPRamseteCommand(
        drivePath1, 
        driveTrain::getPose, // Pose supplier
        new RamseteController(),  
        new SimpleMotorFeedforward(0.14971, 0.0073732, 0.0092238),
        driveTrain.kinematics, // DifferentialDriveKinematics
        driveTrain::getWheelSpeeds, // `DifferentialDriveWheelSpeeds supplier
        leftPID, // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        rightPID, // Right controller (usually the same values as left controller)
        driveTrain::tankDriveVolts, // Voltage bicnsumer
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        driveTrain // Requires this drive subsystem
    )
);
}


  public Command getAutonomousCommand(boolean isFirstPath) {
    //TWO PIEACE AUTO
    return new SequentialCommandGroup(
        new moveIntakePosAuto(intake, Constants.intakePositionControl.downPos),
        new intakeSpinAuto(rollers, 0.5),
        new moveIntakePosAuto(intake, Constants.intakePositionControl.farBackPos),
        drivePath(true, "Test"),
        //BROKE HERE
        
            new moveIntakePosAuto(intake, Constants.intakePositionControl.downPos),
            new intakeSpinAuto(rollers, -0.5),
            new moveIntakePosAuto(intake, Constants.intakePositionControl.farBackPos),
        drivePath(false, "Back"),
        new moveIntakePos(intake, 0),
        new intakeSpit(rollers));
  
    //LOW SCORE + BALANCE
    //return new SequentialCommandGroup(new intakeSpitAuto(rollers), drivePath(true, "longOnCS"));
      
  }
}
