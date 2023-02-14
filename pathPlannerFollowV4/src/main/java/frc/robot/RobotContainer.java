// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DrivetrainDrive;
import frc.robot.commands.resetOdometry;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public final DriveSubsystem driveTrain = new DriveSubsystem();
  public final AutoGamepad driver = new AutoGamepad(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDefualtCommands();
    configureCommnads();

  }
  public void configureDefualtCommands(){
    driveTrain.setDefaultCommand(new DrivetrainDrive(driveTrain, driver));
  }
  public void configureCommnads(){
    driver.getDPadUp().onTrue(new resetOdometry(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(boolean isFirstPath) {
    // An example command will be run in autonomous
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(0.6, 0.2));
    
    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
      new PathConstraints(0.4, 0.2), 
      new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)), // position, heading
      new PathPoint(new Translation2d(2.0, -2.0), Rotation2d.fromDegrees(0)) // position, heading
  );

    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            driveTrain.resetOdometry(traj1.getInitialPose());
        }
      }),
      new PPRamseteCommand(
          traj1, 
          driveTrain::getPose, // Pose supplier
          new RamseteController(),
          new SimpleMotorFeedforward(0.3471, 0.98, 0.31067),
          driveTrain.kinematics, // DifferentialDriveKinematics
          driveTrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
          new PIDController(2.0, 0.1, 0.5), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(2.0, 0.1, 0.5), // Right controller (usually the same values as left controller)
          driveTrain::tankDriveVolts, // Voltage biconsumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          driveTrain // Requires this drive subsystem
      )
  );
  }
}
