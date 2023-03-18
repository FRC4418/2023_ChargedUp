// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class drivePath extends CommandBase {
  /** Creates a new drivePath. */
  private DriveSubsystem driveTrain;
  private PathPlannerTrajectory drivePath1;
  private boolean isFirstPath;
  private PIDController leftPID;
  private PIDController rightPID;
  private Timer timer;
  public drivePath(DriveSubsystem driveTrain, PathPlannerTrajectory traj, boolean isFirstPath, PIDController leftPid, PIDController rightPID) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.drivePath1 = traj;
    this.isFirstPath = isFirstPath;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerServer.sendActivePath(drivePath1.getStates());

    new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        timer.start();
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
          new SimpleMotorFeedforward(0.66871, 1.98, 0.61067),
          driveTrain.kinematics, // DifferentialDriveKinematics
          driveTrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
          leftPID, // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          rightPID, // Right controller (usually the same values as left controller)
          driveTrain::tankDriveVolts, // Voltage bicnsumer
          false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          driveTrain // Requires this drive subsystem
      )
  );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(drivePath1.getTotalTimeSeconds());
  }
}