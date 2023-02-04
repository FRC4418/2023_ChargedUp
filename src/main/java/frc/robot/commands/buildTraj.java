// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class BuildTraj extends CommandBase {
  private DriveSubsystem driveTrain;
  private RamseteCommand ramseteCommand;
  private Traj traj;
  private Trajectory photonTrak;
  private Vision vision;
  /** Creates a new buildTraj. */
  public BuildTraj(DriveSubsystem driveTrain, Traj traj, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, traj);
    this.driveTrain = driveTrain;
    this.traj = traj;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//TODO: USE SAME .getPose in trajectory generator
    // photonTrak = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(
    //       0.0,
    //       0.0, 
    //       new Rotation2d()
    //     ), 
    //     List.of(),
    //     vision.getCameraToTarget(), 
    //     driveTrain.getTrajConfig()
    //   );

      ramseteCommand = new RamseteCommand(
        traj.photonTrak, 
        driveTrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
        new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter
        ), 
        driveTrain.kinematics, 
        driveTrain::getWheelSpeeds, 
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        driveTrain::tankDriveVolts, 
        driveTrain
      );
  
    driveTrain.resetOdometry(photonTrak.getInitialPose());
  
    ramseteCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}
