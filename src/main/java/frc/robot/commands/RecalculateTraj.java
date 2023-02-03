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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RecalculateTraj extends CommandBase {
  private static DriveSubsystem driveTrain;
  private static Vision vision;
  private Trajectory photonTrak;
  private RamseteCommand ramseteCommand;

  /** Creates a new buildTraj. */
  public RecalculateTraj(DriveSubsystem driveTrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, vision);
    this.driveTrain = driveTrain;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    photonTrak = TrajectoryGenerator.generateTrajectory(
      DriveSubsystem.estimator.getEstimatedPosition(),
      List.of(), 
      vision.getCameraToTarget(),
      driveTrain.getTrajConfig()
    );

    SmartDashboard.putNumber("Length of Traj (Seconds)", photonTrak.getTotalTimeSeconds());
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
