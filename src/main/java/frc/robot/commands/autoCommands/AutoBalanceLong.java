// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Ports.Drivetrain;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceLong extends CommandBase {
  /** Creates a new AutoBalance. */
  private DriveSubsystem driveTrain;
  private PathPlannerTrajectory traj;
  private PIDController leftPID;
  private PIDController rightPID;
  public AutoBalanceLong(DriveSubsystem driveTrain, PathPlannerTrajectory traj, PIDController leftPID, PIDController rightPID) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(driveTrain, traj, leftPID, rightPID);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new drivePath(driveTrain, traj, true, leftPID, rightPID).andThen(
      new balanceCommand(driveTrain));
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
