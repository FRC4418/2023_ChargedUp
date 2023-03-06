// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.dopeSlopeCommands.armDownAuto;
import frc.robot.commands.dopeSlopeCommands.armGoTo;
import frc.robot.commands.manndibleCommands.ArmsCloseCone;
import frc.robot.commands.manndibleCommands.IntakePull;
import frc.robot.commands.manndibleCommands.IntakePush;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class twoPieaceAuto extends CommandBase {
  /** Creates a new twoPieaceAuto. */
  private ArmSubsystem arm;
  private Intake intake;
  private Arms mandibleArms;
  private DriveSubsystem driveTrain;
  private PIDController rightPID;
  private PIDController leftPID;
  private boolean isFirstPath;
  private PathPlannerTrajectory trajOut;
  private PathPlannerTrajectory trajIn;
  public twoPieaceAuto(ArmSubsystem arm, Intake intake, Arms mandibleArms, boolean isFirstPath, PIDController leftPid, PIDController rightPID, DriveSubsystem driveTrain, PathPlannerTrajectory trajOut, PathPlannerTrajectory trajIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake, mandibleArms, driveTrain);
    this.arm = arm;
    this.intake = intake;
    this.mandibleArms = mandibleArms;
    this.leftPID = leftPid;
    this.rightPID = rightPID;
    this.driveTrain = driveTrain;
    this.isFirstPath = isFirstPath;
    this.trajOut = trajOut;
    this.trajIn = trajIn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ArmsCloseCone(mandibleArms).andThen(
      new ParallelRaceGroup(
        new armGoTo(arm, Constants.armPositionControl.highPosition), 
        new IntakePull(intake)).andThen(
      new IntakePush(intake)).andThen(
      new ParallelCommandGroup(
        new armDownAuto(arm), 
        new drivePath(driveTrain, trajOut, isFirstPath, leftPID, rightPID)).andThen(
      new IntakePull(intake).andThen(
      new drivePath(driveTrain, trajIn, false, leftPID, rightPID).andThen(
      new defaultAuto(arm, intake, mandibleArms))))));
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
