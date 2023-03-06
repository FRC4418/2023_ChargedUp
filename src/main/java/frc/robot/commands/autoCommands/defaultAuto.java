// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.dopeSlopeCommands.armGoTo;
import frc.robot.commands.manndibleCommands.ArmsCloseCone;
import frc.robot.commands.manndibleCommands.IntakePull;
import frc.robot.commands.manndibleCommands.IntakePush;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Intake;

public class defaultAuto extends CommandBase {
  /** Creates a new defaultAuto. */
  private ArmSubsystem arm;
  private Intake intake;
  private Arms mandibleArms;
  public defaultAuto(ArmSubsystem arm, Intake intake, Arms mandibleArms) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake, mandibleArms);
    this.arm = arm;
    this.intake = intake;
    this.mandibleArms = mandibleArms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ArmsCloseCone(mandibleArms).andThen(new ParallelRaceGroup(new armGoTo(arm, Constants.armPositionControl.highPosition), new IntakePull(intake)).andThen(new IntakePush(intake)));
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
